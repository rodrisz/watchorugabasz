/*
 * ============================================================================
 *  watchcoherenciasz — DISPOSITIVO C
 *  LilyGo T-Watch 2020 V2
 * ============================================================================
 *  HMI del sistema Mindcoherenciasz. Recibe datos por ESP-NOW desde el
 *  dispositivo B (puentesz). No habla nunca con A directamente.
 *
 *    A (Mindcoherenciasz) ←─ I2C ─→ B (puentesz) ←─ ESP-NOW ─→ C (este)
 *
 *  Cuatro paginas:
 *    Pagina 0 (MENU):
 *      - Indicador NeuroSky CONECTADO / DESCONECTADO (segun A)
 *      - Tres botones grandes: PAG 1, PAG 2, PAG 3
 *
 *    Pagina 1 (DATOS):
 *      - Atencion, Meditacion, contador de coherencias
 *      - Vibra 3 s cuando llega una coherencia nueva (nuevoEvento=1)
 *      - Boton MENU arriba a la izquierda
 *
 *    Pagina 2 (CALIBRAR UMBRAL):
 *      - Valor 40..99 con dos flechas (UP/DOWN, paso 1)
 *      - Al cambiar el valor, envia TIPO_UMBRAL a B (debounce 300 ms)
 *      - Muestra umbral activo eco que reporta A
 *      - Boton MENU arriba a la izquierda
 *
 *    Pagina 3 (SESION):
 *      - Numero de sesion + contador de coherencias actuales
 *      - Boton "GUARDAR SESION" → incrementa # sesion en NVS, resetea contador
 *      - Boton MENU arriba a la izquierda
 *
 *  Persistencia: Preferences (NVS) guarda numeroSesion y ultimo umbral.
 * ============================================================================
 */

#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ════════════════════════════════════════════════════════════════════════════
//  CONFIG ESP-NOW
// ════════════════════════════════════════════════════════════════════════════

// MAC de B (puentesz). Capturada del monitor serial al arrancar B.
uint8_t puenteMAC[6] = { 0x5C, 0x01, 0x3B, 0x34, 0x93, 0x1C };

#define ESPNOW_CHANNEL    1
#define LINK_TIMEOUT_MS   3000UL    // si no llega paquete en 3s → link DOWN
#define UMBRAL_TX_DEBOUNCE_MS  300UL

// Vibracion "Eco Mindful": 3 pulsos de intensidad decreciente espaciados
// por pausas de silencio. Total ~2.4 s; el banner UI permanece 3 s.
#define VIBRATE_BANNER_MS      3000UL
#define VIBRATE_PULSE_INTERVAL_MS  800UL
#define VIBRATE_PULSES_TOTAL   3

// ════════════════════════════════════════════════════════════════════════════
//  PROTOCOLO ESP-NOW (debe coincidir con puentesz)
// ════════════════════════════════════════════════════════════════════════════

typedef struct __attribute__((packed)) {
    uint8_t  btConnected;
    uint8_t  poorSignal;
    uint8_t  attention;
    uint8_t  meditation;
    uint16_t numCoherencias;
    uint8_t  nuevoEvento;
    uint8_t  umbralActivo;
    uint32_t seq;
} PaqueteAC;

#define TIPO_UMBRAL   0x01
#define TIPO_RESET    0x02

typedef struct __attribute__((packed)) {
    uint8_t tipo;
    uint8_t valor;
} PaqueteCA;

// ════════════════════════════════════════════════════════════════════════════
//  ESTADO COMPARTIDO (rellenado por ISR ESP-NOW)
// ════════════════════════════════════════════════════════════════════════════

volatile bool      hayNuevoPaquete = false;
PaqueteAC          paqueteRx       = {0, 200, 0, 0, 0, 0, 60, 0};
volatile uint32_t  lastRxMs        = 0;
volatile uint32_t  ultimoEventoMs  = 0;     // millis() del ultimo nuevoEvento=1

// Cache local (acceso seguro desde loop)
uint8_t  uiBtConnected   = 0;
uint8_t  uiPoorSignal    = 200;
uint8_t  uiAttention     = 0;
uint8_t  uiMeditation    = 0;
uint16_t uiNumCoherencias= 0;
uint8_t  uiUmbralActivo  = 60;

// Umbral local (lo edita el usuario en pagina 2)
uint8_t  umbralLocal     = 60;
bool     umbralLocalDirty= false;
uint32_t umbralLastChange= 0;

// Sesion (pagina 3)
Preferences prefs;

// Log circular de sesiones cerradas en NVS del T-Watch.
//   - sesion actual NO esta en el log (es la que esta en curso).
//   - log[0] = mas reciente cerrada; log[count-1] = mas antigua.
//   - HIST_LEN = 50 entradas × 8 bytes = 400 bytes, holgado para NVS.
#define HIST_LEN 50
typedef struct __attribute__((packed)) {
    uint32_t num;          // numero de sesion
    uint16_t cohFinales;   // coherencias al cerrar
    uint16_t reservado;    // alineacion + uso futuro (duracion/timestamp)
} SesionEntry;

SesionEntry sesionLog[HIST_LEN] = {0};
uint8_t     sesionCount   = 0;      // cuantas entradas validas hay (0..HIST_LEN)
uint32_t    sesionActual  = 1;      // numero de la sesion en curso
uint8_t     viewIdx       = 0;      // 0 = actual; 1..count = guardadas (mas reciente → mas antigua)

// Maquina de estados del boton NUEVA SESION (pagina 3)
//   IDLE        → muestra "NUEVA SESION"
//   CONFIRM     → primer tap, muestra "CONFIRMAR?" durante CONFIRM_WINDOW_MS
//   WAIT_ECHO   → confirmado, esperando que A reporte numCoherencias=0
//   OK / FAIL   → resultado final, vuelve a IDLE tras RESULT_DISPLAY_MS
enum SesionBtnState { SBTN_IDLE, SBTN_CONFIRM, SBTN_WAIT_ECHO, SBTN_OK, SBTN_FAIL };
SesionBtnState sesionBtn      = SBTN_IDLE;
uint32_t       sesionBtnSince = 0;
uint32_t       seqAlPedir     = 0;     // paqueteRx.seq al enviar el reset; OK = seq nuevo + numCoh=0
uint16_t       cohAlGuardar   = 0;     // snapshot de coherencias para grabar en el log

#define CONFIRM_WINDOW_MS    3000UL
#define ECHO_TIMEOUT_MS      2000UL
#define RESULT_DISPLAY_MS    1500UL

// Vibracion (no bloqueante, patron de 3 pulsos)
bool     vibrando         = false;
uint32_t vibrateStart     = 0;
uint8_t  vibratePulsesDone = 0;
uint32_t vibrateLastPulse = 0;

// UI
uint8_t  currentPage     = 0;   // 0=menu, 1=datos, 2=umbral, 3=sesion
bool     needFullRedraw  = true;

// ════════════════════════════════════════════════════════════════════════════
//  HARDWARE T-WATCH
// ════════════════════════════════════════════════════════════════════════════

TTGOClass *watch = nullptr;
TFT_eSPI  *tft   = nullptr;

// ════════════════════════════════════════════════════════════════════════════
//  COLORES Y LAYOUT
// ════════════════════════════════════════════════════════════════════════════

#define COLOR_BG       TFT_BLACK
#define COLOR_PANEL    0x10A2
#define COLOR_ACCENT   TFT_ORANGE
#define COLOR_OK       TFT_GREEN
#define COLOR_ERR      TFT_RED
#define COLOR_LABEL    TFT_LIGHTGREY
#define COLOR_VALUE    TFT_WHITE
#define COLOR_BTN      TFT_DARKGREEN
#define COLOR_BTN2     TFT_NAVY
#define COLOR_BTN3     TFT_PURPLE

#define HEADER_H       28

// ════════════════════════════════════════════════════════════════════════════
//  BOTONES
// ════════════════════════════════════════════════════════════════════════════

struct Boton {
    int16_t x, y, w, h;
    inline bool dentro(int16_t tx, int16_t ty) const {
        return tx >= x && tx < x + w && ty >= y && ty < y + h;
    }
};

Boton btnMenu     = { 5, 5, 60, HEADER_H };

// Pagina 0
Boton btnP1 = { 20,  60, 200, 45 };
Boton btnP2 = { 20, 115, 200, 45 };
Boton btnP3 = { 20, 170, 200, 45 };

// Pagina 2 — flechas
Boton btnUp   = {  30, 100,  70, 70 };
Boton btnDown = { 140, 100,  70, 70 };

// Pagina 3
//   Layout vertical (pantalla 240x240, header 0-28):
//     35-55:   etiqueta "Sesion ACTUAL" o "N/M"
//     65-115:  numero de sesion grande
//     125-175: zona de navegacion (flechas laterales + coherencias en centro)
//     180-218: boton NUEVA SESION
Boton btnPrevSes = {   5, 125, 55, 50 };   // flecha izq (mas reciente)
Boton btnNextSes = { 180, 125, 55, 50 };   // flecha der (mas antigua)
Boton btnGuardar = {  20, 185, 200, 38 };  // NUEVA SESION

// ════════════════════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACKS (ISR-context: minimo trabajo)
// ════════════════════════════════════════════════════════════════════════════

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(PaqueteAC)) return;
    PaqueteAC tmp;
    memcpy(&tmp, data, sizeof(PaqueteAC));
    paqueteRx = tmp;
    lastRxMs  = millis();
    if (tmp.nuevoEvento) ultimoEventoMs = lastRxMs;
    hayNuevoPaquete = true;
}

void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
    // No critico; el reintento sucede por debounce/TX periodico del usuario.
    (void)mac; (void)status;
}

// ════════════════════════════════════════════════════════════════════════════
//  ENVIO TIPO_UMBRAL → B
// ════════════════════════════════════════════════════════════════════════════

void enviarUmbralAB() {
    PaqueteCA p;
    p.tipo  = TIPO_UMBRAL;
    p.valor = umbralLocal;
    esp_now_send(puenteMAC, (uint8_t *)&p, sizeof(p));
    Serial.printf("[ESPNOW TX] umbral=%u\n", (unsigned)umbralLocal);
}

void enviarResetAB() {
    PaqueteCA p;
    p.tipo  = TIPO_RESET;
    p.valor = 0;
    esp_now_send(puenteMAC, (uint8_t *)&p, sizeof(p));
    Serial.println("[ESPNOW TX] reset coherencias");
}

// ════════════════════════════════════════════════════════════════════════════
//  VIBRACION (no bloqueante)
// ════════════════════════════════════════════════════════════════════════════

// Efectos DRV2605 por pulso, intensidad decreciente:
//   pulso 0 → Strong Buzz 100%   (efecto 14): firme y claro, ~1s
//   pulso 1 → Strong Click 100%  (efecto 1):  confirmacion clara
//   pulso 2 → Medium Click 60%   (efecto 23): eco perceptible que cierra
static const uint8_t VIBRATE_PATTERN[VIBRATE_PULSES_TOTAL] = { 14, 1, 23 };

static void firePulse(uint8_t idx) {
    if (!watch || !watch->drv) return;
    watch->drv->setWaveform(0, VIBRATE_PATTERN[idx]);
    watch->drv->setWaveform(1, 0);   // end of sequence
    watch->drv->go();
}

void vibrarStart() {
    if (!watch || !watch->drv) return;
    vibrando          = true;
    vibrateStart      = millis();
    vibrateLastPulse  = vibrateStart;
    vibratePulsesDone = 1;
    firePulse(0);
}

void vibrarTick(uint32_t now) {
    if (!vibrando) return;

    // Disparar pulsos siguientes con la cadencia ergonomica
    if (vibratePulsesDone < VIBRATE_PULSES_TOTAL &&
        now - vibrateLastPulse >= VIBRATE_PULSE_INTERVAL_MS) {
        firePulse(vibratePulsesDone);
        vibratePulsesDone++;
        vibrateLastPulse = now;
    }

    // Terminar cuando ya pasaron los 3 pulsos + tiempo del banner
    if (vibratePulsesDone >= VIBRATE_PULSES_TOTAL &&
        now - vibrateStart >= VIBRATE_BANNER_MS) {
        vibrando = false;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  DIBUJO COMUN
// ════════════════════════════════════════════════════════════════════════════

void drawHeader(const char *titulo, bool showMenuBtn) {
    tft->fillRect(0, 0, 240, HEADER_H, COLOR_PANEL);
    tft->drawFastHLine(0, HEADER_H, 240, COLOR_ACCENT);

    if (showMenuBtn) {
        tft->fillRoundRect(btnMenu.x, btnMenu.y, btnMenu.w, btnMenu.h, 4, COLOR_ACCENT);
        tft->setTextColor(TFT_BLACK, COLOR_ACCENT);
        tft->setTextDatum(MC_DATUM);
        tft->setTextFont(2);
        tft->drawString("MENU", btnMenu.x + btnMenu.w/2, btnMenu.y + btnMenu.h/2);
    }

    tft->setTextColor(TFT_WHITE, COLOR_PANEL);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->drawString(titulo, 120, HEADER_H / 2);
}

void drawLinkStatus(uint32_t now) {
    bool linkUp = (lastRxMs != 0) && (now - lastRxMs < LINK_TIMEOUT_MS);
    tft->fillRect(180, 4, 56, 20, COLOR_PANEL);
    tft->setTextDatum(MR_DATUM);
    tft->setTextFont(1);
    tft->setTextColor(linkUp ? COLOR_OK : COLOR_ERR, COLOR_PANEL);
    tft->drawString(linkUp ? "LINK OK" : "LINK--", 234, HEADER_H / 2);
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 0 — MENU
// ════════════════════════════════════════════════════════════════════════════

void drawPage0Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("watchcoherenciasz", false);

    // Indicador NeuroSky
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("NeuroSky:", 120, 45);

    // Botones
    auto drawBtn = [](Boton b, const char *t, uint16_t col) {
        tft->fillRoundRect(b.x, b.y, b.w, b.h, 8, col);
        tft->drawRoundRect(b.x, b.y, b.w, b.h, 8, TFT_WHITE);
        tft->setTextColor(TFT_WHITE, col);
        tft->setTextDatum(MC_DATUM);
        tft->setTextFont(4);
        tft->drawString(t, b.x + b.w/2, b.y + b.h/2);
    };

    drawBtn(btnP1, "1 DATOS",   COLOR_BTN);
    drawBtn(btnP2, "2 UMBRAL",  COLOR_BTN2);
    drawBtn(btnP3, "3 SESION",  COLOR_BTN3);
}

void drawPage0Update(uint32_t now) {
    // Estado NeuroSky (puede cambiar)
    tft->fillRect(0, 28, 240, 28, COLOR_BG);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("NeuroSky:", 80, 45);

    bool linkUp = (lastRxMs != 0) && (now - lastRxMs < LINK_TIMEOUT_MS);
    const char *ns;
    uint16_t nc;
    if (!linkUp) {
        ns = "SIN PUENTE";
        nc = COLOR_ERR;
    } else if (uiBtConnected) {
        ns = "CONECTADO";
        nc = COLOR_OK;
    } else {
        ns = "DESCONECTADO";
        nc = COLOR_ERR;
    }
    tft->setTextColor(nc, COLOR_BG);
    tft->drawString(ns, 170, 45);
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 1 — DATOS
// ════════════════════════════════════════════════════════════════════════════

void drawPage1Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("DATOS", true);

    tft->setTextDatum(ML_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Atencion",     20,  60);
    tft->drawString("Meditacion",   20, 110);
    tft->drawString("Coherencias",  20, 175);
}

void drawPage1Update(uint32_t now) {
    char buf[12];

    // Atencion
    tft->fillRect(140, 45, 90, 30, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiAttention);
    tft->setTextDatum(MR_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 230, 60);

    // Meditacion
    tft->fillRect(140, 95, 90, 30, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiMeditation);
    tft->drawString(buf, 230, 110);

    // Linea separadora
    tft->drawFastHLine(15, 150, 210, COLOR_PANEL);

    // Coherencias
    tft->fillRect(140, 155, 90, 35, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiNumCoherencias);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString(buf, 230, 175);

    // Banner "COHERENCIA!" mientras vibra
    tft->fillRect(0, 200, 240, 28, COLOR_BG);
    if (vibrando) {
        tft->fillRect(0, 200, 240, 28, COLOR_ACCENT);
        tft->setTextColor(TFT_BLACK, COLOR_ACCENT);
        tft->setTextDatum(MC_DATUM);
        tft->setTextFont(4);
        tft->drawString("COHERENCIA!", 120, 214);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 2 — CALIBRAR UMBRAL
// ════════════════════════════════════════════════════════════════════════════

void drawArrow(Boton b, bool up, uint16_t col) {
    tft->fillRoundRect(b.x, b.y, b.w, b.h, 8, col);
    tft->drawRoundRect(b.x, b.y, b.w, b.h, 8, TFT_WHITE);
    int cx = b.x + b.w/2;
    int cy = b.y + b.h/2;
    if (up) {
        tft->fillTriangle(cx, cy-18, cx-20, cy+14, cx+20, cy+14, TFT_WHITE);
    } else {
        tft->fillTriangle(cx, cy+18, cx-20, cy-14, cx+20, cy-14, TFT_WHITE);
    }
}

void drawPage2Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("UMBRAL", true);

    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Umbral coherenciasz", 120, 50);

    drawArrow(btnDown, false, COLOR_BTN2);
    drawArrow(btnUp,   true,  COLOR_BTN);

    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(1);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("activo en A:", 120, 195);
}

void drawPage2Update() {
    char buf[12];
    // Valor local grande en el centro
    tft->fillRect(95, 75, 50, 24, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)umbralLocal);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(umbralLocalDirty ? COLOR_ACCENT : COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 87);

    // Eco del umbral activo que reporta A
    tft->fillRect(50, 205, 140, 20, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiUmbralActivo);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(COLOR_OK, COLOR_BG);
    tft->drawString(buf, 120, 215);
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 3 — SESION
// ════════════════════════════════════════════════════════════════════════════

void drawSesionButton() {
    const char *label;
    uint16_t    col;
    switch (sesionBtn) {
        case SBTN_CONFIRM:   label = "CONFIRMAR?";  col = COLOR_ACCENT; break;
        case SBTN_WAIT_ECHO: label = "esperando..."; col = COLOR_BTN2;  break;
        case SBTN_OK:        label = "OK";           col = COLOR_OK;     break;
        case SBTN_FAIL:      label = "FALLO";        col = COLOR_ERR;    break;
        default:             label = "NUEVA SESION"; col = COLOR_BTN;    break;
    }
    tft->fillRoundRect(btnGuardar.x, btnGuardar.y, btnGuardar.w, btnGuardar.h, 8, col);
    tft->drawRoundRect(btnGuardar.x, btnGuardar.y, btnGuardar.w, btnGuardar.h, 8, TFT_WHITE);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(sesionBtn == SBTN_CONFIRM ? TFT_BLACK : TFT_WHITE, col);
    tft->drawString(label, btnGuardar.x + btnGuardar.w/2,
                    btnGuardar.y + btnGuardar.h/2);
}

void drawArrowSes(const Boton &b, bool left, bool habilitado) {
    uint16_t col = habilitado ? COLOR_BTN2 : COLOR_PANEL;
    tft->fillRoundRect(b.x, b.y, b.w, b.h, 6, col);
    tft->drawRoundRect(b.x, b.y, b.w, b.h, 6,
                       habilitado ? TFT_WHITE : COLOR_LABEL);
    int cx = b.x + b.w/2;
    int cy = b.y + b.h/2;
    uint16_t triCol = habilitado ? TFT_WHITE : COLOR_LABEL;
    if (left) {
        tft->fillTriangle(cx-12, cy, cx+10, cy-15, cx+10, cy+15, triCol);
    } else {
        tft->fillTriangle(cx+12, cy, cx-10, cy-15, cx-10, cy+15, triCol);
    }
}

void drawPage3Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("SESION", true);

    // Etiqueta "Coherencias" centrada entre las flechas
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Coherencias", 120, 132);

    drawSesionButton();
}

void drawPage3Update() {
    char buf[24];

    // Determinar que sesion mostrar y si es "actual" o de log
    uint32_t mostrarNum;
    uint16_t mostrarCoh;
    bool     esActual = (viewIdx == 0);
    if (esActual) {
        mostrarNum = sesionActual;
        mostrarCoh = uiNumCoherencias;
    } else {
        uint8_t idx = viewIdx - 1;
        if (idx >= sesionCount) idx = 0;
        mostrarNum = sesionLog[idx].num;
        mostrarCoh = sesionLog[idx].cohFinales;
    }

    // Etiqueta "Sesion N de M" (cabecera de la vista de log)
    tft->fillRect(0, 35, 240, 22, COLOR_BG);
    if (esActual) {
        snprintf(buf, sizeof(buf), "Sesion ACTUAL");
    } else {
        snprintf(buf, sizeof(buf), "Sesion %u/%u",
                 (unsigned)viewIdx, (unsigned)sesionCount);
    }
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(esActual ? COLOR_ACCENT : COLOR_LABEL, COLOR_BG);
    tft->drawString(buf, 120, 46);

    // Numero de sesion grande
    tft->fillRect(50, 65, 140, 50, COLOR_BG);
    snprintf(buf, sizeof(buf), "#%lu", (unsigned long)mostrarNum);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 90);

    // Flechas (con estado habilitado/deshabilitado)
    bool puedeIzq = (viewIdx > 0);                          // ir hacia mas reciente (actual)
    bool puedeDer = (viewIdx < sesionCount);                // ir hacia mas antigua
    drawArrowSes(btnPrevSes, true,  puedeIzq);
    drawArrowSes(btnNextSes, false, puedeDer);

    // Coherencias (valor grande, color segun fuente). Solo cubre el espacio entre flechas.
    tft->fillRect(65, 145, 110, 32, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)mostrarCoh);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(esActual ? COLOR_ACCENT : COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 160);
}

// ════════════════════════════════════════════════════════════════════════════
//  TRANSICIONES
// ════════════════════════════════════════════════════════════════════════════

// ─── Log de sesiones (NVS) ──────────────────────────────────────────────

void cargarLogSesiones() {
    sesionCount  = (uint8_t) prefs.getUChar("scnt", 0);
    if (sesionCount > HIST_LEN) sesionCount = 0;
    sesionActual = prefs.getUInt("snext", 1);
    if (sesionCount > 0) {
        size_t expected = sizeof(SesionEntry) * sesionCount;
        size_t got = prefs.getBytes("slog", sesionLog, expected);
        if (got != expected) {
            sesionCount = 0;   // corrupcion → empezar limpio
        }
    }
    Serial.printf("[NVS] log: %u sesiones, proxima=#%lu\n",
                  (unsigned)sesionCount, (unsigned long)sesionActual);
}

void guardarLogSesiones() {
    prefs.putUChar("scnt", sesionCount);
    prefs.putUInt ("snext", sesionActual);
    if (sesionCount > 0) {
        prefs.putBytes("slog", sesionLog, sizeof(SesionEntry) * sesionCount);
    }
}

// Anade una entrada al frente del log (mas reciente), desplazando las demas.
void anadirSesionAlLog(uint32_t num, uint16_t coh) {
    uint8_t copyLen = sesionCount;
    if (copyLen >= HIST_LEN) copyLen = HIST_LEN - 1;   // descarta la mas antigua
    for (int8_t i = copyLen; i > 0; i--) {
        sesionLog[i] = sesionLog[i - 1];
    }
    sesionLog[0].num        = num;
    sesionLog[0].cohFinales = coh;
    sesionLog[0].reservado  = 0;
    if (sesionCount < HIST_LEN) sesionCount++;
    guardarLogSesiones();
}

void setPage(uint8_t newPage) {
    currentPage    = newPage;
    needFullRedraw = true;
    // Salir de pagina 3 → cancelar confirmacion + volver a vista "actual"
    if (newPage != 3) {
        sesionBtn = SBTN_IDLE;
        viewIdx   = 0;
    }
}

void confirmarNuevaSesion() {
    // Snapshot del contador actual: lo guardaremos en el log SOLO si A confirma el reset.
    cohAlGuardar = uiNumCoherencias;
    seqAlPedir   = paqueteRx.seq;

    // Pedir reset a A via B (todavia no incrementamos sesionActual ni grabamos
    // en NVS: si A no responde queremos quedarnos en la sesion actual).
    enviarResetAB();

    sesionBtn      = SBTN_WAIT_ECHO;
    sesionBtnSince = millis();
    drawSesionButton();
}

void onSesionTap() {
    uint32_t now = millis();
    switch (sesionBtn) {
        case SBTN_IDLE:
            sesionBtn      = SBTN_CONFIRM;
            sesionBtnSince = now;
            drawSesionButton();
            break;
        case SBTN_CONFIRM:
            confirmarNuevaSesion();
            break;
        // En WAIT_ECHO / OK / FAIL ignoramos taps al boton
        default:
            break;
    }
}

void tickSesionBtn(uint32_t now) {
    switch (sesionBtn) {
        case SBTN_CONFIRM:
            if (now - sesionBtnSince >= CONFIRM_WINDOW_MS) {
                sesionBtn = SBTN_IDLE;
                drawSesionButton();
            }
            break;
        case SBTN_WAIT_ECHO:
            // Eco exitoso: llego un paquete POSTERIOR al envio con numCoherencias=0
            if (paqueteRx.seq > seqAlPedir && uiNumCoherencias == 0) {
                // Confirmado por A → recien ahora persistimos la sesion cerrada
                // y avanzamos el contador. Si timeout-eamos no se toca nada.
                anadirSesionAlLog(sesionActual, cohAlGuardar);
                sesionActual++;
                guardarLogSesiones();
                Serial.printf("[NVS] sesion #%lu cerrada con %u coherencias; proxima=#%lu\n",
                              (unsigned long)(sesionActual - 1),
                              (unsigned)cohAlGuardar,
                              (unsigned long)sesionActual);
                needFullRedraw = true;   // refresca pagina entera (numeros nuevos)
                sesionBtn      = SBTN_OK;
                sesionBtnSince = now;
                drawSesionButton();
            } else if (now - sesionBtnSince >= ECHO_TIMEOUT_MS) {
                sesionBtn      = SBTN_FAIL;
                sesionBtnSince = now;
                drawSesionButton();
            }
            break;
        case SBTN_OK:
        case SBTN_FAIL:
            if (now - sesionBtnSince >= RESULT_DISPLAY_MS) {
                sesionBtn = SBTN_IDLE;
                drawSesionButton();
            }
            break;
        default:
            break;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ESP-NOW INIT
// ════════════════════════════════════════════════════════════════════════════

void espnowInit() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("[ESPNOW] MAC propia (C): ");
    Serial.println(WiFi.macAddress());
    Serial.printf("[ESPNOW] Peer puente: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  puenteMAC[0], puenteMAC[1], puenteMAC[2],
                  puenteMAC[3], puenteMAC[4], puenteMAC[5]);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] init FAILED");
        return;
    }
    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_register_send_cb(onEspNowSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, puenteMAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("[ESPNOW] peer B registrado");
    } else {
        Serial.println("[ESPNOW] ERROR registrando peer B");
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== watchcoherenciasz (T-Watch 2020 V2) ===");

    watch = TTGOClass::getWatch();
    watch->begin();
    watch->openBL();
    watch->setBrightness(150);

    tft = watch->tft;
    tft->setRotation(0);
    tft->fillScreen(COLOR_BG);

    // DRV2605 (motor haptico): habilitar + configurar libreria + modo trigger
    watch->enableDrv2650();
    if (watch->drv) {
        watch->drv->selectLibrary(1);
        watch->drv->setMode(DRV2605_MODE_INTTRIG);
    }

    // NVS
    prefs.begin("watchcoh", false);
    cargarLogSesiones();
    umbralLocal  = (uint8_t)prefs.getUChar("umbr", 60);
    if (umbralLocal < 40 || umbralLocal > 99) umbralLocal = 60;

    espnowInit();

    // Splash 1s
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString("watchcoherenciasz", 120, 110);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("esperando puente...", 120, 140);
    delay(1000);

    setPage(0);
}

// ════════════════════════════════════════════════════════════════════════════
//  TOUCH HANDLER
// ════════════════════════════════════════════════════════════════════════════

void handleTouch(int16_t tx, int16_t ty) {
    // MENU global (paginas 1, 2, 3)
    if (currentPage != 0 && btnMenu.dentro(tx, ty)) {
        setPage(0);
        return;
    }

    switch (currentPage) {
        case 0:
            if      (btnP1.dentro(tx, ty)) setPage(1);
            else if (btnP2.dentro(tx, ty)) setPage(2);
            else if (btnP3.dentro(tx, ty)) setPage(3);
            break;

        case 2:
            if (btnUp.dentro(tx, ty)) {
                if (umbralLocal < 99) {
                    umbralLocal++;
                    umbralLocalDirty  = true;
                    umbralLastChange  = millis();
                    drawPage2Update();
                }
            } else if (btnDown.dentro(tx, ty)) {
                if (umbralLocal > 40) {
                    umbralLocal--;
                    umbralLocalDirty  = true;
                    umbralLastChange  = millis();
                    drawPage2Update();
                }
            }
            break;

        case 3:
            if (btnGuardar.dentro(tx, ty)) {
                onSesionTap();
            } else if (btnPrevSes.dentro(tx, ty)) {
                // Flecha izq → vista mas reciente (idx menor; 0 = actual)
                if (viewIdx > 0) {
                    viewIdx--;
                    drawPage3Update();
                }
            } else if (btnNextSes.dentro(tx, ty)) {
                // Flecha der → vista mas antigua (idx mayor, hasta sesionCount)
                if (viewIdx < sesionCount) {
                    viewIdx++;
                    drawPage3Update();
                }
            }
            break;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // 1. Refrescar cache desde el ultimo paquete RX
    if (hayNuevoPaquete) {
        hayNuevoPaquete  = false;
        uiBtConnected    = paqueteRx.btConnected;
        uiPoorSignal     = paqueteRx.poorSignal;
        uiAttention      = paqueteRx.attention;
        uiMeditation     = paqueteRx.meditation;
        uiNumCoherencias = paqueteRx.numCoherencias;
        uiUmbralActivo   = paqueteRx.umbralActivo;

        // Disparar vibracion en evento nuevo
        if (paqueteRx.nuevoEvento && !vibrando) {
            vibrarStart();
        }
    }

    // 2. Vibracion tick
    vibrarTick(now);

    // 3. Touch
    int16_t tx, ty;
    static uint32_t lastTouchMs = 0;
    if (watch->getTouch(tx, ty)) {
        if (now - lastTouchMs > 200) {     // debounce simple
            lastTouchMs = now;
            handleTouch(tx, ty);
        }
    }

    // 4. Debounce TX umbral
    if (umbralLocalDirty && (now - umbralLastChange >= UMBRAL_TX_DEBOUNCE_MS)) {
        enviarUmbralAB();
        prefs.putUChar("umbr", umbralLocal);
        umbralLocalDirty = false;
    }

    // 5. Render
    static uint32_t lastUiUpdate = 0;
    if (needFullRedraw) {
        needFullRedraw = false;
        switch (currentPage) {
            case 0: drawPage0Full(); break;
            case 1: drawPage1Full(); break;
            case 2: drawPage2Full(); break;
            case 3: drawPage3Full(); break;
        }
        lastUiUpdate = 0;   // forzar update inmediato
    }

    // 5b. Tick maquina de estados del boton NUEVA SESION (solo pagina 3)
    if (currentPage == 3) {
        tickSesionBtn(now);
    }

    if (now - lastUiUpdate >= 250) {
        lastUiUpdate = now;
        drawLinkStatus(now);
        switch (currentPage) {
            case 0: drawPage0Update(now); break;
            case 1: drawPage1Update(now); break;
            case 2: drawPage2Update();    break;
            case 3: drawPage3Update();    break;
        }
    }

    delay(15);
}
