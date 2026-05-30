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
#define BATTERY_POLL_MS        2000UL    // refrescar lectura cada 2s

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

// ─── Multi-usuario ──────────────────────────────────────────────────────
// 8 usuarios maximo. Cada uno tiene su propio log de sesiones, contador de
// proxima sesion y umbral. El usuario activo se persiste en NVS clave "uact".
//
// Claves NVS por usuario u (0..7): slog<u>, scnt<u>, snext<u>, umbr<u>.
//
// El cronometro de sesion en curso y el contador (uiNumCoherencias, que
// viene de A) son GLOBALES — solo hay un dispositivo A.
#define USUARIO_MAX  8     // IDs 1..USUARIO_MAX en UI; internamente 0..USUARIO_MAX-1

uint8_t usuarioActivo    = 0;    // indice interno 0..7 (UI muestra +1)
uint8_t usuarioCandidato = 0;    // pagina 4: lo que el usuario esta seleccionando

// Modal de confirmacion al cambiar de usuario con sesion en curso
enum CambioUserState { CU_IDLE, CU_CONFIRM };
CambioUserState cambioUser      = CU_IDLE;
uint32_t        cambioUserSince = 0;
#define CAMBIO_USER_CONFIRM_MS  5000UL

// Cronometro de la sesion en curso: millis() cuando empezo.
// Se reinicia con cada NUEVA SESION exitosa y al boot.
uint32_t sesionStartMs = 0;

// Log circular de sesiones cerradas en NVS del T-Watch.
//   - sesion actual NO esta en el log (es la que esta en curso).
//   - log[0] = mas reciente cerrada; log[count-1] = mas antigua.
//   - HIST_LEN = 50 entradas × 8 bytes = 400 bytes, holgado para NVS.
#define HIST_LEN 50
typedef struct __attribute__((packed)) {
    uint32_t num;          // numero de sesion
    uint16_t cohFinales;   // coherencias al cerrar
    uint16_t duracionSeg;  // duracion total en segundos (max 18 h)
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

// Maquina de estados del boton BORRAR (mas simple: solo IDLE/CONFIRM/DONE)
enum BorrarBtnState { BBTN_IDLE, BBTN_CONFIRM, BBTN_DONE };
BorrarBtnState borrarBtn      = BBTN_IDLE;
uint32_t       borrarBtnSince = 0;
#define BORRAR_CONFIRM_MS     5000UL
#define BORRAR_DONE_MS        1500UL
uint32_t       seqAlPedir     = 0;     // paqueteRx.seq al enviar el reset; OK = seq nuevo + numCoh=0
uint16_t       cohAlGuardar   = 0;     // snapshot de coherencias para grabar en el log
uint16_t       duracionAlGuardar = 0;  // snapshot de duracion (segundos) para el log

#define CONFIRM_WINDOW_MS    5000UL    // ventana generosa para el 2o tap
#define ECHO_TIMEOUT_MS      2000UL
#define RESULT_DISPLAY_MS    1500UL
#define TOUCH_DEBOUNCE_MS     150UL    // debounce general (era 200)

// Vibracion (no bloqueante, patron de 3 pulsos)
bool     vibrando         = false;
uint32_t vibrateStart     = 0;
uint8_t  vibratePulsesDone = 0;
uint32_t vibrateLastPulse = 0;

// UI
uint8_t  currentPage     = 0;   // 0=menu, 1=datos, 2=umbral, 3=sesion
bool     needFullRedraw  = true;
uint32_t pageEnterMs     = 0;   // millis() al entrar a la pagina, para ignorar taps fantasma

// Bateria (lectura periodica via AXP202)
uint8_t  batteryPct      = 0;     // 0..100
bool     batteryCharging = false;
bool     batteryConn     = false;
uint32_t batteryLastRead = 0;

#define PAGE_ENTER_LOCKOUT_MS  400UL    // ignorar taps en los primeros 400ms tras cambio

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
// Pagina 0: 4 botones mas compactos para dar lugar al boton USUARIO
Boton btnP1 = { 20,  62, 200, 36 };   // DATOS
Boton btnP2 = { 20, 102, 200, 36 };   // UMBRAL
Boton btnP3 = { 20, 142, 200, 36 };   // SESION
Boton btnP4 = { 20, 182, 200, 36 };   // USUARIO

// Pagina 2 — flechas
Boton btnUp   = {  30, 100,  70, 70 };
Boton btnDown = { 140, 100,  70, 70 };

// Pagina 3
//   Layout vertical (pantalla 240x240, header 0-28):
//     35-50:  etiqueta "Sesion ACTUAL" o "N/M"
//     55-100: numero de sesion grande
//     105-150: zona de navegacion (flechas laterales + coherencias en centro)
//     165-215: dos botones lado a lado: NUEVA (izq) y BORRAR (der)
Boton btnPrevSes = {   5, 105, 55, 45 };   // flecha izq (mas reciente)
Boton btnNextSes = { 180, 105, 55, 45 };   // flecha der (mas antigua)
Boton btnGuardar = {  10, 165, 110, 50 };  // NUEVA SESION (izq)
Boton btnBorrar  = { 130, 165, 100, 50 };  // BORRAR (der)

// Pagina 4 — seleccion de usuario
Boton btnUserDown   = {   5, 105, 55, 50 };   // flecha izq → disminuir
Boton btnUserUp     = { 180, 105, 55, 50 };   // flecha der → aumentar
Boton btnUserChange = {  20, 175, 200, 45 };   // CAMBIAR
// Modal de confirmacion sobre la pagina 4
Boton btnUserConfYes = {  10, 110, 105, 60 };
Boton btnUserConfNo  = { 125, 110, 105, 60 };

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

// ─── Forward declarations (definiciones mas abajo en el archivo) ──────
static inline uint32_t safeElapsed(uint32_t now, uint32_t since);
void formatSesionTime(uint32_t ms, char *buf, size_t bufLen);

// ─── Bateria ───────────────────────────────────────────────────────────
//
//  El AXP202 expone getBattVoltage() (mV) e isChargeing(). Calculamos %
//  por voltaje porque getBattPercentage() devuelve 127 sin calibrar.

void leerBateria() {
    if (!watch || !watch->power) return;
    float mv = watch->power->getBattVoltage();
    int pct = (int)((mv - 3000.0f) / (4200.0f - 3000.0f) * 100.0f);
    if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
    batteryPct      = (uint8_t)pct;
    batteryCharging = watch->power->isChargeing();
    batteryConn     = watch->power->isBatteryConnect();
}

// Dibuja icono de pila + porcentaje. Tamaño compacto para header.
//   x, y son la esquina superior izquierda del icono (icono 28x12).
//   El porcentaje sale justo a la derecha del icono.
void drawBatteryWidget(int16_t x, int16_t y) {
    const int16_t w = 28, h = 12;
    uint16_t col;
    if (!batteryConn)        col = COLOR_LABEL;
    else if (batteryCharging) col = TFT_CYAN;
    else if (batteryPct >= 60) col = TFT_GREEN;
    else if (batteryPct >= 30) col = TFT_YELLOW;
    else                       col = TFT_RED;

    // Limpiar zona (icono + texto a la derecha)
    tft->fillRect(x, y - 1, w + 38, h + 3, COLOR_PANEL);

    // Marco de la pila
    tft->drawRect(x, y, w, h, col);
    // Punta (boton +)
    tft->fillRect(x + w, y + 3, 2, h - 6, col);

    // Relleno proporcional al %
    int fillW = (int)((batteryPct * (w - 4)) / 100);
    if (fillW > 0) tft->fillRect(x + 2, y + 2, fillW, h - 4, col);

    // Porcentaje en texto a la derecha
    char buf[12];
    snprintf(buf, sizeof(buf), "%u%%", (unsigned)batteryPct);
    tft->setTextDatum(ML_DATUM);
    tft->setTextFont(1);
    tft->setTextColor(col, COLOR_PANEL);
    tft->drawString(buf, x + w + 5, y + h/2);
}

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

    // Widget bateria (izquierda del header, ya que aqui no hay boton MENU)
    drawBatteryWidget(5, 8);

    // Indicador NeuroSky (etiqueta fija; valor se redibuja en update)
    tft->setTextDatum(ML_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("NeuroSky:", 10, 45);

    // Botones (font 2 porque ahora son mas bajitos)
    auto drawBtn = [](Boton b, const char *t, uint16_t col) {
        tft->fillRoundRect(b.x, b.y, b.w, b.h, 6, col);
        tft->drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
        tft->setTextColor(TFT_WHITE, col);
        tft->setTextDatum(MC_DATUM);
        tft->setTextFont(4);
        tft->drawString(t, b.x + b.w/2, b.y + b.h/2);
    };

    drawBtn(btnP1, "1 DATOS",   COLOR_BTN);
    drawBtn(btnP2, "2 UMBRAL",  COLOR_BTN2);
    drawBtn(btnP3, "3 SESION",  COLOR_BTN3);

    // Boton USUARIO con numero actual incluido (se redibuja en update tambien)
    char buf[16];
    snprintf(buf, sizeof(buf), "4 USUARIO %u", (unsigned)(usuarioActivo + 1));
    drawBtn(btnP4, buf, COLOR_ACCENT);
}

void drawPage0Update(uint32_t now) {
    // Batteria (se redibuja con cualquier cambio)
    drawBatteryWidget(5, 8);

    // Estado NeuroSky: solo la zona del valor (la etiqueta es fija)
    tft->fillRect(85, 35, 155, 20, COLOR_BG);

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
    tft->setTextDatum(ML_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(nc, COLOR_BG);
    tft->drawString(ns, 90, 45);
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 1 — DATOS
// ════════════════════════════════════════════════════════════════════════════

void drawPage1Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("DATOS", true);

    // Etiquetas fijas
    tft->setTextDatum(ML_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Atencion",     20,  82);
    tft->drawString("Meditacion",   20, 122);
    tft->drawString("Coherencias",  20, 180);
}

void drawPage1Update(uint32_t now) {
    char buf[12];

    // Cronometro de la sesion (arriba, centrado)
    tft->fillRect(70, 33, 100, 22, COLOR_BG);
    formatSesionTime(safeElapsed(now, sesionStartMs), buf, sizeof(buf));
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString(buf, 120, 44);

    // Atencion
    tft->fillRect(140, 65, 90, 30, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiAttention);
    tft->setTextDatum(MR_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 230, 80);

    // Meditacion
    tft->fillRect(140, 105, 90, 30, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiMeditation);
    tft->drawString(buf, 230, 120);

    // Linea separadora
    tft->drawFastHLine(15, 155, 210, COLOR_PANEL);

    // Coherencias
    tft->fillRect(140, 160, 90, 35, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiNumCoherencias);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString(buf, 230, 180);

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
        case SBTN_CONFIRM:   label = "OK?";       col = COLOR_ACCENT; break;
        case SBTN_WAIT_ECHO: label = "esperando"; col = COLOR_BTN2;   break;
        case SBTN_OK:        label = "OK";        col = COLOR_OK;     break;
        case SBTN_FAIL:      label = "FALLO";     col = COLOR_ERR;    break;
        default:             label = "NUEVA";     col = COLOR_BTN;    break;
    }
    tft->fillRoundRect(btnGuardar.x, btnGuardar.y, btnGuardar.w, btnGuardar.h, 8, col);
    tft->drawRoundRect(btnGuardar.x, btnGuardar.y, btnGuardar.w, btnGuardar.h, 8, TFT_WHITE);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(sesionBtn == SBTN_CONFIRM ? TFT_BLACK : TFT_WHITE, col);
    tft->drawString(label, btnGuardar.x + btnGuardar.w/2,
                    btnGuardar.y + btnGuardar.h/2);
}

void drawBorrarButton() {
    const char *label;
    uint16_t    col;
    switch (borrarBtn) {
        case BBTN_CONFIRM: label = "BORRAR?";  col = COLOR_ACCENT; break;
        case BBTN_DONE:    label = "BORRADO";  col = COLOR_OK;     break;
        default:           label = "BORRAR";   col = COLOR_ERR;    break;
    }
    tft->fillRoundRect(btnBorrar.x, btnBorrar.y, btnBorrar.w, btnBorrar.h, 8, col);
    tft->drawRoundRect(btnBorrar.x, btnBorrar.y, btnBorrar.w, btnBorrar.h, 8, TFT_WHITE);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(borrarBtn == BBTN_CONFIRM ? TFT_BLACK : TFT_WHITE, col);
    tft->drawString(label, btnBorrar.x + btnBorrar.w/2,
                    btnBorrar.y + btnBorrar.h/2);
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
    tft->drawString("Coherencias", 120, 115);

    drawSesionButton();
    drawBorrarButton();
}

void drawPage3Update() {
    char buf[24];

    // Determinar que sesion mostrar y si es "actual" o de log
    uint32_t mostrarNum;
    uint16_t mostrarCoh;
    uint32_t mostrarMs;        // duracion en ms (actual) o en seg*1000 (log)
    bool     esActual = (viewIdx == 0);
    if (esActual) {
        mostrarNum = sesionActual;
        mostrarCoh = uiNumCoherencias;
        mostrarMs  = safeElapsed(millis(), sesionStartMs);
    } else {
        uint8_t idx = viewIdx - 1;
        if (idx >= sesionCount) idx = 0;
        mostrarNum = sesionLog[idx].num;
        mostrarCoh = sesionLog[idx].cohFinales;
        mostrarMs  = (uint32_t)sesionLog[idx].duracionSeg * 1000UL;
    }

    // Etiqueta "Sesion N/M" + duracion (cabecera de la vista de log)
    tft->fillRect(0, 32, 240, 18, COLOR_BG);
    char timeBuf[12];
    formatSesionTime(mostrarMs, timeBuf, sizeof(timeBuf));
    if (esActual) {
        snprintf(buf, sizeof(buf), "ACTUAL  %s", timeBuf);
    } else {
        snprintf(buf, sizeof(buf), "%u/%u  %s",
                 (unsigned)viewIdx, (unsigned)sesionCount, timeBuf);
    }
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(esActual ? COLOR_ACCENT : COLOR_LABEL, COLOR_BG);
    tft->drawString(buf, 120, 42);

    // Numero de sesion grande
    tft->fillRect(50, 55, 140, 50, COLOR_BG);
    snprintf(buf, sizeof(buf), "#%lu", (unsigned long)mostrarNum);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 80);

    // Flechas (con estado habilitado/deshabilitado)
    // Izquierda DISMINUYE (va a mas antigua): habilitada si quedan mas antiguas.
    // Derecha   AUMENTA   (va a mas reciente): habilitada si no estamos en actual.
    bool puedeIzq = (viewIdx < sesionCount);
    bool puedeDer = (viewIdx > 0);
    drawArrowSes(btnPrevSes, true,  puedeIzq);
    drawArrowSes(btnNextSes, false, puedeDer);

    // Coherencias (valor grande) + tasa cpm (linea pequena debajo).
    // Zona total entre flechas: y=125-160, ancho 110 (x=65-175).
    tft->fillRect(65, 125, 110, 35, COLOR_BG);

    // Valor de coherencias
    snprintf(buf, sizeof(buf), "%u", (unsigned)mostrarCoh);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(esActual ? COLOR_ACCENT : COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 138);

    // Tasa: coherencias por minuto. Requiere al menos 5s de sesion para
    // evitar valores absurdos al arrancar (4 coh en 2s → 120 cpm).
    uint32_t totalSec = mostrarMs / 1000;
    if (totalSec >= 5) {
        float cpm = (float)mostrarCoh * 60.0f / (float)totalSec;
        snprintf(buf, sizeof(buf), "%.2f/min", cpm);
    } else {
        snprintf(buf, sizeof(buf), "-- /min");
    }
    tft->setTextFont(1);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString(buf, 120, 156);
}

// ════════════════════════════════════════════════════════════════════════════
//  PAGINA 4 — SELECCION DE USUARIO
// ════════════════════════════════════════════════════════════════════════════

void drawPage4Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("USUARIO", true);

    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Selecciona usuario", 120, 50);
}

void drawPage4Update() {
    char buf[16];

    // Numero grande del candidato
    tft->fillRect(50, 70, 140, 60, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)(usuarioCandidato + 1));
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(7);
    bool esActivo = (usuarioCandidato == usuarioActivo);
    tft->setTextColor(esActivo ? COLOR_OK : COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 100);

    // Sub-texto "activo" si coincide
    tft->fillRect(40, 138, 160, 18, COLOR_BG);
    if (esActivo) {
        tft->setTextFont(2);
        tft->setTextColor(COLOR_OK, COLOR_BG);
        tft->drawString("activo", 120, 148);
    }

    // Flechas
    bool puedeDown = (usuarioCandidato > 0);
    bool puedeUp   = (usuarioCandidato < USUARIO_MAX - 1);
    drawArrowSes(btnUserDown, true,  puedeDown);
    drawArrowSes(btnUserUp,   false, puedeUp);

    // Boton CAMBIAR (deshabilitado si ya es el activo)
    uint16_t btnCol = esActivo ? COLOR_PANEL : COLOR_BTN;
    tft->fillRoundRect(btnUserChange.x, btnUserChange.y,
                       btnUserChange.w, btnUserChange.h, 8, btnCol);
    tft->drawRoundRect(btnUserChange.x, btnUserChange.y,
                       btnUserChange.w, btnUserChange.h, 8,
                       esActivo ? COLOR_LABEL : TFT_WHITE);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->setTextColor(esActivo ? COLOR_LABEL : TFT_WHITE, btnCol);
    tft->drawString("CAMBIAR",
                    btnUserChange.x + btnUserChange.w / 2,
                    btnUserChange.y + btnUserChange.h / 2);
}

// Modal de confirmacion: se dibuja encima de la pagina 4 cuando se intenta
// cambiar mientras hay coherencias acumuladas (sesion en curso).
void drawCambioUserModal() {
    // Panel
    tft->fillRect(0, 60, 240, 180, COLOR_BG);
    tft->drawRect(5, 65, 230, 170, COLOR_ACCENT);

    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString("Hay sesion en curso", 120, 80);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Cambiar de usuario", 120, 98);

    // Botones SI / NO
    tft->fillRoundRect(btnUserConfYes.x, btnUserConfYes.y,
                       btnUserConfYes.w, btnUserConfYes.h, 8, COLOR_ERR);
    tft->drawRoundRect(btnUserConfYes.x, btnUserConfYes.y,
                       btnUserConfYes.w, btnUserConfYes.h, 8, TFT_WHITE);
    tft->setTextColor(TFT_WHITE, COLOR_ERR);
    tft->setTextFont(4);
    tft->drawString("DESCARTAR",
                    btnUserConfYes.x + btnUserConfYes.w / 2,
                    btnUserConfYes.y + btnUserConfYes.h / 2);

    tft->fillRoundRect(btnUserConfNo.x, btnUserConfNo.y,
                       btnUserConfNo.w, btnUserConfNo.h, 8, COLOR_BTN2);
    tft->drawRoundRect(btnUserConfNo.x, btnUserConfNo.y,
                       btnUserConfNo.w, btnUserConfNo.h, 8, TFT_WHITE);
    tft->setTextColor(TFT_WHITE, COLOR_BTN2);
    tft->drawString("CANCELAR",
                    btnUserConfNo.x + btnUserConfNo.w / 2,
                    btnUserConfNo.y + btnUserConfNo.h / 2);

    tft->setTextFont(1);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("DESCARTAR borra la sesion en curso", 120, 215);
}

// ════════════════════════════════════════════════════════════════════════════
//  TRANSICIONES
// ════════════════════════════════════════════════════════════════════════════

// ─── Cronometro de sesion ──────────────────────────────────────────────

// Formatea ms transcurridos en "MM:SS" (hasta 99:59) o "HH:MM:SS" si supera.
// buf debe ser >= 10 chars.
void formatSesionTime(uint32_t ms, char *buf, size_t bufLen) {
    uint32_t totalSec = ms / 1000;
    uint32_t mm = totalSec / 60;
    uint32_t ss = totalSec % 60;
    if (mm < 100) {
        snprintf(buf, bufLen, "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);
    } else {
        uint32_t hh = mm / 60;
        mm = mm % 60;
        snprintf(buf, bufLen, "%lu:%02lu:%02lu",
                 (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
    }
}

// ─── Log de sesiones (NVS) — claves por usuario ─────────────────────────

// Genera "<base><u>" en buf (ej "slog3"). buf debe ser >= 8 chars.
static void makeKey(char *buf, const char *base, uint8_t u) {
    snprintf(buf, 8, "%s%u", base, (unsigned)u);
}

// Carga log + proxima sesion + umbral del usuario u en variables globales.
void cargarDatosUsuario(uint8_t u) {
    char k[8];

    makeKey(k, "scnt", u);
    sesionCount = (uint8_t) prefs.getUChar(k, 0);
    if (sesionCount > HIST_LEN) sesionCount = 0;

    makeKey(k, "snext", u);
    sesionActual = prefs.getUInt(k, 1);

    if (sesionCount > 0) {
        makeKey(k, "slog", u);
        size_t expected = sizeof(SesionEntry) * sesionCount;
        size_t got = prefs.getBytes(k, sesionLog, expected);
        if (got != expected) {
            sesionCount = 0;   // corrupcion → empezar limpio
        }
    }

    makeKey(k, "umbr", u);
    umbralLocal = (uint8_t) prefs.getUChar(k, 60);
    if (umbralLocal < 40 || umbralLocal > 99) umbralLocal = 60;

    Serial.printf("[NVS] usuario %u cargado: %u sesiones, proxima=#%lu, umbral=%u\n",
                  (unsigned)(u + 1), (unsigned)sesionCount,
                  (unsigned long)sesionActual, (unsigned)umbralLocal);
}

void guardarLogSesiones() {
    char k[8];
    makeKey(k, "scnt", usuarioActivo);
    prefs.putUChar(k, sesionCount);
    makeKey(k, "snext", usuarioActivo);
    prefs.putUInt(k, sesionActual);
    if (sesionCount > 0) {
        makeKey(k, "slog", usuarioActivo);
        prefs.putBytes(k, sesionLog, sizeof(SesionEntry) * sesionCount);
    }
}

void guardarUmbralUsuario() {
    char k[8];
    makeKey(k, "umbr", usuarioActivo);
    prefs.putUChar(k, umbralLocal);
}

// Borra TODAS las sesiones guardadas del usuario activo y reinicia la
// numeracion a #1. No toca al dispositivo A ni a otros usuarios.
void borrarHistorialUsuario() {
    char k[8];

    sesionCount  = 0;
    sesionActual = 1;
    memset(sesionLog, 0, sizeof(sesionLog));

    makeKey(k, "scnt", usuarioActivo);
    prefs.putUChar(k, 0);
    makeKey(k, "snext", usuarioActivo);
    prefs.putUInt(k, 1);
    makeKey(k, "slog", usuarioActivo);
    prefs.remove(k);   // libera bytes en NVS

    viewIdx = 0;       // volver a vista actual al borrar
    Serial.printf("[NVS] historial usuario %u borrado\n",
                  (unsigned)(usuarioActivo + 1));
}

// Cambia el usuario activo SIN guardar la sesion en curso.
// Carga su log, su proxima sesion y su umbral. Reinicia el cronometro y
// envia el nuevo umbral a A para que recalcule coherencias con el umbral
// del usuario nuevo. Tambien resetea el contador en A.
void cambiarAUsuario(uint8_t nuevo) {
    if (nuevo >= USUARIO_MAX) return;
    if (nuevo == usuarioActivo) return;

    usuarioActivo = nuevo;
    prefs.putUChar("uact", usuarioActivo);
    cargarDatosUsuario(usuarioActivo);

    // Reiniciar cronometro y resetear contador en A
    sesionStartMs = millis();
    enviarResetAB();
    enviarUmbralAB();   // notifica el umbral del usuario nuevo a A

    Serial.printf("[USR] cambiado a usuario %u\n", (unsigned)(usuarioActivo + 1));
}

// Anade una entrada al frente del log (mas reciente), desplazando las demas.
void anadirSesionAlLog(uint32_t num, uint16_t coh, uint16_t duracionSeg) {
    uint8_t copyLen = sesionCount;
    if (copyLen >= HIST_LEN) copyLen = HIST_LEN - 1;   // descarta la mas antigua
    for (int8_t i = copyLen; i > 0; i--) {
        sesionLog[i] = sesionLog[i - 1];
    }
    sesionLog[0].num         = num;
    sesionLog[0].cohFinales  = coh;
    sesionLog[0].duracionSeg = duracionSeg;
    if (sesionCount < HIST_LEN) sesionCount++;
    guardarLogSesiones();
}

void setPage(uint8_t newPage) {
    currentPage    = newPage;
    needFullRedraw = true;
    pageEnterMs    = millis();
    // Salir de pagina 3 → cancelar confirmaciones + volver a vista "actual"
    if (newPage != 3) {
        sesionBtn = SBTN_IDLE;
        borrarBtn = BBTN_IDLE;
        viewIdx   = 0;
    }
    // Entrar a pagina 4 → candidato = usuario activo. Salir → cancelar modal.
    if (newPage == 4) {
        usuarioCandidato = usuarioActivo;
        cambioUser       = CU_IDLE;
    } else {
        cambioUser = CU_IDLE;
    }
}

void confirmarNuevaSesion() {
    // Snapshot del contador y duracion actual: se grabaran SOLO si A confirma.
    cohAlGuardar      = uiNumCoherencias;
    uint32_t elapsedSec = safeElapsed(millis(), sesionStartMs) / 1000;
    duracionAlGuardar = (elapsedSec > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsedSec;
    seqAlPedir        = paqueteRx.seq;

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

// Diferencia segura: si `since` es posterior a `now` (clock skew dentro del
// mismo loop), devuelve 0 en lugar de un underflow uint32_t.
static inline uint32_t safeElapsed(uint32_t now, uint32_t since) {
    return (now >= since) ? (now - since) : 0;
}

void tickSesionBtn(uint32_t now) {
    switch (sesionBtn) {
        case SBTN_CONFIRM:
            if (safeElapsed(now, sesionBtnSince) >= CONFIRM_WINDOW_MS) {
                sesionBtn = SBTN_IDLE;
                drawSesionButton();
            }
            break;
        case SBTN_WAIT_ECHO:
            // Eco exitoso: llego un paquete POSTERIOR al envio con numCoherencias=0
            if (paqueteRx.seq > seqAlPedir && uiNumCoherencias == 0) {
                // Confirmado por A → recien ahora persistimos la sesion cerrada
                // y avanzamos el contador. Si timeout-eamos no se toca nada.
                anadirSesionAlLog(sesionActual, cohAlGuardar, duracionAlGuardar);
                sesionActual++;
                guardarLogSesiones();
                Serial.printf("[NVS] sesion #%lu cerrada: %u coh, %u s; proxima=#%lu\n",
                              (unsigned long)(sesionActual - 1),
                              (unsigned)cohAlGuardar,
                              (unsigned)duracionAlGuardar,
                              (unsigned long)sesionActual);
                // Reiniciar cronometro de la sesion nueva
                sesionStartMs  = now;
                needFullRedraw = true;   // refresca pagina entera (numeros nuevos)
                sesionBtn      = SBTN_OK;
                sesionBtnSince = now;
                drawSesionButton();
            } else if (safeElapsed(now, sesionBtnSince) >= ECHO_TIMEOUT_MS) {
                sesionBtn      = SBTN_FAIL;
                sesionBtnSince = now;
                drawSesionButton();
            }
            break;
        case SBTN_OK:
        case SBTN_FAIL:
            if (safeElapsed(now, sesionBtnSince) >= RESULT_DISPLAY_MS) {
                sesionBtn = SBTN_IDLE;
                drawSesionButton();
            }
            break;
        default:
            break;
    }
}

// Doble-tap del boton BORRAR del historial del usuario actual.
void onBorrarTap() {
    uint32_t now = millis();
    switch (borrarBtn) {
        case BBTN_IDLE:
            borrarBtn      = BBTN_CONFIRM;
            borrarBtnSince = now;
            drawBorrarButton();
            break;
        case BBTN_CONFIRM:
            borrarHistorialUsuario();
            borrarBtn      = BBTN_DONE;
            borrarBtnSince = now;
            needFullRedraw = true;   // los numeros volvieron a 0
            drawBorrarButton();
            break;
        default:
            break;
    }
}

void tickBorrarBtn(uint32_t now) {
    switch (borrarBtn) {
        case BBTN_CONFIRM:
            if (safeElapsed(now, borrarBtnSince) >= BORRAR_CONFIRM_MS) {
                borrarBtn = BBTN_IDLE;
                drawBorrarButton();
            }
            break;
        case BBTN_DONE:
            if (safeElapsed(now, borrarBtnSince) >= BORRAR_DONE_MS) {
                borrarBtn = BBTN_IDLE;
                drawBorrarButton();
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

    // NVS: cargar usuario activo y sus datos (log + proxima sesion + umbral).
    prefs.begin("watchcoh", false);
    usuarioActivo = prefs.getUChar("uact", 0);
    if (usuarioActivo >= USUARIO_MAX) usuarioActivo = 0;
    cargarDatosUsuario(usuarioActivo);

    espnowInit();

    // Lectura inicial de bateria
    leerBateria();
    batteryLastRead = millis();

    // Cronometro: la "sesion en curso" arranca con el boot
    sesionStartMs = millis();

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
    // Lockout tras cambio de pagina: evita que un mismo toque "atraviese"
    // el cambio de UI y dispare dos acciones (ej. P0 boton SESION en
    // y=170-215 solapa con P3 NUEVA SESION en y=185-223).
    if (millis() - pageEnterMs < PAGE_ENTER_LOCKOUT_MS) {
        return;
    }

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
            else if (btnP4.dentro(tx, ty)) setPage(4);
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
            } else if (btnBorrar.dentro(tx, ty)) {
                onBorrarTap();
            } else if (btnPrevSes.dentro(tx, ty)) {
                // Flecha izq → DISMINUIR: ir a sesion mas antigua (idx mayor)
                if (viewIdx < sesionCount) {
                    viewIdx++;
                    drawPage3Update();
                }
            } else if (btnNextSes.dentro(tx, ty)) {
                // Flecha der → AUMENTAR: ir a sesion mas reciente (idx menor;
                //              0 = actual)
                if (viewIdx > 0) {
                    viewIdx--;
                    drawPage3Update();
                }
            }
            break;

        case 4:
            if (cambioUser == CU_CONFIRM) {
                // Modal activo: solo SI / NO responden
                if (btnUserConfYes.dentro(tx, ty)) {
                    // DESCARTAR: aplicar el cambio
                    cambiarAUsuario(usuarioCandidato);
                    cambioUser     = CU_IDLE;
                    needFullRedraw = true;
                } else if (btnUserConfNo.dentro(tx, ty)) {
                    cambioUser     = CU_IDLE;
                    needFullRedraw = true;
                }
            } else {
                // Modo normal
                if (btnUserDown.dentro(tx, ty)) {
                    if (usuarioCandidato > 0) {
                        usuarioCandidato--;
                        drawPage4Update();
                    }
                } else if (btnUserUp.dentro(tx, ty)) {
                    if (usuarioCandidato < USUARIO_MAX - 1) {
                        usuarioCandidato++;
                        drawPage4Update();
                    }
                } else if (btnUserChange.dentro(tx, ty)) {
                    if (usuarioCandidato == usuarioActivo) {
                        // No hacer nada: ya es el activo
                    } else if (uiNumCoherencias > 0) {
                        // Sesion en curso → pedir confirmacion
                        cambioUser      = CU_CONFIRM;
                        cambioUserSince = millis();
                        drawCambioUserModal();
                    } else {
                        // Sin sesion → cambiar directo
                        cambiarAUsuario(usuarioCandidato);
                        needFullRedraw = true;
                    }
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

    // 2b. Lectura periodica de bateria (no bloqueante)
    if (now - batteryLastRead >= BATTERY_POLL_MS) {
        batteryLastRead = now;
        leerBateria();
    }

    // 3. Touch
    int16_t tx, ty;
    static uint32_t lastTouchMs = 0;
    if (watch->getTouch(tx, ty)) {
        if (now - lastTouchMs > TOUCH_DEBOUNCE_MS) {
            lastTouchMs = now;
            handleTouch(tx, ty);
        }
    }

    // 4. Debounce TX umbral
    if (umbralLocalDirty && (now - umbralLastChange >= UMBRAL_TX_DEBOUNCE_MS)) {
        enviarUmbralAB();
        guardarUmbralUsuario();
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
            case 4:
                drawPage4Full();
                if (cambioUser == CU_CONFIRM) drawCambioUserModal();
                break;
        }
        lastUiUpdate = 0;   // forzar update inmediato
    }

    // 5b. Tick maquina de estados del boton NUEVA SESION (solo pagina 3).
    //     IMPORTANTE: refrescar millis() porque handleTouch() pudo asignar
    //     sesionBtnSince con un valor posterior al `now` capturado al inicio
    //     del loop. Sin refresco, (now - sesionBtnSince) hace underflow
    //     uint32_t → 0xFFFFFFFF y el timeout dispara siempre.
    if (currentPage == 3) {
        uint32_t mnow = millis();
        tickSesionBtn(mnow);
        tickBorrarBtn(mnow);
    }

    // 5c. Timeout del modal de cambio de usuario (pagina 4)
    if (currentPage == 4 && cambioUser == CU_CONFIRM) {
        if (safeElapsed(millis(), cambioUserSince) >= CAMBIO_USER_CONFIRM_MS) {
            cambioUser     = CU_IDLE;
            needFullRedraw = true;
        }
    }

    if (now - lastUiUpdate >= 250) {
        lastUiUpdate = now;
        drawLinkStatus(now);
        switch (currentPage) {
            case 0: drawPage0Update(now); break;
            case 1: drawPage1Update(now); break;
            case 2: drawPage2Update();    break;
            case 3: drawPage3Update();    break;
            case 4:
                // En modal no actualizamos (es estatico hasta que se cierre)
                if (cambioUser != CU_CONFIRM) drawPage4Update();
                break;
        }
    }

    delay(15);
}
