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
#define VIBRATE_DURATION_MS    3000UL

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
uint32_t numeroSesion    = 1;

// Maquina de estados del boton NUEVA SESION (pagina 3)
//   IDLE        → muestra "NUEVA SESION"
//   CONFIRM     → primer tap, muestra "CONFIRMAR?" durante CONFIRM_WINDOW_MS
//   WAIT_ECHO   → confirmado, esperando que A reporte numCoherencias=0
//   OK / FAIL   → resultado final, vuelve a IDLE tras RESULT_DISPLAY_MS
enum SesionBtnState { SBTN_IDLE, SBTN_CONFIRM, SBTN_WAIT_ECHO, SBTN_OK, SBTN_FAIL };
SesionBtnState sesionBtn      = SBTN_IDLE;
uint32_t       sesionBtnSince = 0;
uint32_t       seqAlPedir     = 0;     // paqueteRx.seq al enviar el reset; OK = seq nuevo + numCoh=0

#define CONFIRM_WINDOW_MS    3000UL
#define ECHO_TIMEOUT_MS      2000UL
#define RESULT_DISPLAY_MS    1500UL

// Vibracion (no bloqueante)
bool     vibrando        = false;
uint32_t vibrateStart    = 0;

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
Boton btnGuardar = { 20, 170, 200, 50 };

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

void vibrarStart() {
    if (!watch || !watch->drv) return;
    // Efecto 118 = "Long buzz for programmatic stopping" — 100%.
    // Una sola llamada → buzz continuo hasta que llamemos a stop().
    watch->drv->setWaveform(0, 118);
    watch->drv->setWaveform(1, 0);
    watch->drv->go();
    vibrando      = true;
    vibrateStart  = millis();
}

void vibrarTick(uint32_t now) {
    if (!vibrando) return;
    if (now - vibrateStart >= VIBRATE_DURATION_MS) {
        if (watch && watch->drv) watch->drv->stop();
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
    tft->setTextFont(4);
    tft->setTextColor(sesionBtn == SBTN_CONFIRM ? TFT_BLACK : TFT_WHITE, col);
    tft->drawString(label, btnGuardar.x + btnGuardar.w/2,
                    btnGuardar.y + btnGuardar.h/2);
}

void drawPage3Full() {
    tft->fillScreen(COLOR_BG);
    drawHeader("SESION", true);

    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(2);
    tft->setTextColor(COLOR_LABEL, COLOR_BG);
    tft->drawString("Sesion #",      120,  55);
    tft->drawString("Coherencias",   120, 120);

    drawSesionButton();
}

void drawPage3Update() {
    char buf[16];
    // Numero de sesion
    tft->fillRect(60, 70, 120, 32, COLOR_BG);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)numeroSesion);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(6);
    tft->setTextColor(COLOR_VALUE, COLOR_BG);
    tft->drawString(buf, 120, 85);

    // Coherencias de la sesion (vienen de A → B → C; se acumulan mientras A no se reinicia)
    tft->fillRect(60, 135, 120, 32, COLOR_BG);
    snprintf(buf, sizeof(buf), "%u", (unsigned)uiNumCoherencias);
    tft->setTextColor(COLOR_ACCENT, COLOR_BG);
    tft->drawString(buf, 120, 150);
}

// ════════════════════════════════════════════════════════════════════════════
//  TRANSICIONES
// ════════════════════════════════════════════════════════════════════════════

void setPage(uint8_t newPage) {
    currentPage    = newPage;
    needFullRedraw = true;
    // Salir de pagina 3 → cancelar cualquier confirmacion pendiente
    if (newPage != 3) sesionBtn = SBTN_IDLE;
}

void confirmarNuevaSesion() {
    // Capturar seq actual: el eco del reset llegara en un seq posterior
    seqAlPedir = paqueteRx.seq;

    numeroSesion++;
    prefs.putUInt("nses", numeroSesion);
    Serial.printf("[NVS] sesion guardada → ahora #%lu\n", (unsigned long)numeroSesion);

    // Resetear contador en A via B
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
    numeroSesion = prefs.getUInt("nses", 1);
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
