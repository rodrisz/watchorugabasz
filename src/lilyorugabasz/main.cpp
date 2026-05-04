#define LILYGO_WATCH_2020_V2
#include <LilyGoWatch.h>
#include <WiFi.h>
#include <esp_now.h>

/*
 * lilyorugabasz — T-Watch 2020 V2
 *
 * Comunicacion BIDIRECCIONAL via ESP-NOW con el XIAO ESP32-C3 (orugabasz).
 *   - Botones tactiles ON/OFF envian comando al XIAO
 *   - Recibe ACK del XIAO y lo muestra en pantalla
 */

// ====== CAMBIAR POR LA MAC REAL DEL XIAO ESP32-C3 ======
uint8_t xiaoMac[6] = { 0xA0, 0x76, 0x4E, 0x44, 0xE8, 0x80 };
// =======================================================

#define CMD_LED_ON  1
#define CMD_LED_OFF 0

#define ACK_OK    1
#define ACK_ERROR 0

typedef struct __attribute__((packed)) {
  uint8_t comando;
  uint32_t contador;
} MensajeOruga;

typedef struct __attribute__((packed)) {
  uint8_t ack;
  uint8_t comandoEco; //prueba
  uint32_t contadorEco;
  uint8_t estadoLed;
} RespuestaOruga;

TTGOClass *watch = nullptr;
TFT_eSPI *tft = nullptr;

const int BTN_W = 180;
const int BTN_H = 70;
const int BTN_X = (240 - BTN_W) / 2;
const int BTN_ON_Y  = 40;
const int BTN_OFF_Y = 130;

uint32_t contadorEnvio = 0;
volatile bool hayNuevaRespuesta = false;
RespuestaOruga ultimaRespuesta;

String estadoEnvio = "Listo";
String estadoRx    = "---";

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  estadoEnvio = (status == ESP_NOW_SEND_SUCCESS) ? "TX OK" : "TX FALLO";
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(RespuestaOruga)) return;
  memcpy((void *)&ultimaRespuesta, data, sizeof(RespuestaOruga));
  hayNuevaRespuesta = true;
}

void dibujarBoton(int y, const char *texto, uint16_t color) {
  tft->fillRoundRect(BTN_X, y, BTN_W, BTN_H, 10, color);
  tft->drawRoundRect(BTN_X, y, BTN_W, BTN_H, 10, TFT_WHITE);
  tft->setTextColor(TFT_WHITE, color);
  tft->setTextDatum(MC_DATUM);
  tft->setTextFont(4);
  tft->drawString(texto, 120, y + BTN_H / 2);
}

void dibujarEstadoEnvio() {
  tft->fillRect(0, 205, 240, 15, TFT_BLACK);
  tft->setTextColor(TFT_CYAN, TFT_BLACK);
  tft->setTextDatum(MC_DATUM);
  tft->setTextFont(2);
  tft->drawString("TX: " + estadoEnvio, 120, 212);
}

void dibujarEstadoRx() {
  tft->fillRect(0, 222, 240, 18, TFT_BLACK);
  tft->setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft->setTextDatum(MC_DATUM);
  tft->setTextFont(2);
  tft->drawString("RX: " + estadoRx, 120, 230);
}

void dibujarInterfaz() {
  tft->fillScreen(TFT_BLACK);

  tft->setTextColor(TFT_ORANGE, TFT_BLACK);
  tft->setTextDatum(MC_DATUM);
  tft->setTextFont(2);
  tft->drawString("watchorugabasz", 120, 15);

  dibujarBoton(BTN_ON_Y,  "ON",  TFT_DARKGREEN);
  dibujarBoton(BTN_OFF_Y, "OFF", TFT_RED);
  dibujarEstadoEnvio();
  dibujarEstadoRx();
}

void enviarComando(uint8_t cmd) {
  contadorEnvio++;

  MensajeOruga msg;
  msg.comando = cmd;
  msg.contador = contadorEnvio;

  esp_err_t r = esp_now_send(xiaoMac, (uint8_t *)&msg, sizeof(msg));
  if (r == ESP_OK) {
    estadoEnvio = (cmd == CMD_LED_ON) ? "ON #" : "OFF #";
    estadoEnvio += String(contadorEnvio);
  } else {
    estadoEnvio = "Error TX";
  }
  dibujarEstadoEnvio();
}

void procesarRespuesta() {
  if (!hayNuevaRespuesta) return;
  hayNuevaRespuesta = false;

  RespuestaOruga r;
  noInterrupts();
  memcpy(&r, (const void *)&ultimaRespuesta, sizeof(RespuestaOruga));
  interrupts();

  String ok = (r.ack == ACK_OK) ? "OK" : "ERR";
  String led = r.estadoLed ? "ON" : "OFF";
  estadoRx = "ACK " + ok + " LED=" + led + " #" + String((unsigned long)r.contadorEco);
  dibujarEstadoRx();

  Serial.printf("RX ACK ack=%u cmd=%u led=%u #%lu\n",
                r.ack, r.comandoEco, r.estadoLed,
                (unsigned long)r.contadorEco);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  watch = TTGOClass::getWatch();
  watch->begin();
  watch->openBL();
  watch->setBrightness(100);

  tft = watch->tft;
  tft->setRotation(0);

  dibujarInterfaz();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  Serial.println("=== lilyorugabasz (T-Watch) ===");
  Serial.print("MAC T-Watch: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    estadoEnvio = "ESP-NOW err";
    dibujarEstadoEnvio();
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, xiaoMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Error anadiendo peer XIAO");
    estadoEnvio = "Peer err";
    dibujarEstadoEnvio();
  } else {
    Serial.println("Peer XIAO anadido");
  }
}

void loop() {
  int16_t x, y;
  if (watch->getTouch(x, y)) {
    if (x >= BTN_X && x <= BTN_X + BTN_W &&
        y >= BTN_ON_Y && y <= BTN_ON_Y + BTN_H) {
      enviarComando(CMD_LED_ON);
      delay(250);
    } else if (x >= BTN_X && x <= BTN_X + BTN_W &&
               y >= BTN_OFF_Y && y <= BTN_OFF_Y + BTN_H) {
      enviarComando(CMD_LED_OFF);
      delay(250);
    }
  }

  procesarRespuesta();
  delay(20);
}
