#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

/*
 * orugabasz — XIAO ESP32-C3 (Seeed Studio)
 * FCC ID: Z4T-XIAOESP32C3
 *
 * Comunicacion BIDIRECCIONAL via ESP-NOW con el T-Watch.
 *   - Recibe comando del T-Watch (ON/OFF)
 *   - Controla el LED integrado
 *   - Responde con ACK al T-Watch confirmando accion
 *
 * Al arrancar imprime su MAC address por Serial. Copia esa MAC
 * y pegala en lilyorugabasz/main.cpp en el array xiaoMac[].
 */

// LED integrado del XIAO ESP32-C3 en GPIO10 (activo BAJO)
#define LED_PIN 10

// Comandos que llegan desde el T-Watch
#define CMD_LED_ON  1
#define CMD_LED_OFF 0

// Tipos de respuesta que enviamos al T-Watch
#define ACK_OK       1
#define ACK_ERROR    0

typedef struct __attribute__((packed)) {
  uint8_t comando;
  uint32_t contador;
} MensajeOruga;

typedef struct __attribute__((packed)) {
  uint8_t ack;           // ACK_OK / ACK_ERROR
  uint8_t comandoEco;    // repetimos el comando recibido
  uint32_t contadorEco;  // repetimos el contador
  uint8_t estadoLed;     // 1 = encendido, 0 = apagado
} RespuestaOruga;

MensajeOruga mensajeRecibido;
RespuestaOruga respuesta;

uint8_t macTwatch[6];
bool peerTwatchAnadido = false;
uint8_t estadoLedActual = 0;

void anadirPeerTwatch(const uint8_t *mac) {
  if (peerTwatchAnadido) return;

  memcpy(macTwatch, mac, 6);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macTwatch, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    peerTwatchAnadido = true;
    Serial.printf("Peer T-Watch anadido: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("Error anadiendo peer T-Watch");
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("ACK enviado -> %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FALLO");
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(MensajeOruga)) {
    Serial.printf("Tamano inesperado: %d bytes\n", len);
    return;
  }

  memcpy(&mensajeRecibido, data, sizeof(mensajeRecibido));

  Serial.printf("Recibido desde %02X:%02X:%02X:%02X:%02X:%02X — cmd=%u #%lu\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                mensajeRecibido.comando, (unsigned long)mensajeRecibido.contador);

  // Ejecutar accion
  uint8_t ackStatus = ACK_OK;
  if (mensajeRecibido.comando == CMD_LED_ON) {
    digitalWrite(LED_PIN, LOW);
    estadoLedActual = 1;
    Serial.println("LED ON");
  } else if (mensajeRecibido.comando == CMD_LED_OFF) {
    digitalWrite(LED_PIN, HIGH);
    estadoLedActual = 0;
    Serial.println("LED OFF");
  } else {
    ackStatus = ACK_ERROR;
    Serial.println("Comando desconocido");
  }

  // Aprender MAC del T-Watch si aun no la tenemos
  anadirPeerTwatch(mac);

  // Enviar respuesta de vuelta
  respuesta.ack = ackStatus;
  respuesta.comandoEco = mensajeRecibido.comando;
  respuesta.contadorEco = mensajeRecibido.contador;
  respuesta.estadoLed = estadoLedActual;

  if (peerTwatchAnadido) {
    esp_err_t r = esp_now_send(macTwatch, (uint8_t *)&respuesta, sizeof(respuesta));
    if (r != ESP_OK) {
      Serial.printf("Error enviando ACK: %d\n", r);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // apagado al inicio

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  Serial.println("=== orugabasz (XIAO ESP32-C3) ===");
  Serial.print("MAC de este XIAO: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Copia esta MAC en lilyorugabasz/main.cpp");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  Serial.println("ESP-NOW listo. Bidireccional. Esperando comandos...");
}

void loop() {
  delay(1000);
}
