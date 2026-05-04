Proyecto watchorugabasz — Comunicacion ESP-NOW entre T-Watch y XIAO ESP32-C3

Dos entornos PlatformIO en el mismo proyecto:

1. lilyorugabasz (src/lilyorugabasz/)
   - Placa: TTGO T-Watch 2020 V2 (ESP32)
   - Upload: COM13
   - Funcion: EMISOR. Muestra dos botones tactiles ON/OFF en pantalla.
     Envia comandos ESP-NOW al XIAO para controlar su LED.
   - Usa la libreria TTGO_TWatch_Library-master del proyecto lilygosz
     via lib_extra_dirs (no se duplica)

2. orugabasz (src/orugabasz/)
   - Placa: XIAO ESP32-C3 Seeed Studio (FCC: Z4T-XIAOESP32C3)
   - Upload: COM8 (ajustar segun puerto real)
   - Funcion: RECEPTOR. Al arrancar imprime su MAC por Serial.
     Recibe comandos ESP-NOW y enciende/apaga su LED (GPIO10, activo BAJO).

Flujo de primer uso:
1. Compilar y subir orugabasz al XIAO ESP32-C3
2. Abrir Serial Monitor (115200) y copiar la MAC que imprime
3. Pegar la MAC en src/lilyorugabasz/main.cpp en el array xiaoMac[]
4. Compilar y subir lilyorugabasz al T-Watch
5. Tocar los botones ON/OFF en el T-Watch para controlar el LED del XIAO

Protocolo ESP-NOW:
- Estructura MensajeOruga: { uint8_t comando }
- comando = 1 → LED ON
- comando = 0 → LED OFF
- Ambos en modo WIFI_STA, sin encriptacion, canal 0

Contexto Gapaxionsz:
Este proyecto es la base para integrar el XIAO ESP32-C3 como nodo controlable
desde el T-Watch. En el futuro, el XIAO puede servir como gateway al resto del
sistema Chitrovergym (puente entre ESP-NOW y la red RF24 mesh del crawler).
