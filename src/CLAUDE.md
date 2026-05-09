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


==========================================================================
RESUMEN DE MODULOS DEL SISTEMA CHITROVERGYM A INTEGRAR
==========================================================================

Estos 4 modulos provienen del proyecto Chitrovergym
(c:\Users\rodri\Documents\PlatformIO\Projects\Chitrovergym\src\) y se
migraran a este proyecto con mejoras:

- maszter, nextion, HD9BAsensor → se integran a lilyorugabasz (T-Watch)
- crawler → se integra a orugabasz (XIAO ESP32-C3)

----------------------------------------------------------------
1. MASZTER (01_Maszter/main.cpp) — cerebro del sistema
----------------------------------------------------------------
Plataforma actual: Arduino Nano (RFnano old bootloader, RF24 CE=10 CSN=9)
Rol: nodo MAESTRO de la red RF24Mesh (NODE_ID = 0)

Funciones principales:
- Recibe meditacion + 5 bandas EEG (theta, lowAlpha, highAlpha, lowBeta,
  highBeta) desde HD9BAsensor por I2C (direccion 9, 11 bytes)
- Calcula meditacion derivada de bandas EEG:
  meditCalc = constrain((theta+lowAlpha+highAlpha)/(lowBeta+highBeta) * 12.5, 1, 100)
- Maquina de estados (semaforo): INICIOS → ESPERANDO_INICALA → CALCULANDO_A
  → ESPERANDO_AB → CALCULANDO_B → MOSTRANDO_RESULTADO → ESPERANDO_SECUENCIAS
  → PROGRAMA_TERMINADO. Recolecta NUM_DATOS muestras de meditacion para dos
  estados mentales A y B, calcula promedios y los compara.
- Estadisticas con libreria Statistic: promedios, desviaciones estandar
  (statAB, statRestas, statRestasPos, statRestasNeg, statMd9), restas A-B
  positivas y negativas acumuladas.
- 4 secuencias (numSecuencias=4): la primera decide repetir, las otras 3
  generan instrucciones binarias que se suman para dar 8 valores posibles
  (instruccionfinal = parcial1 + parcial2 + parcial3) → comando crawler.
- Salidas locales: NeoPixel (chakras, GPIO 8), motor vibrador (GPIO 2),
  buzzer activo (GPIO 7) con 5 patrones de sonido.
- Protocolo RF24Mesh:
  * 'V' → rosarobot (NODE_ID 1): voltage, pwmValue, meditacion
  * 'M' → excell (NODE_ID 3): stream meditacion + velorosa
  * 'X' → nextion (NODE_ID 5): tonextion (todos los promedios, SD,
    bandas EEG, secuencia, etc.)
  * 'C' → crawler (NODE_ID 6): tocrawlersz (estado + ledsmasztocrawler)
  * Recibe 'R' (rosa), 'N' (nextion: valordenex, nummuestrasrecibida),
    'W' (crawler: pulsosz, angulosz, finpulsosz)
- Comandos desde nextion: 41=iniciar, 42=parar, 43=continuar despues de fin.

Mejora prevista al migrar a lilyorugabasz (T-Watch):
- Eliminar RF24Mesh y reemplazar por ESP-NOW al XIAO (orugabasz/crawler)
- Mantener maquina de estados, estadisticas y logica de secuencias
- La pantalla del T-Watch reemplaza a Nextion (fusion maszter+nextion)
- Buzzer/vibrador/NeoPixel pueden mapearse a hardware del T-Watch

----------------------------------------------------------------
2. NEXTION (03_nextion/main.cpp) — interfaz HMI
----------------------------------------------------------------
Plataforma actual: Arduino Nano (RFnano), conectado a display Nextion por
Serial (9600). NODE_ID = 5 en la red mesh.

Funciones principales:
- Recibe estructura demaszter por RF24Mesh ('X'): meditanow, promedioA/B,
  promediodelta absoluto/positivo/negativo, nexempezar, secudemasz,
  pulsosz, angulosz, crawlerfinalizo, desviacionA, eegTheta, meditCalc
- Renderiza valores en widgets Nextion:
  * n1.val ← med9 (meditacion actual)
  * t0..t16 ← promedios A/B, deltas, SD, theta, meditCalc (formato float)
  * n2.val ← botonoff (estado boton on/off)
- Lee toques de botones del Nextion y envia a maszter ('N'):
  fromnextiontomasz { datonextion, nummuestrastomasz }
- Comandos desde Nextion al maszter: 41=iniciar, 42=parar, 43=continuar,
  + numero de muestras configurable.

Mejora prevista al migrar a lilyorugabasz (T-Watch):
- Reemplazar el display Nextion por la pantalla TFT touch del T-Watch (LVGL)
- La logica de UI (botones, displays numericos, graficas) se reescribe
  en LVGL nativo del T-Watch
- Los datos llegan localmente (no por RF) ya que maszter vive en el mismo
  T-Watch — desaparece la estructura demaszter via mesh
- La estructura fromnextiontomasz desaparece: los toques de UI llaman
  funciones internas del maszter

----------------------------------------------------------------
3. HD9BAsensor (06_HD9BAsensor/main.cpp) — adaptador NeuroSky
----------------------------------------------------------------
Plataforma actual: Arduino Pro Micro / Leonardo (Serial1 a 57600 baud).

Funciones principales:
- Lee paquetes ThinkGear (NeuroSky MindWave) por Serial1
- Sincroniza con sync bytes 0xAA 0xAA, valida checksum (one's complement)
- Parsea paquetes:
  * 0x02 → poorQuality (calidad de senal)
  * 0x04 → attention (0-100)
  * 0x05 → meditation (0-100)
  * 0x83 → ASIC_EEG_POWER: 8 bandas de 3 bytes big-endian (delta, theta,
    lowAlpha, highAlpha, lowBeta, highBeta, lowGamma, midGamma)
- Escala bandas EEG de 24 bits a uint16 (>>8) para reducir tamano I2C
- Esclavo I2C en direccion 9, callback enviarsz() entrega 11 bytes:
  meditation(1) + theta(2) + lowAlpha(2) + highAlpha(2) + lowBeta(2) +
  highBeta(2) high-byte primero
- Pin listo9 (GPIO 4) HIGH cuando hay dato nuevo, LOW cuando el master
  termina de leerlo (handshake).
- Vibrador local (GPIO 9) que se activa si meditation > umbral (60).

Mejora prevista al migrar a lilyorugabasz (T-Watch):
- T-Watch lee NeuroSky MindWave Mobile via Bluetooth Classic SPP
  (ya existe en proyecto lilygosz: integracion BT NeuroSky)
- Eliminar I2C y handshake con pin listo9 — todo es interno al T-Watch
- Mantener el parser ThinkGear (sync, checksum, payload 0x02/0x04/0x05/0x83)
- Las bandas EEG quedan disponibles directamente para maszter sin
  conversion a uint16 (se puede usar unsigned long sin perdida)

----------------------------------------------------------------
4. CRAWLER (02_crawler/main.cpp) — robot oruga
----------------------------------------------------------------
Plataforma actual: Arduino Nano (RFnano), NODE_ID = 6 en la red mesh.

Hardware:
- 2 servos (delantero D3, trasero D2) con giro opuesto = circle turn
- Motor DC con driver DRV8835 en modo PHASE/ENABLE (PWM=GPIO 6, PHASE=GPIO 8)
- Sensor Hall analogico (A0, umbral 619) para conteo de pulsos por iman
  (12 imanes/vuelta, rueda diam 55mm → DISTANCIA_POR_PULSO calculada)
- 10 LEDs WS2812B addressables (GPIO 4)
- Buzzer activo (GPIO 7), LED respiracion (GPIO 5)

Funciones principales:
- Recibe via RF24Mesh ('C') estructura datademasz {movicrawler, ledszdato}
- Decodifica movicrawler (0..7) en 8 movimientos binarios:
  0=RE retro, 1=AI adel-izq, 2=GR cambio-grados, 3=AD adel-der,
  4=RI retro-izq, 5=PU cambio-pulsos, 6=RD retro-der, 7=AF adelante
- Logica circle turn: servos delantero y trasero giran en direcciones
  OPUESTAS para giro en radio reducido. Centro 96/90, delta ±35 (o ±17
  cuando GR alterna).
- PULSOS_OBJETIVO alterna entre 7 y 3 con comando PU. Velocidad objetivo
  20 mm/s, PWM se incrementa progresivamente hasta alcanzarla.
- Detencion automatica al llegar a PULSOS_OBJETIVO (contadorPulsos del
  sensor hall), envia ack al maszter ('W') con datatomasz {pulsosz,
  angulosz, finpulsosz=7}.
- LEDs WS2812B con codigos por ledszdato (11..36) que indican etapa de
  la secuencia y resultado SI/NO.
- Buzzer con 5 sonidos (corto, doble, triple, largo, especial — buzzer
  activo logica invertida: LOW=on, HIGH=off).
- LED respiracion fade in/out cuando motor activo.

Mejora prevista al migrar a orugabasz (XIAO ESP32-C3):
- Reemplazar RF24Mesh por ESP-NOW (recibir desde lilyorugabasz/T-Watch)
- DRV8835 sigue en modo PHASE/ENABLE (ver
  ObsidianVault\NeuroProjects\01_Chitrovergym\DRV8835_modos_control.md)
- Replantear pines GPIO al mapa del XIAO ESP32-C3:
  * Servos en pines PWM disponibles
  * MOTORPWM/MOTORphase a GPIO con LEDC (PWM hardware ESP32)
  * Hall analogico en ADC (A0 → GPIO con ADC capability)
  * WS2812B en cualquier GPIO (lib RMT del ESP32 soporta NeoPixel)
- ESP-NOW permite mas datos por paquete y latencia menor que RF24Mesh
- Mantener libreria Servo (compatible con ESP32) o migrar a ESP32Servo
- Estructura de comandos similar: {movicrawler, ledszdato} → solo cambia
  la capa de transporte
