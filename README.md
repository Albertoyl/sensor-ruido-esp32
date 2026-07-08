# Sensor de Ruido IoT — ESP32-C3

Sonómetro de bajo coste basado en ESP32-C3 y micrófono I2S MEMS (INMP441 /
ICS-4343x). Mide el nivel sonoro con ponderación A (LAeq), publica las medidas
por MQTT y ofrece una página web para su calibración.

## Autoría

Proyecto desarrollado por **Alberto Yubero López / AVUDS**
(Agrupación de Vecinos Unidos por el Derecho al Descanso y la Salud).

Copyright (c) 2026 Alberto Yubero López / AVUDS.
Distribuido bajo licencia **GNU GPL v3** (véase el archivo `LICENSE`).

## Atribución

Este proyecto se basa en el trabajo de código abierto:

- **esp32-i2s-slm** — Copyright (c) Ivan Kostoski — GPL v3
  https://github.com/ikostoski/esp32-i2s-slm
  https://hackaday.io/project/166867

De ese proyecto proceden los coeficientes de los filtros IIR (ecualización del
micrófono y ponderación A) y la arquitectura original de filtrado.

## Trabajo propio sobre esa base

Aportaciones originales de Alberto Yubero López / AVUDS:

- **Port de los filtros a aritmética de punto fijo (Q28)** para el ESP32-C3.
  El ESP32-C3 (RISC-V) no tiene unidad de coma flotante (FPU): el filtrado en
  `float` no llegaba a tiempo real (tardaba ~214 ms por cada 125 ms de audio),
  desbordaba el buffer DMA y perdía ~60% del audio, lo que generaba un suelo de
  ruido elevado, picos falsos y aparente no-linealidad. La reescritura en
  enteros redujo el filtrado a ~12 ms por bloque, resolviéndolo de raíz.
- Sistema de **calibración por página web** con promedio LAeq de 1 minuto.
- **Portal de configuración WiFi** (WiFiManager), **publicación MQTT** y
  descubrimiento **mDNS**.
- Gestión de credenciales en NVS, reinicio por doble arranque e instrumentación
  de diagnóstico (timing de ciclo/filtrado).

## Hardware

- Placa: ESP32-C3 (probado en ESP32-C3 SuperMini)
- Micrófono: INMP441 o ICS-4343x (I2S MEMS)

Conexiones del micrófono:

| Micrófono | ESP32-C3 |
|-----------|----------|
| VDD       | 3V3      |
| GND       | GND      |
| L/R       | GND      |
| SD        | GPIO4    |
| SCK       | GPIO5    |
| WS        | GPIO6    |

## Software

Librerías necesarias (Arduino IDE):

- WiFiManager (tzapu)
- PubSubClient (Nick O'Leary)
- ArduinoJson (Benoit Blanchon)

Núcleo ESP32 para Arduino 3.x (usa el driver `i2s_std.h`).

## Configuración antes de compilar

Editar el bloque de AJUSTES al inicio del `.ino`:

- `NODE_ID` — identificador único del nodo
- `MQTT_HOST`, `MQTT_PORT`, `MQTT_USER`, `MQTT_PASS` — datos del broker MQTT
- `CAL_USER`, `CAL_PASS` — usuario y clave de la página de calibración

## Licencia

GNU General Public License v3. Al basarse en un proyecto GPL v3, este trabajo
se distribuye obligatoriamente bajo la misma licencia. Cualquier redistribución
o trabajo derivado debe mantener los avisos de copyright y atribución, y
permanecer bajo GPL v3.

## Agradecimientos

Durante el desarrollo se utilizó asistencia de IA (Claude, de Anthropic) como
herramienta de apoyo para depuración y documentación. El diseño, las decisiones
técnicas, las pruebas de hardware y la autoría del proyecto corresponden a
Alberto Yubero López / AVUDS.

