/**
 * ============================================================================
 *  Sensor de Ruido IoT v1
 *  Sonómetro (LAeq, ponderación A) sobre ESP32-C3 con micrófono I2S MEMS
 * ============================================================================
 *
 *  Copyright (c) 2026 Alberto Yubero López / AVUDS
 *  (Agrupación de Vecinos Unidos por el Derecho al Descanso y la Salud)
 *
 *  Este programa es software libre: puede redistribuirse y/o modificarse
 *  bajo los términos de la Licencia Pública General de GNU (GPL) versión 3,
 *  publicada por la Free Software Foundation.
 *
 *  Se distribuye con la esperanza de que sea útil, pero SIN NINGUNA GARANTÍA;
 *  ni siquiera la garantía implícita de COMERCIABILIDAD o IDONEIDAD PARA UN
 *  PROPÓSITO PARTICULAR. Véase la Licencia Pública General de GNU para más
 *  detalles. Debería haber recibido una copia de la GPL v3 junto con este
 *  programa (archivo LICENSE); si no, véase <https://www.gnu.org/licenses/>.
 *
 * ----------------------------------------------------------------------------
 *  ATRIBUCIÓN
 * ----------------------------------------------------------------------------
 *  Los coeficientes de los filtros IIR (ecualización del micrófono y
 *  ponderación A) y la arquitectura original de filtrado están basados en el
 *  proyecto de código abierto:
 *
 *      esp32-i2s-slm  —  Copyright (c) Ivan Kostoski  —  GPL v3
 *      https://github.com/ikostoski/esp32-i2s-slm
 *      https://hackaday.io/project/166867
 *
 *  Trabajo original de Alberto Yubero López / AVUDS sobre esa base:
 *    - Port de los filtros a aritmética de punto fijo (Q28) para el ESP32-C3,
 *      que carece de FPU (el filtrado en coma flotante no llegaba a tiempo
 *      real y desbordaba el DMA).
 *    - Sistema de calibración por página web con promedio LAeq de 1 minuto.
 *    - Portal de configuración WiFi, publicación MQTT y descubrimiento mDNS.
 *    - Gestión de credenciales, reinicio por doble arranque e instrumentación.
 *
 * ============================================================================
 *  ESP32-C3 + micrófono I2S MEMS (INMP441 / ICS-4343x)
 *
 *  Conexiones del micrófono I2S:
 *    VDD → 3V3
 *    GND → GND
 *    L/R → GND
 *    SD  → GPIO4
 *    SCK → GPIO5
 *    WS  → GPIO6
 *
 *  Librerías necesarias:
 *    - WiFiManager  (tzapu)
 *    - PubSubClient (Nick O'Leary)
 *    - ArduinoJson  (Benoit Blanchon)
 *
 *  Configuración mínima antes de compilar: revisar el bloque de AJUSTES
 *  (NODE_ID, credenciales MQTT, usuario/clave de calibración).
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <driver/i2s_std.h>
#include <Preferences.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <nvs_flash.h>   // para el borrado opcional de NVS (ver setup)

// ── AJUSTES: CAMBIAR ANTES DE COMPILAR ────────────────
// Identificador único de este nodo (p. ej. "nodo-1", "salon", "calle").
#define NODE_ID  "nodo-1"

static i2s_chan_handle_t rx_handle = NULL;



// ── MQTT ─────────────────────────────────────────────
// Sustituir por los datos de tu broker MQTT.
const char* MQTT_HOST = "192.168.1.100";   // IP o dominio del broker
const int   MQTT_PORT = 1883;
const char* MQTT_USER = "usuario";          // usuario del broker (o "" si no usa auth)
const char* MQTT_PASS = "contrasena";       // contraseña del broker

const int PUBLISH_MS  = 2000;
#define   RESET_PIN   0

// ── PINES I2S ─────────────────────────────────────────
#define I2S_WS   6
#define I2S_SCK  5
#define I2S_SD   4
#define I2S_PORT I2S_NUM_0

// ── SONÓMETRO ─────────────────────────────────────────
#define SAMPLE_RATE    48000
#define SAMPLE_BITS    32
#define SAMPLES_SHORT  (SAMPLE_RATE / 8)   // 6000 muestras = 125ms
#define SAMPLES_LEQ    (SAMPLE_RATE * 1)   // 1 segundo

#define MIC_BITS        24
#define MIC_SENSITIVITY -26.0
#define MIC_REF_DB       94.0
#define MIC_OVERLOAD_DB  116.0
#define MIC_NOISE_DB     10.0
#define MIC_OFFSET_DEFAULT 3.0103   // valor de fábrica según proyecto original ikostoski/esp32-i2s-slm

// El offset de calibración ahora es una variable que se guarda en flash (NVS)
// y se puede ajustar desde la página web de calibración del sensor.
double MIC_OFFSET_DB = MIC_OFFSET_DEFAULT;

// Credenciales de la página de calibración (HTTP Basic Auth)
const char* CAL_USER = "admin";
const char* CAL_PASS = "calibrar";   // cambiar por seguridad

// Con muestras normalizadas a -1.0..+1.0, MIC_REF_AMPL no lleva el factor entero
const double MIC_REF_AMPL = pow(10.0, MIC_SENSITIVITY / 20.0);


// ── FILTROS IIR (PUNTO FIJO Q28) ──────────────────────
//
// IMPORTANTE: el ESP32-C3 (RISC-V) NO tiene FPU. Las operaciones float se
// emulan por software (~50 ciclos cada una) y el filtrado float a 48 kHz
// NO llegaba a tiempo real: 214 ms de CPU por cada 125 ms de audio, con lo
// que el DMA se desbordaba y se perdía ~60% del audio. Las discontinuidades
// resultantes generaban el suelo de ruido elevado, los picos falsos y la
// aparente "no linealidad" que veíamos en TODAS las placas y micrófonos.
//
// Solución: filtros en aritmética entera (punto fijo Q28, Direct Form I),
// ~30x más rápida. Validado en Python contra la referencia float:
// diferencia de RMS 0.00000 dB, ruido de cuantización -69 dB, estable.
//
// Los coeficientes son los mismos de ikostoski/esp32-i2s-slm, convertidos
// a enteros Q28: int32(round(coef * 2^28)). Todas las etapas tienen b0 = 1.
// Las ganancias escalares (INMP441_gain * Aw_gain) salen del bucle por
// muestra y se aplican UNA vez por bloque (conmutan con filtros lineales):
//   G_TOTAL = 1.00197834654696 * 0.169994948147430 = 0.170331257066098

#define FQ 28
const double FILTER_G_TOTAL  = 0.170331257066098;
const double FILTER_G_TOTAL2 = 0.029012737133717;   // G_TOTAL^2, para sum_sqr

// ── PONDERACIÓN TEMPORAL "FAST" (τ=125ms, IEC 61672-1) ────
// El "pico máximo" se calculaba como la muestra cruda de mayor amplitud
// dentro de un bloque de 125ms, sin ningún filtrado. Una sola muestra
// anómala (glitch de EMI, autoruido del micrófono) disparaba un "pico"
// alto aunque no hubiera un evento sonoro real.
//
// La normativa (IEC 61672-1, base de las ordenanzas de ruido) no mide así:
// define Lmax/LAFmax como el máximo de una envolvente de potencia con
// ponderación temporal "Fast", constante de tiempo τ=125ms, es decir un
// filtro paso-bajo exponencial continuo sobre la señal al cuadrado.
// Un evento de 1 muestra pesa ~1/6000 en esa media exponencial (no mueve
// nada); un evento sostenido (varios cientos de ms) sí eleva la
// envolvente de forma perceptible. Esto separa eventos reales de
// artefactos del sensor.
//
// alpha = dt/tau = (1/48000) / 0.125 = 1/6000, en Q28 (mismo formato que
// los filtros IIR, coste extra de CPU marginal: una multiplicación int64
// más por muestra).
const int64_t ALPHA_FAST_Q28 = 44739;   // round((1.0/6000.0) * (1LL<<28))

// ── INSTRUMENTACIÓN DE DIAGNÓSTICO ────────────────────
// Pon esto a 0 antes de un despliegue definitivo/desatendido: evita el
// Serial.printf de [TIMING] y el riesgo de que el USB-CDC nativo de
// algunos ESP32 se comporte de forma rara sin un host leyendo el puerto.
// Ponlo a 1 mientras estés validando con el monitor serie abierto.
#define DEBUG_TIMING 1

struct biquad_i_t {          // coeficientes Q28; b0 = 1.0 implícito
  int32_t b1, b2, a1, a2;
};

// Ecualizador INMP441 — 2º orden, datasheet del micrófono
const biquad_i_t INMP441_sos = { -533359899, 264935924, -535576653, 267142757 };

// Ponderación A — 3 etapas para 48 kHz (error <0.05 dB vs IEC 61672)
const biquad_i_t Aw_sos[3] = {
  { -536943379,  268508084,  284774703,  44020045 },
  { 1170143396,  829788395, -324382754,  73327708 },
  { -190402084,  -78039202, -532104078, 263683771 },
};

// Estados DF1 por etapa: {x1, x2, y1, y2}. 4 etapas.
int32_t filtro_st[4][4] = {{0}};

// Envolvente de potencia "Fast" (τ=125ms), continua muestra a muestra.
// IMPORTANTE: NO se resetea por bloque, solo al iniciar/reiniciar el
// sensor -- es un filtro continuo, igual que en un sonómetro real.
int64_t env_fast_i = 0;

void resetFiltros() {
  memset(filtro_st, 0, sizeof(filtro_st));
  env_fast_i = 0;
}

// Biquad en punto fijo, Direct Form I (la forma recomendada en entero:
// los estados son entradas/salidas reales, sin ganancia interna oculta).
static inline int32_t biquad_i(int32_t x, int32_t* st, const biquad_i_t& c) {
  int64_t acc = ((int64_t)x << FQ)
              + (int64_t)c.b1 * st[0]
              + (int64_t)c.b2 * st[1]
              - (int64_t)c.a1 * st[2]
              - (int64_t)c.a2 * st[3];
  int32_t y = (int32_t)((acc + (1LL << (FQ - 1))) >> FQ);  // redondeo
  st[1] = st[0];  st[0] = x;
  st[3] = st[2];  st[2] = y;
  return y;
}

// Cadena completa: ecualizador INMP441 + ponderación A (todo en enteros).
// Entrada: muestra de 24 bits (raw >> 8). Salida: misma escala de 24 bits,
// SIN la ganancia escalar (se aplica por bloque).
static inline int32_t aplicarFiltrosInt(int32_t muestra24) {
  int32_t s = biquad_i(muestra24, filtro_st[0], INMP441_sos);
  s = biquad_i(s, filtro_st[1], Aw_sos[0]);
  s = biquad_i(s, filtro_st[2], Aw_sos[1]);
  s = biquad_i(s, filtro_st[3], Aw_sos[2]);
  return s;
}

// ── VARIABLES GLOBALES ────────────────────────────────

struct MedicionBloque {
  double sum_sqr;
  float  peak;
};

QueueHandle_t samples_queue;
float         samples_buf[SAMPLES_SHORT] __attribute__((aligned(4)));

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastPublish   = 0;
unsigned long lastMqttRetry = 0;

double   Leq_sum_sqr = 0;
uint32_t Leq_samples = 0;
double   Leq_dB      = 0;
double   peak_dB     = 0;
double   min_dB      = 999;

// ── LEQ PROMEDIO 1 MINUTO (para calibración) ──────────
// Buffer circular con la energía de los últimos 60 segundos.
// Se promedia energéticamente, no aritméticamente (correcto en acústica).
#define LEQ1M_SEGS 60
double  leq1m_energia[LEQ1M_SEGS] = {0};  // energía media de cada segundo
int     leq1m_idx    = 0;                  // posición actual del buffer
int     leq1m_count  = 0;                  // segundos acumulados (hasta 60)
double  Leq_1min_dB  = 0;                  // resultado en dB(A)

// Añade la energía de un segundo al buffer y recalcula el Leq de 1 min
void acumularLeq1min(double energiaMediaSegundo) {
  leq1m_energia[leq1m_idx] = energiaMediaSegundo;
  leq1m_idx = (leq1m_idx + 1) % LEQ1M_SEGS;
  if (leq1m_count < LEQ1M_SEGS) leq1m_count++;

  double suma = 0;
  for (int i = 0; i < leq1m_count; i++) suma += leq1m_energia[i];
  double media = suma / leq1m_count;

  Leq_1min_dB = MIC_OFFSET_DB + MIC_REF_DB +
                20.0 * log10(sqrt(media) / MIC_REF_AMPL + 1e-10);
  Leq_1min_dB = constrain(Leq_1min_dB, MIC_NOISE_DB, MIC_OVERLOAD_DB);
}

// ── SERVIDOR WEB DE CALIBRACIÓN ───────────────────────
WebServer server(80);
Preferences prefs;

// Cargar el offset guardado en flash (o el de fábrica si no hay ninguno)
void cargarOffset() {
  prefs.begin("noisesensor", true);   // solo lectura
  MIC_OFFSET_DB = prefs.getDouble("offset", MIC_OFFSET_DEFAULT);
  prefs.end();
  Serial.printf("[CAL] Offset cargado: %.2f dB\n", MIC_OFFSET_DB);
}

// Guardar un nuevo offset en flash
void guardarOffset(double nuevo) {
  MIC_OFFSET_DB = nuevo;
  prefs.begin("noisesensor", false);
  prefs.putDouble("offset", nuevo);
  prefs.end();
  Serial.printf("[CAL] Offset guardado: %.2f dB\n", nuevo);
}

// ── LED DE ESTADO (azul integrado, GPIO8) ─────────────
// En el ESP32-C3 SuperMini el LED azul y el WS2812 comparten GPIO8.
// Usamos solo el LED simple (digitalWrite) para evitar conflictos.
#define LED_STATUS 8

enum EstadoLed { LED_CONFIG_WIFI, LED_CONECTANDO, LED_OK, LED_ERROR };
EstadoLed estadoActual = LED_CONFIG_WIFI;
unsigned long lastBlink = 0;
bool ledOn = false;

void actualizarLed() {
  unsigned long ahora = millis();

  switch (estadoActual) {
    case LED_CONFIG_WIFI:
      // Parpadeo rápido (250ms) — esperando configuración WiFi
      if (ahora - lastBlink >= 250) {
        lastBlink = ahora;
        ledOn = !ledOn;
        digitalWrite(LED_STATUS, ledOn);
      }
      break;

    case LED_CONECTANDO:
      // Parpadeo lento (600ms) — conectando WiFi/MQTT
      if (ahora - lastBlink >= 600) {
        lastBlink = ahora;
        ledOn = !ledOn;
        digitalWrite(LED_STATUS, ledOn);
      }
      break;

    case LED_OK:
      // Fijo encendido — todo funcionando
      digitalWrite(LED_STATUS, HIGH);
      break;

    case LED_ERROR:
      // Parpadeo muy rápido (100ms) — error persistente
      if (ahora - lastBlink >= 100) {
        lastBlink = ahora;
        ledOn = !ledOn;
        digitalWrite(LED_STATUS, ledOn);
      }
      break;
  }
}


// ── WIFIMANAGER ───────────────────────────────────────

// Detecta doble arranque rápido para forzar el portal de configuración.
// El vecino solo tiene que desenchufar y enchufar el sensor dos veces
// seguidas (menos de 5 segundos entre arranques) para reconfigurar el WiFi.

bool detectarDobleArranque() {
  prefs.begin("noisesensor", false);
  int contador = prefs.getInt("boot", 0);
  contador++;

  // Si es el segundo arranque dentro de la ventana, forzar portal
  if (contador >= 2) {
    prefs.putInt("boot", 0);   // resetear para el próximo arranque
    prefs.end();
    Serial.println("[BOOT] Doble arranque detectado");
    return true;
  }

  // Primer arranque: marcamos el contador y esperamos una ventana corta.
  // Si el sensor sigue encendido tras la ventana, lo consideramos arranque
  // normal y reseteamos el contador AQUÍ MISMO (no en el loop), para que
  // una conexión WiFi lenta no deje el contador sin limpiar.
  prefs.putInt("boot", contador);
  prefs.end();

  Serial.println("[BOOT] Esperando ventana de doble arranque (3s)...");
  delay(3000);   // ventana: si se apaga antes de 3s y reinicia, cuenta doble

  prefs.begin("noisesensor", false);
  prefs.putInt("boot", 0);
  prefs.end();
  Serial.println("[BOOT] Ventana superada, arranque normal");

  return false;
}

// HTML/CSS personalizado inyectado en la cabecera del portal WiFiManager
String portalHeader() {
  String h = "<style>";
  h += "body{background:#F8FAFC;}";
  h += ".wrap{text-align:center;}";
  h += "h1,h3{color:#00467C !important;}";
  h += "button{background:#2563EB !important;border:0 !important;}";
  h += "input{border:1px solid #2563EB !important;}";
  h += ".sensor-title{color:#F97316;font-weight:bold;font-size:18px;margin:15px 0;}";
  h += "</style>";
  h += "<div class='sensor-title'>Sensor de Ruido</div>";
  return h;
}

void configurarWiFi(bool forzarPortal) {
  pinMode(RESET_PIN, INPUT_PULLUP);
  WiFi.persistent(true);   // fuerza el guardado de credenciales WiFi en NVS

  // Forzar potencia de transmisión máxima. Ayuda con la antena cerámica
  // débil del ESP32-C3 SuperMini (problema conocido de estos módulos).
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(84);   // 84 = 21 dBm (máximo)

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  static String headerHtml = portalHeader();
  wm.setCustomHeadElement(headerHtml.c_str());

  String apName = String("NoiseSensor-") + NODE_ID;

  if (forzarPortal) {
    Serial.println("[WiFi] Doble arranque detectado -> abriendo portal");
    estadoActual = LED_CONFIG_WIFI;
    wm.resetSettings();

    // Modo no bloqueante para poder parpadear el LED mientras el portal
    // está abierto esperando que el vecino configure el WiFi.
    wm.setConfigPortalBlocking(false);
    wm.startConfigPortal(apName.c_str());

    unsigned long inicioPortal = millis();
    while (WiFi.status() != WL_CONNECTED) {
      wm.process();          // atiende el portal
      actualizarLed();       // parpadeo rápido (LED_CONFIG_WIFI)
      if (millis() - inicioPortal > 180000) {  // timeout 180s
        Serial.println("[WiFi] Timeout portal, reiniciando...");
        delay(3000);
        ESP.restart();
      }
      delay(10);
    }
  } else {
    // Intento de conexión al WiFi guardado con parpadeo lento.
    // El ESP32 guarda las credenciales de forma persistente; WiFi.begin()
    // sin argumentos las reutiliza. Comprobamos si hay SSID guardado
    // leyéndolo de la configuración persistente (no de la conexión actual).
    wifi_config_t conf;
    esp_wifi_get_config(WIFI_IF_STA, &conf);
    bool hayCredenciales = (strlen((char*)conf.sta.ssid) > 0);

    if (hayCredenciales) {
      Serial.printf("[WiFi] Credenciales guardadas: %s\n", (char*)conf.sta.ssid);
      Serial.println("[WiFi] Intentando conectar (20s)...");
      estadoActual = LED_CONECTANDO;
      WiFi.mode(WIFI_STA);
      WiFi.begin();
      unsigned long inicioConn = millis();
      while (WiFi.status() != WL_CONNECTED &&
             millis() - inicioConn < 20000) {
        actualizarLed();     // parpadeo lento
        delay(10);
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] Conexion al WiFi guardado OK");
      } else {
        Serial.println("[WiFi] FALLO al conectar al WiFi guardado (red no disponible o senal debil) -> abriendo portal");
      }
    } else {
      Serial.println("[WiFi] No hay credenciales guardadas");
    }

    // Si no conectó (sin credenciales o fallo), abrir el portal
    if (WiFi.status() != WL_CONNECTED) {
      estadoActual = LED_CONFIG_WIFI;
      wm.setConfigPortalBlocking(false);
      wm.startConfigPortal(apName.c_str());
      unsigned long inicioPortal = millis();
      while (WiFi.status() != WL_CONNECTED) {
        wm.process();
        actualizarLed();     // parpadeo rápido
        if (millis() - inicioPortal > 180000) {
          Serial.println("[WiFi] Timeout, reiniciando...");
          delay(3000);
          ESP.restart();
        }
        delay(10);
      }
    }
  }

  Serial.printf("[WiFi] Conectado · IP: %s\n", WiFi.localIP().toString().c_str());

  // Confirmar que las credenciales han quedado guardadas en la flash (NVS)
  wifi_config_t confGuardada;
  esp_wifi_get_config(WIFI_IF_STA, &confGuardada);
  if (strlen((char*)confGuardada.sta.ssid) > 0) {
    Serial.printf("[WiFi] Credenciales GUARDADAS correctamente: %s\n",
                  (char*)confGuardada.sta.ssid);
  } else {
    Serial.println("[WiFi] AVISO: las credenciales NO se guardaron en la flash");
  }
}

// ── MQTT ─────────────────────────────────────────────

void conectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  String clientId = String("noisesensor-") + NODE_ID + "-" +
                    String(random(0xffff), HEX);

  Serial.printf("[MQTT] Conectando como %s... ", clientId.c_str());

  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println("OK");
    String topicStatus = String("noisesensor/") + NODE_ID + "/status";
    String msg = String("{\"nodo\":\"") + NODE_ID + "\",\"estado\":\"online\"}";
    mqttClient.publish(topicStatus.c_str(), msg.c_str(), true);
  } else {
    Serial.printf("Error rc=%d\n", mqttClient.state());
  }
}

void publicarMedicion(double leq, double peak, double minDb) {
  // NOTA: "peak" es LAFmax (máximo con ponderación temporal Fast,
  // τ=125ms, IEC 61672-1) durante el intervalo de publicación. No es la
  // muestra cruda de mayor amplitud, que es sensible a glitches del
  // sensor. El nombre del campo JSON se mantiene por compatibilidad con
  // dashboards existentes, aunque el significado ahora es el normativo.
  if (!mqttClient.connected()) return;

  StaticJsonDocument<200> doc;
  doc["nodo"]  = NODE_ID;
  doc["leq"]   = round(leq   * 10) / 10.0;
  doc["peak"]  = round(peak  * 10) / 10.0;
  doc["min"]   = round(minDb * 10) / 10.0;
  doc["ts"]    = millis();

  char payload[200];
  serializeJson(doc, payload);

  String topic = String("noisesensor/") + NODE_ID + "/datos";
  mqttClient.publish(topic.c_str(), payload);

  Serial.printf("[→] Leq:%.1f Peak:%.1f Min:%.1f dB(A)\n", leq, peak, minDb);
}

// ── TAREA I2S ─────────────────────────────────────────

void mic_i2s_init() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
    I2S_NUM_0, I2S_ROLE_MASTER
  );
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                  I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK,
      .ws   = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };

  // Canal IZQUIERDO: confirmado experimentalmente como el correcto para
  // nuestro cableado L/R -> GND con el driver nuevo (con RIGHT no hay señal).
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  i2s_channel_init_std_mode(rx_handle, &std_cfg);
  i2s_channel_enable(rx_handle);
  Serial.println("[I2S] Inicializado (slot_mask = LEFT)");
}

void mic_reader_task(void* param) {
  mic_i2s_init();

  size_t bytes_read = 0;

  // Descartar primer bloque (INMP441 necesita ~83ms arranque)
  i2s_channel_read(rx_handle, samples_buf,
                   SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  // Resetear filtros tras el primer bloque corrupto
  resetFiltros();

  // Descartar dos bloques más para estabilizar
  i2s_channel_read(rx_handle, samples_buf,
                   SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  i2s_channel_read(rx_handle, samples_buf,
                   SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    i2s_channel_read(rx_handle, samples_buf,
                     SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

    // ── INSTRUMENTACIÓN: detectar pérdida de muestras / saturación CPU ──
    // Un bloque de 6000 muestras a 48kHz debe llegar cada 125.0 ms exactos.
    // Si el ciclo real es mayor, el DMA se está desbordando y PERDEMOS audio.
#if DEBUG_TIMING
    static unsigned long t_ciclo_prev = 0;
    static uint32_t nbloques = 0;
    static uint32_t ciclo_acum_us = 0, filtro_acum_us = 0;
    static uint32_t bytes_cortos = 0;

    unsigned long t0 = micros();
    if (t_ciclo_prev != 0) ciclo_acum_us += (uint32_t)(t0 - t_ciclo_prev);
    t_ciclo_prev = t0;

    if (bytes_read != SAMPLES_SHORT * sizeof(int32_t)) bytes_cortos++;
#endif

    int32_t* raw = (int32_t*)samples_buf;

    // Procesado 100% en enteros (el C3 no tiene FPU): acumulador de
    // energía en int64 y envolvente Fast (para LAFmax) en int64.
    // Ni una operación float por muestra.
    int64_t sum_sqr_i = 0;
    int64_t env_max_i = 0;   // máximo de la envolvente Fast en este bloque

    for (int i = 0; i < SAMPLES_SHORT; i++) {
      // Descartar los 8 bits bajos (relleno) -> muestra de 24 bits útiles
      int32_t muestra24 = raw[i] >> 8;
      int32_t a = aplicarFiltrosInt(muestra24);
      int64_t a_sq = (int64_t)a * a;
      sum_sqr_i += a_sq;

      // Envolvente Fast (IEC 61672-1): filtro paso-bajo exponencial de
      // τ=125ms sobre la potencia, continuo entre bloques. Un glitch de
      // 1 muestra apenas la mueve; un evento sostenido sí.
      env_fast_i += ((a_sq - env_fast_i) * ALPHA_FAST_Q28) >> FQ;
      if (env_fast_i > env_max_i) env_max_i = env_fast_i;
    }

    // Conversión a escala normalizada (-1..+1) y ganancia escalar de los
    // filtros, UNA sola vez por bloque (aquí sí usamos float/double).
    const double ESC = 8388608.0;                     // 2^23
    double sum  = ((double)sum_sqr_i / (ESC * ESC)) * FILTER_G_TOTAL2;

    // env_max_i está en escala de POTENCIA (igual que sum_sqr_i): se
    // normaliza igual, con la ganancia al cuadrado (FILTER_G_TOTAL2), NO
    // con la ganancia lineal -- así el pico usa la misma "vara de medir"
    // que el Leq, y la conversión a dB más abajo usa 10*log10 (potencia)
    // en vez de 20*log10 (amplitud).
    float  pico = (float)(((double)env_max_i / (ESC * ESC)) * FILTER_G_TOTAL2);

#if DEBUG_TIMING
    filtro_acum_us += (uint32_t)(micros() - t0);
    nbloques++;

    // Resumen cada 8 bloques (~1 s si vamos a tiempo real)
    if (nbloques >= 8) {
      float ciclo_ms  = (ciclo_acum_us / 1000.0f) / 7.0f;  // 7 intervalos entre 8 bloques
      float filtro_ms = (filtro_acum_us / 1000.0f) / 8.0f;
      Serial.printf("[TIMING] ciclo=%.1f ms (ideal 125.0) | filtrado=%.1f ms | lecturas cortas=%lu\n",
                    ciclo_ms, filtro_ms, bytes_cortos);
      nbloques = 0; ciclo_acum_us = 0; filtro_acum_us = 0;
      t_ciclo_prev = 0;
    }
#endif
    // ── FIN INSTRUMENTACIÓN ──

    MedicionBloque bloque = { sum, pico };
    xQueueSend(samples_queue, &bloque, portMAX_DELAY);
  }
}

// ── PÁGINA WEB DE CALIBRACIÓN ─────────────────────────

// Página HTML principal de calibración
void handleRoot() {
  if (!server.authenticate(CAL_USER, CAL_PASS)) {
    return server.requestAuthentication();
  }

  String html = "<!DOCTYPE html><html lang='es'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Calibracion Sensor " + String(NODE_ID) + "</title><style>";
  html += "body{font-family:sans-serif;background:#F8FAFC;color:#00467C;text-align:center;padding:20px;}";
  html += "h2{color:#F97316;}";
  html += ".leq{font-size:56px;font-weight:bold;color:#2563EB;margin:8px;}";
  html += ".leqbig{font-size:72px;font-weight:bold;color:#00467C;margin:8px;}";
  html += ".box{background:#FFFFFF;border:1px solid #D6E4F5;border-radius:12px;padding:20px;max-width:400px;margin:15px auto;box-shadow:0 2px 6px rgba(0,70,124,0.08);}";
  html += "input{font-size:20px;padding:8px;width:120px;text-align:center;border-radius:6px;border:1px solid #2563EB;}";
  html += "button{font-size:18px;padding:10px 20px;background:#2563EB;color:#fff;border:0;border-radius:8px;margin-top:12px;cursor:pointer;}";
  html += ".small{color:#7a93c4;font-size:14px;}";
  html += ".aviso{color:#F97316;font-size:13px;margin-top:8px;}";
  html += "</style></head><body>";
  html += "<h2>Sensor de Ruido</h2>";
  html += "<div class='small'>Nodo: " + String(NODE_ID) + "</div>";

  // Promedio 1 minuto (el importante para calibrar)
  html += "<div class='box'><div class='small'>Promedio ultimo minuto</div>";
  html += "<div class='leqbig' id='leq1m'>--</div><div class='small'>dB(A) &middot; <span id='segs'>0</span>s acumulados</div></div>";

  // Instantáneo (informativo)
  html += "<div class='box'><div class='small'>Instantaneo (1s)</div>";
  html += "<div class='leq' id='leq'>--</div><div class='small'>dB(A)</div></div>";

  // Calibración
  html += "<div class='box'><div class='small'>Offset actual: <b id='off'>--</b> dB</div>";
  html += "<p>Valor <b>promedio 1 min</b> de tu app de referencia:</p>";
  html += "<input type='number' id='ref' step='0.1' placeholder='48.0'> dB(A)<br>";
  html += "<button onclick='calibrar()'>Aplicar calibracion</button>";
  html += "<div class='aviso'>Manten el movil junto al sensor durante 1 minuto con ruido estable antes de calibrar.</div>";
  html += "<div class='small' id='msg' style='margin-top:10px;'></div></div>";

  html += "<script>";
  html += "function upd(){fetch('/leq').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('leq').innerText=d.leq.toFixed(1);";
  html += "document.getElementById('leq1m').innerText=d.leq1m.toFixed(1);";
  html += "document.getElementById('segs').innerText=d.segs;";
  html += "document.getElementById('off').innerText=d.offset.toFixed(2);});}";
  html += "setInterval(upd,1000);upd();";
  html += "function calibrar(){var ref=document.getElementById('ref').value;";
  html += "if(!ref){alert('Introduce un valor');return;}";
  html += "fetch('/cal?ref='+ref).then(r=>r.json()).then(d=>{";
  html += "document.getElementById('msg').innerHTML='Nuevo offset: <b>'+d.offset.toFixed(2)+' dB</b>';";
  html += "document.getElementById('ref').value='';});}";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

// Devuelve el Leq actual, el promedio 1min y el offset como JSON
void handleLeq() {
  if (!server.authenticate(CAL_USER, CAL_PASS)) {
    return server.requestAuthentication();
  }
  String json = "{\"leq\":" + String(Leq_dB, 1) +
                ",\"leq1m\":" + String(Leq_1min_dB, 1) +
                ",\"segs\":" + String(leq1m_count) +
                ",\"offset\":" + String(MIC_OFFSET_DB, 2) + "}";
  server.send(200, "application/json", json);
}

// Aplica la calibración usando el PROMEDIO DE 1 MINUTO:
// nuevo_offset = offset_actual + (referencia_1min - leq_1min_medido)
void handleCal() {
  if (!server.authenticate(CAL_USER, CAL_PASS)) {
    return server.requestAuthentication();
  }
  if (!server.hasArg("ref")) {
    server.send(400, "application/json", "{\"error\":\"falta ref\"}");
    return;
  }
  double ref = server.arg("ref").toDouble();
  double nuevo = MIC_OFFSET_DB + (ref - Leq_1min_dB);
  guardarOffset(nuevo);

  String json = "{\"offset\":" + String(nuevo, 2) + "}";
  server.send(200, "application/json", json);
}

void iniciarServidorWeb() {
  server.on("/", handleRoot);
  server.on("/leq", handleLeq);
  server.on("/cal", handleCal);
  server.begin();

  // mDNS: permite acceder al sensor por nombre en vez de por IP.
  // El nombre sera "noisesensor-nodo-1.local" (según NODE_ID).
  String hostname = String("noisesensor-") + NODE_ID;
  if (MDNS.begin(hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[CAL] Servidor web en http://%s.local  (IP: %s)\n",
                  hostname.c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("[CAL] Servidor web en http://%s  (mDNS no disponible)\n",
                  WiFi.localIP().toString().c_str());
  }
}

// ── SETUP ─────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ─────────────────────────────────────────────────────
  // BORRADO DE NVS (memoria flash de configuración)
  // Descomentar SOLO al preparar una placa que ya se usó en pruebas
  // anteriores (borra offset, contador de arranque y credenciales WiFi
  // viejas). Subir una vez con esto activo, confirmar en el log que el
  // offset carga -8.99, y luego VOLVER A COMENTAR y subir de nuevo.
  // No hace falta en placas nuevas de fábrica.
  //
  // nvs_flash_erase();
  // nvs_flash_init();
  // Serial.println("[NVS] Flash borrada completamente");
  // ─────────────────────────────────────────────────────

  pinMode(LED_STATUS, OUTPUT);
  estadoActual = LED_CONFIG_WIFI;

  Serial.println("\n╔══════════════════════════════╗");
  Serial.printf( "║  Sensor de Ruido · %-9s║\n", NODE_ID);
  Serial.println("╚══════════════════════════════╝");

  // Cargar el offset de calibración guardado en flash
  cargarOffset();

  // Detectar doble arranque rápido para forzar portal de configuración
  bool forzarPortal = detectarDobleArranque();

  configurarWiFi(forzarPortal);

  samples_queue = xQueueCreate(8, sizeof(MedicionBloque));

  // Resetear filtros al arrancar
  resetFiltros();

  // ESP32-C3 tiene solo 1 núcleo (núcleo 0)
  xTaskCreatePinnedToCore(
    mic_reader_task, "I2S Reader",
    4096, NULL, 4, NULL, 0
  );

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(512);
  conectarMQTT();

  estadoActual = mqttClient.connected() ? LED_OK : LED_CONECTANDO;

  // Iniciar el servidor web de calibración
  iniciarServidorWeb();

  Serial.println("[OK] Listo");
}

// ── LOOP ──────────────────────────────────────────────

void loop() {
  // Atender peticiones de la página web de calibración
  server.handleClient();

  // Reconexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    estadoActual = LED_ERROR;
    actualizarLed();
    Serial.println("[WiFi] Reconectando...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  // Reconexión MQTT
  if (!mqttClient.connected()) {
    estadoActual = LED_CONECTANDO;
    unsigned long ahora = millis();
    if (ahora - lastMqttRetry > 5000) {
      lastMqttRetry = ahora;
      conectarMQTT();
      if (mqttClient.connected()) estadoActual = LED_OK;
    }
  } else {
    estadoActual = LED_OK;
  }
  mqttClient.loop();
  actualizarLed();

  // Procesar bloques de medición
  MedicionBloque bloque;
  while (xQueueReceive(samples_queue, &bloque, 0)) {

    double rms_db = MIC_OFFSET_DB + MIC_REF_DB +
                    20.0 * log10(sqrt(bloque.sum_sqr / SAMPLES_SHORT) / MIC_REF_AMPL + 1e-10);

    // LAFmax del bloque: bloque.peak ya es la envolvente Fast en escala de
    // POTENCIA (como sum_sqr), por eso aquí es 10*log10 con REF^2, igual
    // que se haría para pasar de potencia a dB en cualquier sonómetro.
    double pico_db = MIC_OFFSET_DB + MIC_REF_DB +
                     10.0 * log10((double)bloque.peak / (MIC_REF_AMPL * MIC_REF_AMPL) + 1e-20);

    // Acumular solo si está en rango válido
    if (rms_db >= MIC_NOISE_DB && rms_db <= MIC_OVERLOAD_DB) {
      Leq_sum_sqr += bloque.sum_sqr;
      Leq_samples += SAMPLES_SHORT;
      if (pico_db > peak_dB) peak_dB = pico_db;
      if (rms_db  < min_dB)  min_dB  = rms_db;
    }

    // Calcular Leq cada segundo
    if (Leq_samples >= SAMPLES_LEQ) {
      double energiaMedia = Leq_sum_sqr / Leq_samples;
      double rms = sqrt(energiaMedia);
      Leq_dB  = MIC_OFFSET_DB + MIC_REF_DB + 20.0 * log10(rms / MIC_REF_AMPL + 1e-10);
      Leq_dB  = constrain(Leq_dB,  MIC_NOISE_DB, MIC_OVERLOAD_DB);
      peak_dB = constrain(peak_dB, MIC_NOISE_DB, MIC_OVERLOAD_DB);
      min_dB  = constrain(min_dB,  MIC_NOISE_DB, MIC_OVERLOAD_DB);

      // Alimentar el promedio móvil de 1 minuto (para calibración)
      acumularLeq1min(energiaMedia);

      Leq_sum_sqr = 0;
      Leq_samples = 0;
    }
  }

  // Publicar cada PUBLISH_MS
  unsigned long ahora = millis();
  if (ahora - lastPublish >= PUBLISH_MS && Leq_dB > 0) {
    lastPublish = ahora;
    publicarMedicion(Leq_dB, peak_dB, min_dB);
    peak_dB = 0;
    min_dB  = 999;
  }
}
//Lmax: envolvente Fast (IEC 61672-1) en vez de muestra cruda + flag DEBUG_TIMING
