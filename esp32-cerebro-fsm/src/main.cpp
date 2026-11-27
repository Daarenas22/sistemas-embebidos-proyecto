#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

// --- CONFIGURACIÓN WI-FI Y SERVIDOR FASTAPI ---
const char* WIFI_SSID            = "PC";
const char* WIFI_PASS            = "12345678";
const char* SERVER_HOST          = "192.168.137.1";
const uint16_t SERVER_PORT       = 8000;
const char* SERVER_PATH          = "/auditar?solo_resumen=1";
const char* SERVER_SALUDO_WAV    = "/saludo-wav";     // Audio WAV PCM 16kHz del saludo IA
const char* SERVER_VEREDICTO_WAV = "/veredicto-wav";  // Audio WAV PCM 16kHz del resumen del veredicto
const char* SERVER_CAM_TRIGGER   = "/disparar-camara"; // Disparo coordinado por Wi-Fi para S3-CAM

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- PINES DE CONEXIÓN ---
#define BOTON_PIN 18       // Botón físico de FSM (despierta desde Standby) - Lado Derecho (D18)
#define TRIGGER_CAM_PIN 17 // Salida hacia el GPIO de disparo de la ESP32-S3-CAM - Lado Derecho (TX2 / GPIO17)
#define MIC_ANALOG_PIN 36  // Entrada analógica / AC para FFT (A0 / GPIO36 / VP) - Lado Izquierdo

// --- PINES I2C PARA PANTALLA OLED SSD1306 (LADO IZQUIERDO) ---
#define OLED_SDA 25        // Pin marcado D25 en la placa
#define OLED_SCL 26        // Pin marcado D26 en la placa

// --- PINES I2S PARA MICRÓFONO INMP441 (I2S_NUM_0 — LADO DERECHO) ---
#define I2S_WS   15   // Word Select (LRCK) - Pin marcado D15
#define I2S_SCK   4   // Serial Clock (BCLK) - Pin marcado D4
#define I2S_SD   19   // Serial Data (DOUT del mic) - Pin marcado D19
#define I2S_PORT I2S_NUM_0

// --- PINES I2S PARA AMPLIFICADOR MAX98357A (I2S_NUM_1 — SPEAKER) ---
// Pines del módulo MAX98357A (de izq. a der. en la placa morada):
//   LRC  -> GPIO 14 (D14) | BCLK -> GPIO 27 (D27) | DIN -> GPIO 13 (D13)
//   GAIN -> Sin conectar (9 dB) | SD -> Sin conectar (mezcla L+R activa)
//   GND  -> GND | Vin -> 3.3V o VIN (5V) | Terminal +/- -> Bocina 4-8 Ohm
#define SPK_BCLK       27        // BCLK del MAX98357A - Pin marcado D27 (Lado Izquierdo)
#define SPK_LRC        14        // LRC (Word Select) del MAX98357A - Pin marcado D14 (Lado Derecho)
#define SPK_DIN        13        // DIN (Data In) del MAX98357A - Pin marcado D13 (Lado Derecho)
#define SPK_PORT       I2S_NUM_1
#define SPK_SAMPLE_RATE 16000    // 16 kHz para voz clara y natural

// Umbral de sonido calibrado para detección de audio
#define SOUND_THRESHOLD 4200000
#define TIEMPO_ESCUCHA_MS 4000 // 4 segundos para hacer la pregunta
unsigned long inicioCapturaVoz = 0;

// ====================================================================
// --- MÓDULO DSP Y CONFIGURACIÓN FFT ---
// ====================================================================
#define MODO_SIMULACION 1      // 1: Onda sintética (60Hz + 180Hz + 300Hz), 0: Muestreo ADC por Hardware Timer
#define SAMPLES 256            // 256 muestras según especificación
#define SAMPLING_FREQ 2048.0   // fs = 2048 Hz (Resolución bin = 2048/256 = 8 Hz/bin)

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

double frecuenciaDominante = 0.0;
bool esIncandescente = false;

// Variables globales para resultados de DSP
volatile double magFundamental = 0.0; // Componente fundamental (60 Hz)
volatile double magArmonico3   = 0.0; // 3.er armónico (180 Hz)
volatile double magArmonico5   = 0.0; // 5.º armónico (300 Hz)
volatile double magArmonico7   = 0.0; // 7.º armónico (420 Hz)
volatile double porcentajeTHD  = 0.0; // % THD (Total Harmonic Distortion)

// --- TIMER POR HARDWARE DEL ESP32 (fs = 2048 Hz) ---
hw_timer_t *timerDSP = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t bufferADC[SAMPLES];
volatile int indiceMuestra = 0;
volatile bool muestreoEnProgreso = false;
SemaphoreHandle_t semMuestreoListo = NULL;

void IRAM_ATTR onTimerDSP() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (muestreoEnProgreso && indiceMuestra < SAMPLES) {
    bufferADC[indiceMuestra++] = analogRead(MIC_ANALOG_PIN);
    if (indiceMuestra >= SAMPLES) {
      muestreoEnProgreso = false;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(semMuestreoListo, &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void configurarTimerHardware() {
  // Periodo = 1.000.000 us / 2048 Hz ≈ 488 us
  #if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    timerDSP = timerBegin(1000000); // 1 MHz -> 1 tick = 1 us
    timerAttachInterrupt(timerDSP, &onTimerDSP);
    timerAlarm(timerDSP, 488, true, 0);
  #else
    timerDSP = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz -> 1 tick = 1 us)
    timerAttachInterrupt(timerDSP, &onTimerDSP, true);
    timerAlarmWrite(timerDSP, 488, true);
    timerAlarmEnable(timerDSP);
  #endif
}

void iniciarMuestreoTimer() {
  portENTER_CRITICAL(&timerMux);
  indiceMuestra = 0;
  muestreoEnProgreso = true;
  portEXIT_CRITICAL(&timerMux);
}

// --- VARIABLES PARA VISUALIZACIÓN Y GRABACIÓN DEL MICRÓFONO ---
#define MIC_SAMPLES 256
#define AUDIO_SAMPLE_RATE 8000
#define AUDIO_MAX_SECONDS 4
#define AUDIO_MAX_SAMPLES (AUDIO_SAMPLE_RATE * AUDIO_MAX_SECONDS)

int32_t micBuffer[MIC_SAMPLES];
volatile int32_t amplitudPromedio = 0;
int16_t* audioPCM = NULL;
volatile size_t audioSampleIndex = 0;
volatile bool grabandoAudio = false;
TaskHandle_t taskAudioHandle = NULL;

// Variables para respuesta del servidor
bool ultimoHttpOk = false;
int ultimoHttpCodigo = 0;
String resumenOledServidor = "";

// ====================================================================
// --- MÓDULO DE AUDIO: MAX98357A I2S SPEAKER (STREAMING HTTP WAV) ---
// ====================================================================
volatile bool reproduciendo = false;
bool saludoReproducido = false;        // Para que el saludo suene al entrar en STANDBY
unsigned long ultimoIntentoSaludoMs = 0;
uint8_t intentosSaludo = 0;

#define BAR_HISTORY 32
int barHistory[BAR_HISTORY];
int barIndex = 0;

// --- ESTADOS DE LA MÁQUINA DE ESTADOS (FSM) ---
enum EstadoFSM {
  STANDBY,       // Reposo: solo el botón físico despierta al sistema
  CAPTURA_VOZ,   // Grabación / escucha del micrófono I2S
  MUESTREO,      // Disparo digital S3-CAM y tarea DSP en Core 1
  ENVIO_HTTP,    // Simulación de envío HTTP y recepción de respuesta
  RESULTADO      // Veredicto en OLED durante 4s y retorno automático a Standby
};

EstadoFSM estadoActual = STANDBY;

// --- BANDERAS DE SINCRONIZACIÓN Y SEMÁFOROS FREERTOS ---
volatile bool audioListo = false;
volatile bool muestreoListo = false;
volatile bool httpListo = false;

SemaphoreHandle_t semFSM = NULL;
SemaphoreHandle_t semDispararDSP = NULL;
SemaphoreHandle_t semDSPFinalizado = NULL;
TaskHandle_t taskDSPHandle = NULL;

// Variables para debounce del botón
bool estadoBotonAnterior = HIGH;
unsigned long ultimoTiempoRebote = 0;
const unsigned long retardoRebote = 50;

float faseOnda = 0.0;

// Bitmap 'spidy', 40x40px
const unsigned char epd_bitmap_spidy [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 
  0x07, 0xff, 0xe0, 0x00, 0x00, 0x1e, 0x08, 0x78, 0x00, 0x00, 0x38, 0x08, 0x3c, 0x00, 0x00, 0xe2, 
  0x1c, 0x07, 0x00, 0x01, 0xc3, 0xcb, 0xc3, 0x80, 0x03, 0xa7, 0x08, 0x65, 0xc0, 0x03, 0x69, 0x08, 
  0x17, 0xc0, 0x06, 0x70, 0x08, 0x87, 0x60, 0x0c, 0xf0, 0xef, 0x87, 0x30, 0x0d, 0xfb, 0x89, 0x4f, 
  0xb0, 0x1f, 0xfc, 0x49, 0x3f, 0xb8, 0x19, 0x8e, 0x7e, 0x39, 0xd8, 0x13, 0x87, 0xcb, 0x71, 0xc8, 
  0x13, 0x83, 0x28, 0xe0, 0xc8, 0x33, 0x01, 0xad, 0xc0, 0xcc, 0x33, 0x00, 0xe7, 0x80, 0xcc, 0x37, 
  0x00, 0xc3, 0x00, 0xec, 0x33, 0x00, 0x66, 0x00, 0xcc, 0x33, 0x00, 0x26, 0x00, 0xcc, 0x33, 0x00, 
  0x7e, 0x00, 0xcc, 0x13, 0x80, 0x76, 0x00, 0xc8, 0x1d, 0x80, 0x67, 0x01, 0xd8, 0x19, 0xc0, 0xff, 
  0x01, 0x98, 0x18, 0xe1, 0xe7, 0x83, 0x18, 0x0c, 0x7f, 0xa5, 0xff, 0x30, 0x0c, 0xff, 0x7e, 0xff, 
  0x30, 0x07, 0x0a, 0x00, 0x30, 0xe0, 0x03, 0x04, 0x00, 0x30, 0xc0, 0x03, 0x87, 0x42, 0xe0, 0xc0, 
  0x01, 0xc8, 0x7e, 0x13, 0x80, 0x00, 0xf0, 0x42, 0x0f, 0x00, 0x00, 0x38, 0x42, 0x1e, 0x00, 0x00, 
  0x1e, 0x00, 0x78, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00
};

// ====================================================================
// --- INICIALIZACIÓN DE HARDWARE I2S (MICRÓFONO + SPEAKER) ---
// ====================================================================

void initI2SMic() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 32000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void initI2SSpeaker() {
  const i2s_config_t spk_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SPK_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,   // 6*256 = 1536 frames (~96 ms a 16 kHz) para drenado limpio
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t spk_pins = {
    .bck_io_num = SPK_BCLK,
    .ws_io_num = SPK_LRC,
    .data_out_num = SPK_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(SPK_PORT, &spk_config, 0, NULL);
  i2s_set_pin(SPK_PORT, &spk_pins);
  i2s_zero_dma_buffer(SPK_PORT);
  Serial.println("[SETUP] Amplificador MAX98357A (I2S_NUM_1 @ 16kHz) inicializado.");
}

// ====================================================================
// --- REPRODUCCIÓN DE AUDIO WAV EN STREAMING POR I2S (MAX98357A) ---
// ====================================================================

// Prototipo de utilidad Wi-Fi definida más abajo
bool asegurarWiFiConectado(unsigned long timeoutMs);

static const uint8_t b64_lookup[128] = {
  64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
  64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
  64,64,64,64,64,64,64,64,64,64,64,62,64,64,64,63,
  52,53,54,55,56,57,58,59,60,61,64,64,64, 0,64,64,
  64, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
  15,16,17,18,19,20,21,22,23,24,25,64,64,64,64,64,
  64,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
  41,42,43,44,45,46,47,48,49,50,51,64,64,64,64,64
};

// Descarga un archivo WAV PCM 16-bit directamente desde el servidor y lo reproduce
// en streaming por el MAX98357A sin límite de duración ni uso de heap extra.
// No modifica la pantalla OLED para mantener visible el saludo o el veredicto.
bool reproducirWavDesdeEndpoint(const char* path) {
  Serial.printf("[SPEAKER] Solicitando streaming WAV desde %s...\n", path);

  if (!asegurarWiFiConectado(8000)) {
    Serial.println("[SPEAKER] Sin Wi-Fi para reproducir audio.");
    return false;
  }

  WiFiClient client;
  if (!client.connect(SERVER_HOST, SERVER_PORT)) {
    Serial.printf("[SPEAKER] No se pudo conectar a %s:%u\n", SERVER_HOST, SERVER_PORT);
    return false;
  }

  client.printf("GET %s HTTP/1.1\r\n", path);
  client.printf("Host: %s:%u\r\n", SERVER_HOST, SERVER_PORT);
  client.print("Connection: close\r\n\r\n");

  unsigned long waitStart = millis();
  while (client.connected() && !client.available() && (millis() - waitStart < 15000)) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  if (!client.available()) {
    Serial.println("[SPEAKER] Timeout esperando respuesta del servidor.");
    client.stop();
    return false;
  }

  // Leer línea de estado HTTP
  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  int sp1 = statusLine.indexOf(' ');
  int httpCode = (sp1 > 0) ? statusLine.substring(sp1 + 1).toInt() : 0;
  if (httpCode < 200 || httpCode >= 300) {
    Serial.printf("[SPEAKER] Error HTTP %d en %s\n", httpCode, path);
    client.stop();
    return false;
  }

  // Leer headers HTTP hasta línea vacía y capturar Content-Length
  long contentLength = -1;
  while (client.connected() || client.available()) {
    String hLine = client.readStringUntil('\n');
    if (hLine == "\r" || hLine.length() == 0) break;
    String lower = hLine;
    lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      contentLength = lower.substring(15).toInt();
    }
  }

  // Leer los 44 bytes de cabecera WAV
  uint8_t wavHdr[44];
  size_t hdrRead = 0;
  unsigned long lastRx = millis();
  while (hdrRead < 44 && (client.connected() || client.available()) && (millis() - lastRx < 5000)) {
    if (client.available()) {
      int n = client.read(wavHdr + hdrRead, 44 - hdrRead);
      if (n > 0) {
        hdrRead += n;
        lastRx = millis();
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  if (hdrRead < 44 || memcmp(wavHdr, "RIFF", 4) != 0 || memcmp(wavHdr + 8, "WAVE", 4) != 0) {
    Serial.println("[SPEAKER] Cabecera WAV inválida.");
    client.stop();
    return false;
  }

  uint16_t numChannels = (uint16_t)wavHdr[22] | ((uint16_t)wavHdr[23] << 8);
  uint32_t sampleRate  = (uint32_t)wavHdr[24] | ((uint32_t)wavHdr[25] << 8) |
                         ((uint32_t)wavHdr[26] << 16) | ((uint32_t)wavHdr[27] << 24);
  if (sampleRate < 8000 || sampleRate > 48000) {
    sampleRate = SPK_SAMPLE_RATE;
  }
  i2s_set_clk(SPK_PORT, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(SPK_PORT);

  Serial.printf("[SPEAKER] Streaming WAV: %u Hz, %u ch, Content-Length=%ld\n",
                (unsigned int)sampleRate, (unsigned int)numChannels, contentLength);

  reproduciendo = true;
  uint8_t netBuf[512];
  int16_t stereoChunk[512]; // hasta 256 frames estéreo (1024 bytes)
  uint8_t carryByte = 0;
  bool hasCarry = false;
  size_t totalPcmBytes = 0;
  long remainingBytes = (contentLength > 44) ? (contentLength - 44) : -1;
  lastRx = millis();

  while ((remainingBytes != 0) && (client.connected() || client.available()) && (millis() - lastRx < 5000)) {
    if (!client.available()) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    size_t toRead = sizeof(netBuf) - (hasCarry ? 1 : 0);
    if (remainingBytes > 0 && (long)toRead > remainingBytes) {
      toRead = (size_t)remainingBytes;
    }

    size_t startIdx = hasCarry ? 1 : 0;
    if (hasCarry) {
      netBuf[0] = carryByte;
      hasCarry = false;
    }

    int n = client.read(netBuf + startIdx, toRead);
    if (n <= 0) continue;

    lastRx = millis();
    if (remainingBytes > 0) remainingBytes -= n;

    size_t bufBytes = startIdx + (size_t)n;
    totalPcmBytes += (size_t)n;

    if (numChannels == 1) {
      // Asegurar múltiplo de 2 bytes (16-bit mono)
      if (bufBytes & 1) {
        carryByte = netBuf[bufBytes - 1];
        hasCarry = true;
        bufBytes--;
      }
      size_t monoSamples = bufBytes / 2;
      const int16_t* mono = (const int16_t*)netBuf;
      for (size_t s = 0; s < monoSamples; s++) {
        stereoChunk[s * 2]     = mono[s];
        stereoChunk[s * 2 + 1] = mono[s];
      }
      size_t bw = 0;
      if (monoSamples > 0) {
        i2s_write(SPK_PORT, stereoChunk, monoSamples * 4, &bw, portMAX_DELAY);
      }
    } else {
      // Estéreo 16-bit: asegurar múltiplo de 4 bytes
      size_t rem4 = bufBytes & 3;
      if (rem4 == 1) {
        carryByte = netBuf[bufBytes - 1];
        hasCarry = true;
        bufBytes--;
      } else {
        bufBytes -= rem4;
      }
      size_t bw = 0;
      if (bufBytes > 0) {
        i2s_write(SPK_PORT, netBuf, bufBytes, &bw, portMAX_DELAY);
      }
    }
  }

  client.stop();

  // Enviar silencio equivalente al tamaño del DMA (6 * 256 frames) y esperar a que termine de sonar
  memset(stereoChunk, 0, sizeof(stereoChunk));
  size_t bw = 0;
  for (int k = 0; k < 6; k++) {
    i2s_write(SPK_PORT, stereoChunk, sizeof(stereoChunk), &bw, portMAX_DELAY);
  }
  vTaskDelay(pdMS_TO_TICKS(150));
  i2s_zero_dma_buffer(SPK_PORT);

  reproduciendo = false;
  Serial.printf("[SPEAKER] Reproducción finalizada (%u bytes PCM).\n", (unsigned int)totalPcmBytes);
  return (totalPcmBytes > 0);
}

// Lee el flujo JSON HTTP sin almacenar el audio Base64 en RAM (stream-decode directo a wavOut)
void extraerResumenYAudioDeStream(WiFiClient& client, String* resumenOut,
                                  uint8_t* wavOut, size_t maxWavBytes, size_t* wavLenOut,
                                  unsigned long timeoutMs) {
  if (resumenOut != NULL) *resumenOut = "";
  if (wavLenOut != NULL) *wavLenOut = 0;

  enum ParseState {
    SCAN_KEY,
    IN_KEY,
    WAIT_COLON,
    WAIT_VAL_START,
    IN_SKIP_STR,
    IN_RESUMEN_STR,
    IN_AUDIO_B64
  };

  ParseState st = SCAN_KEY;
  String keyBuf = "";
  bool escaped = false;
  char b64Quartet[4];
  int b64Idx = 0;
  size_t wavPos = 0;
  unsigned long ultimoDato = millis();

  while ((client.connected() || client.available()) && (millis() - ultimoDato) < timeoutMs) {
    if (!client.available()) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    char c = (char)client.read();
    ultimoDato = millis();

    switch (st) {
      case SCAN_KEY:
        if (c == '"') {
          keyBuf = "";
          st = IN_KEY;
        }
        break;

      case IN_KEY:
        if (c == '"') {
          st = WAIT_COLON;
        } else if (keyBuf.length() < 32) {
          keyBuf += c;
        }
        break;

      case WAIT_COLON:
        if (c == ':') {
          st = WAIT_VAL_START;
        } else if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
          st = SCAN_KEY;
        }
        break;

      case WAIT_VAL_START:
        if (c == '"') {
          escaped = false;
          if (resumenOut != NULL && keyBuf == "resumen_oled") {
            st = IN_RESUMEN_STR;
          } else if (wavOut != NULL && keyBuf == "audio_base64") {
            b64Idx = 0;
            wavPos = 0;
            st = IN_AUDIO_B64;
          } else {
            st = IN_SKIP_STR;
          }
        } else if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
          // Valor numérico, booleano, null, objeto o array: volver a buscar claves
          st = SCAN_KEY;
        }
        break;

      case IN_SKIP_STR:
        if (escaped) {
          escaped = false;
        } else if (c == '\\') {
          escaped = true;
        } else if (c == '"') {
          st = SCAN_KEY;
        }
        break;

      case IN_RESUMEN_STR:
        if (escaped) {
          if (c == 'n') *resumenOut += '\n';
          else if (c == 'r') {}
          else if (c == 't') *resumenOut += ' ';
          else *resumenOut += c;
          escaped = false;
        } else if (c == '\\') {
          escaped = true;
        } else if (c == '"') {
          st = SCAN_KEY;
        } else if (resumenOut->length() < 200) {
          *resumenOut += c;
        }
        break;

      case IN_AUDIO_B64:
        if (c == '"') {
          st = SCAN_KEY;
        } else if ((uint8_t)c < 128 && (b64_lookup[(uint8_t)c] < 64 || c == '=')) {
          b64Quartet[b64Idx++] = c;
          if (b64Idx == 4) {
            b64Idx = 0;
            uint8_t pad = (b64Quartet[2] == '=' ? 1 : 0) + (b64Quartet[3] == '=' ? 1 : 0);
            uint32_t s0 = (b64Quartet[0] != '=') ? b64_lookup[(uint8_t)b64Quartet[0]] : 0;
            uint32_t s1 = (b64Quartet[1] != '=') ? b64_lookup[(uint8_t)b64Quartet[1]] : 0;
            uint32_t s2 = (b64Quartet[2] != '=') ? b64_lookup[(uint8_t)b64Quartet[2]] : 0;
            uint32_t s3 = (b64Quartet[3] != '=') ? b64_lookup[(uint8_t)b64Quartet[3]] : 0;
            uint32_t triple = (s0 << 18) | (s1 << 12) | (s2 << 6) | s3;

            if (wavPos < maxWavBytes) wavOut[wavPos++] = (triple >> 16) & 0xFF;
            if (pad < 2 && wavPos < maxWavBytes) wavOut[wavPos++] = (triple >> 8) & 0xFF;
            if (pad < 1 && wavPos < maxWavBytes) wavOut[wavPos++] = triple & 0xFF;
          }
        }
        break;
    }
  }

  if (wavLenOut != NULL) *wavLenOut = wavPos;
}

bool asegurarWiFiConectado(unsigned long timeoutMs);

bool solicitarSaludoServidor() {
  return reproducirWavDesdeEndpoint(SERVER_SALUDO_WAV);
}

// --- DISPARO DIGITAL Y REMOTO A LA CÁMARA (ESP32-S3-CAM) ---
volatile bool camaraDisparadaEnCiclo = false;

void notificarDisparoCamaraServidor() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WROOM] Sin Wi-Fi para disparo remoto de cámara (solo GPIO).");
    return;
  }

  WiFiClient client;
  if (!client.connect(SERVER_HOST, SERVER_PORT, 800)) {
    Serial.println("[WROOM] No se pudo conectar al servidor para /disparar-camara.");
    return;
  }

  client.printf("POST %s HTTP/1.1\r\n", SERVER_CAM_TRIGGER);
  client.printf("Host: %s:%u\r\n", SERVER_HOST, SERVER_PORT);
  client.print("Content-Length: 0\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long tWait = millis();
  while (client.connected() && !client.available() && (millis() - tWait < 400)) {
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (client.available()) {
    String status = client.readStringUntil('\n');
    status.trim();
    Serial.printf("[WROOM] Disparo remoto en servidor -> %s\n", status.c_str());
  }
  client.stop();
}

void enviarDisparoCamara() {
  Serial.printf("[WROOM] Activando pulso digital hacia ESP32-S3-CAM (GPIO %d)...\n", TRIGGER_CAM_PIN);
  unsigned long tInicio = millis();
  digitalWrite(TRIGGER_CAM_PIN, HIGH);
  notificarDisparoCamaraServidor();
  unsigned long transcurrido = millis() - tInicio;
  if (transcurrido < 500) {
    vTaskDelay(pdMS_TO_TICKS(500 - transcurrido));  // 500 ms: margen amplio para detección de flanco
  }
  digitalWrite(TRIGGER_CAM_PIN, LOW);
  camaraDisparadaEnCiclo = true;
  Serial.println("[WROOM] Pulso finalizado.");
}

// --- CÁLCULO DE MAGNITUD DE ARMÓNICOS ---
// Busca el pico máximo en el entorno espectral de la frecuencia buscada (bin = fs/N = 8 Hz)
double obtenerMagnitudArmonico(double freqObjetivo) {
  double binExacto = freqObjetivo / (SAMPLING_FREQ / (double)SAMPLES);
  int binCentral = round(binExacto);
  double maxMag = 0.0;

  for (int b = binCentral - 2; b <= binCentral + 2; b++) {
    if (b >= 0 && b < (SAMPLES / 2)) {
      if (vReal[b] > maxMag) {
        maxMag = vReal[b];
      }
    }
  }
  return maxMag;
}

// --- TAREA FREERTOS DSP FIJADA AL CORE 1 ---
void tareaDSPCore1(void *pvParameters) {
  for (;;) {
    // Esperar la señal de activación de la FSM en estado MUESTREO
    if (xSemaphoreTake(semDispararDSP, portMAX_DELAY) == pdTRUE) {
      Serial.println("\n[DSP Core 1] >> Tarea DSP activada en Core 1.");

      #if MODO_SIMULACION == 1
        Serial.println("[DSP Core 1] [MODO_SIMULACION=1] Generando onda sintetica: 60Hz + 180Hz (3ro) + 300Hz (5to)...");
        for (int i = 0; i < SAMPLES; i++) {
          double t = (double)i / SAMPLING_FREQ;
          // Señal sintética: 1000*sin(60Hz) + 200*sin(180Hz) + 100*sin(300Hz)
          vReal[i] = 1000.0 * sin(2.0 * M_PI * 60.0 * t) +
                      200.0 * sin(2.0 * M_PI * 180.0 * t) +
                      100.0 * sin(2.0 * M_PI * 300.0 * t);
          vImag[i] = 0.0;
        }
        vTaskDelay(pdMS_TO_TICKS(125)); // Tiempo equivalente a 256 muestras a 2048 Hz
      #else
        Serial.println("[DSP Core 1] [MODO_SIMULACION=0] Iniciando captura por Timer de Hardware a 2048 Hz...");
        iniciarMuestreoTimer();
        xSemaphoreTake(semMuestreoListo, portMAX_DELAY);

        // Remover componente de offset DC
        double suma = 0.0;
        for (int i = 0; i < SAMPLES; i++) suma += bufferADC[i];
        double dc = suma / (double)SAMPLES;

        for (int i = 0; i < SAMPLES; i++) {
          vReal[i] = (double)bufferADC[i] - dc;
          vImag[i] = 0.0;
        }
      #endif

      // Procesamiento FFT con ventana Hamming
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();

      frecuenciaDominante = FFT.majorPeak();

      // Cálculo de armónicos
      magFundamental = obtenerMagnitudArmonico(60.0);
      magArmonico3   = obtenerMagnitudArmonico(180.0);
      magArmonico5   = obtenerMagnitudArmonico(300.0);
      magArmonico7   = obtenerMagnitudArmonico(420.0);

      // Cálculo de Distorsión Armónica Total (%THD)
      if (magFundamental > 1.0) {
        double sumaCuadrados = (magArmonico3 * magArmonico3) +
                               (magArmonico5 * magArmonico5) +
                               (magArmonico7 * magArmonico7);
        porcentajeTHD = (sqrt(sumaCuadrados) / magFundamental) * 100.0;
      } else {
        porcentajeTHD = 0.0;
      }

      // Criterio de clasificación de carga
      if (frecuenciaDominante >= 55.0 && frecuenciaDominante <= 65.0) {
        esIncandescente = true;
      } else {
        esIncandescente = false;
      }

      Serial.println("==================================================");
      Serial.println("           REPORTE DSP (Core 1)");
      Serial.println("==================================================");
      Serial.printf(" Frecuencia Pico (Major Peak) : %.2f Hz\n", frecuenciaDominante);
      Serial.printf(" Magnitud Fundamental (60 Hz) : %.2f\n", magFundamental);
      Serial.printf(" Magnitud 3.er Armonico (180 Hz): %.2f\n", magArmonico3);
      Serial.printf(" Magnitud 5.to Armonico (300 Hz): %.2f\n", magArmonico5);
      Serial.printf(" Magnitud 7.mo Armonico (420 Hz): %.2f\n", magArmonico7);
      Serial.printf(" Distorsion Armonica Total (THD): %.2f %%\n", porcentajeTHD);
      Serial.println("==================================================");

      // Notificar a la FSM que el cálculo DSP finalizó
      xSemaphoreGive(semDSPFinalizado);
    }
  }
}

// --- BARRA DINÁMICA EN EL TERMINAL SERIE ---
void imprimirBarraTerminal(int32_t amp, int32_t maxRange) {
  const int anchoBarra = 24;
  int llena = map(constrain(amp, 0, maxRange), 0, maxRange, 0, anchoBarra);
  int porcentaje = (int)((constrain(amp, 0, maxRange) * 100LL) / maxRange);

  Serial.print("[ESCUCHANDO] [");
  for (int i = 0; i < anchoBarra; i++) {
    if (i < llena) {
      Serial.print("=");
    } else if (i == llena && llena > 0) {
      Serial.print(">");
    } else {
      Serial.print(" ");
    }
  }
  Serial.printf("] %3d%% (Amp: %d)", porcentaje, amp);

  if (amp >= SOUND_THRESHOLD) {
    Serial.println("  >>> ¡SONIDO DETECTADO!");
  } else {
    Serial.println();
  }
}

// --- TAREA FREERTOS DE CAPTURA CONTINUA DE AUDIO I2S ---
// Lee el flujo DMA continuamente durante CAPTURA_VOZ sin perder muestras mientras el OLED se actualiza
void tareaAudioI2S(void *pvParameters) {
  int32_t decimSum = 0;
  int decimCount = 0;
  int32_t dcFilter = 0;
  int lastChOffset = 0;
  bool estabaGrabando = false;

  for (;;) {
    if (!grabandoAudio) {
      estabaGrabando = false;
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }

    if (!estabaGrabando) {
      estabaGrabando = true;
      decimSum = 0;
      decimCount = 0;
      dcFilter = 0;
    }

    size_t bytes_read = 0;
    esp_err_t res = i2s_read(I2S_PORT, micBuffer, sizeof(micBuffer), &bytes_read, pdMS_TO_TICKS(50));

    if (res == ESP_OK && bytes_read > 0) {
      int samples_count = bytes_read / sizeof(int32_t);
      int32_t max_ch0 = INT32_MIN, min_ch0 = INT32_MAX;
      int32_t max_ch1 = INT32_MIN, min_ch1 = INT32_MAX;

      for (int i = 0; i < samples_count; i += 2) {
        int32_t val0 = micBuffer[i] >> 8;
        int32_t val1 = micBuffer[i + 1] >> 8;

        if (val0 > max_ch0) max_ch0 = val0;
        if (val0 < min_ch0) min_ch0 = val0;

        if (val1 > max_ch1) max_ch1 = val1;
        if (val1 < min_ch1) min_ch1 = val1;
      }

      int32_t amp_ch0 = max_ch0 - min_ch0;
      int32_t amp_ch1 = max_ch1 - min_ch1;

      amplitudPromedio = (amp_ch0 > amp_ch1) ? amp_ch0 : amp_ch1;

      if (amp_ch0 > 0 || amp_ch1 > 0) {
        lastChOffset = (amp_ch1 > amp_ch0) ? 1 : 0;
      }

      // Decimar 4:1 (32000 Hz -> 8000 Hz) en PCM 16-bit mono
      if (audioPCM != NULL) {
        for (int i = 0; i < samples_count; i += 2) {
          int32_t sample16 = micBuffer[i + lastChOffset] >> 15;
          decimSum += sample16;
          decimCount++;

          if (decimCount == 4) {
            int32_t avg = decimSum / 4;
            decimSum = 0;
            decimCount = 0;

            dcFilter += (avg - dcFilter) >> 8;
            int32_t clean = avg - dcFilter;

            if (clean > 32767) clean = 32767;
            else if (clean < -32768) clean = -32768;

            if (audioSampleIndex < AUDIO_MAX_SAMPLES) {
              audioPCM[audioSampleIndex++] = (int16_t)clean;
            }
          }
        }
      }
    }
  }
}

// --- PANTALLA: CAPTURA DE VOZ CON TEMPORIZADOR ---
void dibujarPantallaMicrofono(unsigned long tiempoRestanteMs) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Encabezado con indicador activo y cuenta regresiva
  display.setCursor(0, 0);
  display.print("ESCUCHANDO");
  
  float segRestantes = tiempoRestanteMs / 1000.0;
  display.setCursor(86, 0);
  display.printf("%.1fs", segRestantes);
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  int32_t ampActual = amplitudPromedio;

  // Indicador de estado del micrófono (Detecta si hay señal activa)
  display.setCursor(4, 13);
  if (ampActual > 150000) { // Umbral bajo solo para confirmar que entra voz
    display.print("MIC: [REC] Captando");
  } else {
    display.print("MIC: [ON]  Habla...");
  }

  // Barra de nivel de volumen en tiempo real
  int anchoBarraOLED = map(constrain(ampActual, 0, SOUND_THRESHOLD), 0, SOUND_THRESHOLD, 0, 120);
  display.drawRect(4, 24, 120, 8, SSD1306_WHITE);
  if (anchoBarraOLED > 2) {
    display.fillRect(6, 26, anchoBarraOLED - 4, 4, SSD1306_WHITE);
  }

  // Historial gráfico inferior (espectrograma de barras)
  int histH = map(constrain(ampActual, 0, SOUND_THRESHOLD), 0, SOUND_THRESHOLD, 0, 24);
  barHistory[barIndex] = histH;
  barIndex = (barIndex + 1) % BAR_HISTORY;

  int barWidth = SCREEN_WIDTH / BAR_HISTORY;
  int baseY = 62;

  for (int i = 0; i < BAR_HISTORY; i++) {
    int idx = (barIndex + i) % BAR_HISTORY;
    int h = barHistory[idx];
    int x = i * barWidth;
    if (h > 0) {
      display.fillRect(x, baseY - h, barWidth - 1, h, SSD1306_WHITE);
    }
  }
  display.drawLine(0, baseY, SCREEN_WIDTH - 1, baseY, SSD1306_WHITE);

  display.display();
  imprimirBarraTerminal(ampActual, SOUND_THRESHOLD);
}

// --- PANTALLA: ONDA AC SIMULADA ---
void dibujarOndaSimulada() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESTADO: MUESTREO");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(4, 12);
  display.println("S3-CAM disparada...");
  display.setCursor(4, 22);
  display.println("Calculando FFT Core 1");

  int amplitud = 12;       
  int centroY = 46;        
  float frecuencia = 0.15; 

  for(int x = 0; x < SCREEN_WIDTH; x++) {
    int y = centroY + (int)(amplitud * sin(frecuencia * x + faseOnda));
    display.drawPixel(x, y, SSD1306_WHITE);
  }
  
  faseOnda += 0.4; 
  display.display();
}

// --- PANTALLA: DESPLIEGUE DE RESULTADOS DSP ---
void mostrarPantallaDSP() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ANALISIS DSP (Core 1)");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(2, 13);
  display.printf("F1 (60Hz):  %.0f\n", magFundamental);
  display.setCursor(2, 22);
  display.printf("H3 (180Hz): %.0f\n", magArmonico3);
  display.setCursor(2, 31);
  display.printf("H5 (300Hz): %.0f\n", magArmonico5);
  display.setCursor(2, 40);
  display.printf("H7 (420Hz): %.0f\n", magArmonico7);

  display.drawLine(0, 50, 128, 50, SSD1306_WHITE);
  display.setCursor(2, 53);
  display.printf("THD: %.1f %%", porcentajeTHD);
  
  #if MODO_SIMULACION == 1
  display.setCursor(80, 53);
  display.print("[SIM]");
  #endif

  display.display();
}

// --- PANTALLA: ACTUALIZACIÓN GENERAL ---
void actualizarPantalla() {
  display.clearDisplay();
  
  if (estadoActual == STANDBY) {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(22, 2);
    display.println("STANDBY");
    display.drawBitmap(44, 22, epd_bitmap_spidy, 40, 40, SSD1306_WHITE);
    display.display();
  }
}

// --- ETAPA: MUESTREO (DISPARO CAM + DSP CORE 1 + VISUALIZACIÓN) ---
void ejecutarMuestreo() {
  Serial.println("[FSM] -> Entrando a estado MUESTREO.");
  
  // 1. Disparar ESP32-S3-CAM si no se disparó justo al terminar la captura de voz
  if (!camaraDisparadaEnCiclo) {
    enviarDisparoCamara();
  }

  // 2. Disparar tarea DSP en Core 1 mediante semáforo
  xSemaphoreGive(semDispararDSP);

  // 3. Simular visualización de onda AC en OLED mientras Core 1 ejecuta la FFT
  unsigned long inicioMuestreo = millis();
  while (millis() - inicioMuestreo < 1500) {
    dibujarOndaSimulada();
    vTaskDelay(pdMS_TO_TICKS(40));
  }

  // 4. Esperar a que la tarea DSP en Core 1 complete el cálculo
  xSemaphoreTake(semDSPFinalizado, portMAX_DELAY);

  // 5. Desplegar los valores calculados en la pantalla OLED antes de avanzar
  mostrarPantallaDSP();
  vTaskDelay(pdMS_TO_TICKS(3500)); // Tiempo de lectura en pantalla

  // 6. Sincronización: señalar que el muestreo terminó
  muestreoListo = true;
  if (semFSM != NULL) {
    xSemaphoreGive(semFSM);
  }
}

// --- UTILIDADES PARA WAV, BASE64 STREAMING Y CONEXIÓN WI-FI ---
void construirCabeceraWAV(uint8_t* header, uint32_t dataBytes, uint32_t sampleRate) {
  uint32_t fileSizeMinus8 = 36 + dataBytes;
  uint32_t byteRate = sampleRate * 2;
  uint16_t blockAlign = 2;
  uint16_t bitsPerSample = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels = 1;
  uint32_t subchunk1Size = 16;

  memcpy(header + 0,  "RIFF", 4);
  memcpy(header + 4,  &fileSizeMinus8, 4);
  memcpy(header + 8,  "WAVE", 4);
  memcpy(header + 12, "fmt ", 4);
  memcpy(header + 16, &subchunk1Size, 4);
  memcpy(header + 20, &audioFormat, 2);
  memcpy(header + 22, &numChannels, 2);
  memcpy(header + 24, &sampleRate, 4);
  memcpy(header + 28, &byteRate, 4);
  memcpy(header + 32, &blockAlign, 2);
  memcpy(header + 34, &bitsPerSample, 2);
  memcpy(header + 36, "data", 4);
  memcpy(header + 40, &dataBytes, 4);
}

void enviarBase64Stream(WiFiClient& client, const uint8_t* wavHeader, size_t headerLen,
                        const uint8_t* pcmBytes, size_t pcmLen) {
  static const char B64_TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const size_t totalLen = headerLen + pcmLen;
  char outBuf[1024];
  size_t outIdx = 0;
  size_t pos = 0;

  while (pos < totalLen) {
    size_t rem = totalLen - pos;
    uint8_t b0 = (pos < headerLen) ? wavHeader[pos] : pcmBytes[pos - headerLen];
    pos++;
    uint8_t b1 = 0;
    if (rem > 1) {
      b1 = (pos < headerLen) ? wavHeader[pos] : pcmBytes[pos - headerLen];
      pos++;
    }
    uint8_t b2 = 0;
    if (rem > 2) {
      b2 = (pos < headerLen) ? wavHeader[pos] : pcmBytes[pos - headerLen];
      pos++;
    }

    uint32_t triple = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;

    outBuf[outIdx++] = B64_TABLE[(triple >> 18) & 0x3F];
    outBuf[outIdx++] = B64_TABLE[(triple >> 12) & 0x3F];
    outBuf[outIdx++] = (rem > 1) ? B64_TABLE[(triple >> 6) & 0x3F] : '=';
    outBuf[outIdx++] = (rem > 2) ? B64_TABLE[triple & 0x3F] : '=';

    if (outIdx == sizeof(outBuf)) {
      client.write((const uint8_t*)outBuf, outIdx);
      outIdx = 0;
      vTaskDelay(1);
    }
  }
  if (outIdx > 0) {
    client.write((const uint8_t*)outBuf, outIdx);
  }
}

bool asegurarWiFiConectado(unsigned long timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;

  Serial.printf("[WIFI] Conectando a %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - inicio) < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(250));
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] Conectado. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("[WIFI] No se pudo conectar.");
  return false;
}

// Extrae el valor string de un campo JSON específico mientras drena el flujo HTTP
String extraerCampoJsonDeStream(WiFiClient& client, const char* claveObjetivo, unsigned long timeoutMs) {
  String cuerpoParcial = "";
  cuerpoParcial.reserve(2560);
  unsigned long ultimoDato = millis();

  while ((client.connected() || client.available()) && (millis() - ultimoDato) < timeoutMs) {
    if (client.available()) {
      char c = (char)client.read();
      ultimoDato = millis();
      if (cuerpoParcial.length() < 2500) {
        cuerpoParcial += c;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  String patron = "\"";
  patron += claveObjetivo;
  patron += "\"";
  int idx = cuerpoParcial.indexOf(patron);
  if (idx < 0) return "";

  int colon = cuerpoParcial.indexOf(':', idx + patron.length());
  if (colon < 0) return "";

  int startQuote = cuerpoParcial.indexOf('"', colon + 1);
  if (startQuote < 0) return "";

  String valor = "";
  bool escaped = false;
  for (int i = startQuote + 1; i < (int)cuerpoParcial.length(); i++) {
    char c = cuerpoParcial[i];
    if (escaped) {
      if (c == 'n') valor += '\n';
      else if (c == 'r') {}
      else if (c == 't') valor += ' ';
      else valor += c;
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else if (c == '"') {
      break;
    } else {
      valor += c;
    }
  }
  return valor;
}

// Extrae todo el body JSON como string crudo para buscar múltiples campos
String leerBodyHTTP(WiFiClient& client, unsigned long timeoutMs) {
  String body = "";
  body.reserve(4096);
  unsigned long ultimoDato = millis();

  while ((client.connected() || client.available()) && (millis() - ultimoDato) < timeoutMs) {
    if (client.available()) {
      char c = (char)client.read();
      ultimoDato = millis();
      if (body.length() < 4000) {
        body += c;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
  return body;
}

// Extrae un campo string de un JSON ya leído
String extraerCampoDeJson(const String& json, const char* clave) {
  String patron = "\"";
  patron += clave;
  patron += "\"";
  int idx = json.indexOf(patron);
  if (idx < 0) return "";

  int colon = json.indexOf(':', idx + patron.length());
  if (colon < 0) return "";

  int startQuote = json.indexOf('"', colon + 1);
  if (startQuote < 0) return "";

  String valor = "";
  bool escaped = false;
  for (int i = startQuote + 1; i < (int)json.length(); i++) {
    char c = json[i];
    if (escaped) {
      if (c == 'n') valor += '\n';
      else if (c == 'r') {}
      else if (c == 't') valor += ' ';
      else valor += c;
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else if (c == '"') {
      break;
    } else {
      valor += c;
    }
  }
  return valor;
}

// --- ETAPA: ENVÍO HTTP REAL (AUDIO WAV + MÉTRICAS FFT A /auditar) ---
void ejecutarEnvioHTTP() {
  Serial.println("[FSM] -> Entrando a estado ENVIO HTTP.");
  ultimoHttpOk = false;
  ultimoHttpCodigo = 0;
  resumenOledServidor = "";

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESTADO: ENVIO HTTP");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(4, 15);
  display.println("Conectando Wi-Fi...");
  display.display();

  if (!asegurarWiFiConectado(10000)) {
    display.setCursor(4, 30);
    display.println("ERROR: Sin Wi-Fi");
    display.display();
    vTaskDelay(pdMS_TO_TICKS(1500));
    httpListo = true;
    if (semFSM != NULL) xSemaphoreGive(semFSM);
    return;
  }

  size_t muestrasValidas = (audioPCM != NULL) ? audioSampleIndex : 0;
  uint32_t dataBytes = (uint32_t)(muestrasValidas * sizeof(int16_t));
  uint8_t wavHeader[44];
  bool tieneAudio = (muestrasValidas > 0 && audioPCM != NULL);

  if (tieneAudio) {
    construirCabeceraWAV(wavHeader, dataBytes, AUDIO_SAMPLE_RATE);
  }

  char jsonPrefix[256];
  snprintf(jsonPrefix, sizeof(jsonPrefix),
           "{\"fundamental\":%.2f,\"h3\":%.2f,\"h5\":%.2f,\"h7\":%.2f,\"thd\":%.2f,\"audio_base64\":\"",
           frecuenciaDominante, magArmonico3, magArmonico5, magArmonico7, porcentajeTHD);
  const char* jsonSuffix = "\"}";

  size_t rawAudioTotal = tieneAudio ? (sizeof(wavHeader) + dataBytes) : 0;
  size_t b64Len = tieneAudio ? (((rawAudioTotal + 2) / 3) * 4) : 0;
  size_t contentLength = strlen(jsonPrefix) + b64Len + strlen(jsonSuffix);

  display.setCursor(4, 27);
  display.printf("POST %s", SERVER_PATH);
  display.setCursor(4, 39);
  display.printf("Audio: %u B + FFT", (unsigned int)rawAudioTotal);
  display.display();

  Serial.printf("[HTTP] Conectando a %s:%u%s (Content-Length: %u bytes, muestras: %u)...\n",
                SERVER_HOST, SERVER_PORT, SERVER_PATH,
                (unsigned int)contentLength, (unsigned int)muestrasValidas);

  WiFiClient client;
  if (client.connect(SERVER_HOST, SERVER_PORT)) {
    client.printf("POST %s HTTP/1.1\r\n", SERVER_PATH);
    client.printf("Host: %s:%u\r\n", SERVER_HOST, SERVER_PORT);
    client.print("Content-Type: application/json\r\n");
    client.printf("Content-Length: %u\r\n", (unsigned int)contentLength);
    client.print("Connection: close\r\n\r\n");

    client.print(jsonPrefix);
    if (tieneAudio) {
      enviarBase64Stream(client, wavHeader, sizeof(wavHeader),
                         (const uint8_t*)audioPCM, dataBytes);
    }
    client.print(jsonSuffix);

    // Esperar respuesta del servidor (STT + Gemini + TTS puede tomar varios segundos)
    unsigned long waitStart = millis();
    while (client.connected() && !client.available() && (millis() - waitStart < 25000)) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (client.available()) {
      String statusLine = client.readStringUntil('\n');
      statusLine.trim();
      Serial.printf("[HTTP] Respuesta: %s\n", statusLine.c_str());

      int sp1 = statusLine.indexOf(' ');
      if (sp1 > 0) {
        ultimoHttpCodigo = statusLine.substring(sp1 + 1).toInt();
      }
      ultimoHttpOk = (ultimoHttpCodigo >= 200 && ultimoHttpCodigo < 300);

      // Saltar encabezados HTTP hasta línea vacía
      while (client.connected() || client.available()) {
        String hLine = client.readStringUntil('\n');
        if (hLine == "\r" || hLine.length() == 0) break;
      }

      // Extraer resumen_oled del JSON (el audio del veredicto se reproduce por streaming desde /veredicto-wav)
      extraerResumenYAudioDeStream(client, &resumenOledServidor, NULL, 0, NULL, 15000);
      if (resumenOledServidor.length() > 0) {
        Serial.println("[HTTP] Resumen OLED recibido:");
        Serial.println(resumenOledServidor);
      }
    } else {
      Serial.println("[HTTP] Timeout esperando respuesta del servidor.");
    }
    client.stop();
  } else {
    Serial.printf("[HTTP] ERROR: No se pudo conectar a %s:%u\n", SERVER_HOST, SERVER_PORT);
  }

  display.setCursor(4, 52);
  if (ultimoHttpOk) {
    display.printf("Respuesta: %d OK!", ultimoHttpCodigo);
  } else if (ultimoHttpCodigo > 0) {
    display.printf("Error HTTP: %d", ultimoHttpCodigo);
  } else {
    display.println("Error de conexion");
  }
  display.display();
  vTaskDelay(pdMS_TO_TICKS(800));

  // Sincronización: señalar que la comunicación HTTP terminó
  httpListo = true;
  if (semFSM != NULL) {
    xSemaphoreGive(semFSM);
  }
}

// --- ETAPA: RESULTADO (VEREDICTO EN OLED + AUDIO TTS POR SPEAKER) ---
void mostrarResultadoYFinalizar() {
  Serial.println("[FSM] -> Entrando a estado RESULTADO (Dictamen).");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESTADO: DICTAMEN");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  if (ultimoHttpOk && resumenOledServidor.length() > 0) {
    int y = 14;
    int start = 0;
    while (start < (int)resumenOledServidor.length() && y <= 44) {
      int nl = resumenOledServidor.indexOf('\n', start);
      String linea = (nl >= 0) ? resumenOledServidor.substring(start, nl) : resumenOledServidor.substring(start);
      if (linea.length() > 21) linea = linea.substring(0, 21);
      display.setCursor(2, y);
      display.println(linea);
      y += 11;
      if (nl < 0) break;
      start = nl + 1;
    }
  } else {
    display.setCursor(4, 14);
    display.printf("Freq. Pico: %.1f Hz\n", frecuenciaDominante);

    display.setCursor(4, 25);
    display.printf("THD: %.1f %% | %s\n", porcentajeTHD, ultimoHttpOk ? "OK (200)" : "LOCAL");

    display.setCursor(4, 37);
    display.setTextSize(2);
    if (esIncandescente || frecuenciaDominante == 0.0) {
      display.println("INCANDESC.");
    } else {
      display.println("OTRA CARGA");
    }
  }

  display.setTextSize(1);
  display.setCursor(4, 55);
  display.println(ultimoHttpOk ? "Hablando veredicto..." : "Modo local (sin red)");
  display.display();

  unsigned long inicioDictamen = millis();

  // *** REPRODUCIR RESUMEN DEL VEREDICTO POR MAX98357A EN STREAMING ***
  if (ultimoHttpOk) {
    Serial.println("[RESULTADO] Reproduciendo resumen del veredicto por speaker...");
    reproducirWavDesdeEndpoint(SERVER_VEREDICTO_WAV);
  }

  // Garantizar al menos 4 segundos visibles en pantalla para leer el dictamen
  unsigned long transcurridoDictamen = millis() - inicioDictamen;
  if (transcurridoDictamen < 4000) {
    vTaskDelay(pdMS_TO_TICKS(4000 - transcurridoDictamen));
  }

  // Mostrar mensaje de regreso
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(4, 28);
  display.println("Regresando a Standby...");
  display.display();
  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial.println("[FSM] Tiempo de dictamen cumplido. Regresando a STANDBY.");

  // Reiniciar banderas de sincronización para el siguiente ciclo
  audioListo = false;
  muestreoListo = false;
  httpListo = false;
  camaraDisparadaEnCiclo = false;

  estadoActual = STANDBY;
  actualizarPantalla();
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  pinMode(BOTON_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_CAM_PIN, OUTPUT);
  digitalWrite(TRIGGER_CAM_PIN, LOW);

  Wire.begin(OLED_SDA, OLED_SCL);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error: No se encontró la pantalla OLED"));
    for(;;); 
  }

  // Inicializar semáforos de FreeRTOS para sincronización de tareas
  semFSM = xSemaphoreCreateBinary();
  semDispararDSP = xSemaphoreCreateBinary();
  semDSPFinalizado = xSemaphoreCreateBinary();
  semMuestreoListo = xSemaphoreCreateBinary();

  // Configurar Timer por hardware del ESP32 (fs = 2048 Hz)
  configurarTimerHardware();

  // Crear e inicializar Tarea DSP fijada al Core 1
  xTaskCreatePinnedToCore(
    tareaDSPCore1,      // Función de la tarea
    "TareaDSPCore1",    // Nombre de la tarea
    4096,               // Tamaño de stack (bytes)
    NULL,               // Parámetros
    2,                  // Prioridad
    &taskDSPHandle,     // Handle de la tarea
    1                   // Fijada a Core 1
  );

  // Reservar buffer de audio PCM (4s @ 8000Hz 16-bit = 64KB, compartido con TTS)
  audioPCM = (int16_t*)malloc(AUDIO_MAX_SAMPLES * sizeof(int16_t));
  if (audioPCM == NULL) {
    Serial.println("[WARN] No se pudo reservar buffer de audio en heap.");
  } else {
    Serial.printf("[SETUP] Buffer de audio reservado (%u bytes). Heap libre: %u\n",
                  (unsigned int)(AUDIO_MAX_SAMPLES * sizeof(int16_t)), ESP.getFreeHeap());
  }

  // Inicializar micrófono I2S y tarea de captura continua
  initI2SMic();
  xTaskCreatePinnedToCore(
    tareaAudioI2S,
    "TareaAudioI2S",
    4096,
    NULL,
    2,
    &taskAudioHandle,
    0
  );
  Serial.println("[SETUP] Microfono I2S INMP441 y Modulo DSP inicializados.");

  // Inicializar amplificador MAX98357A (I2S Speaker)
  initI2SSpeaker();

  // Iniciar conexión Wi-Fi en segundo plano
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[SETUP] Conectando Wi-Fi (%s)...\n", WIFI_SSID);

  memset(barHistory, 0, sizeof(barHistory));
  
  actualizarPantalla();

  // Mostrar mensaje de bienvenida en OLED mientras se conecta Wi-Fi
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 5);
  display.println("SISTEMA AUDITORIA");
  display.setCursor(16, 18);
  display.println("ELECTRICA v2.0");
  display.drawLine(0, 28, 128, 28, SSD1306_WHITE);
  display.setCursor(4, 33);
  display.println("Conectando Wi-Fi...");
  display.setCursor(4, 46);
  display.println("Esperando saludo IA");
  display.display();
}

// --- LOOP PRINCIPAL (CONTROL DE LA MÁQUINA DE ESTADOS) ---
void loop() {
  switch (estadoActual) {
    case STANDBY: {
      // Saludo de bienvenida de la IA al estar en STANDBY con Wi-Fi listo
      if (!saludoReproducido && intentosSaludo < 3 && WiFi.status() == WL_CONNECTED &&
          (intentosSaludo == 0 || (millis() - ultimoIntentoSaludoMs) > 5000)) {
        Serial.println("[FSM] Wi-Fi conectado. Solicitando saludo de la IA...");
        intentosSaludo++;
        ultimoIntentoSaludoMs = millis();

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(4, 5);
        display.println("BIENVENIDO!");
        display.drawLine(0, 15, 128, 15, SSD1306_WHITE);
        display.setCursor(4, 20);
        display.println("La IA te saluda...");
        display.drawBitmap(44, 28, epd_bitmap_spidy, 40, 40, SSD1306_WHITE);
        display.display();

        if (solicitarSaludoServidor()) {
          saludoReproducido = true;
        }
        actualizarPantalla();
      }

      // El botón físico ÚNICAMENTE tiene efecto en este estado
      int lecturaBoton = digitalRead(BOTON_PIN);

      if (lecturaBoton != estadoBotonAnterior) {
        ultimoTiempoRebote = millis();
      }

      if ((millis() - ultimoTiempoRebote) > retardoRebote) {
        if (lecturaBoton == LOW) {
          Serial.println("[FSM] Boton presionado: Saliendo de STANDBY -> CAPTURA_VOZ");
          audioListo = false;
          muestreoListo = false;
          httpListo = false;
          camaraDisparadaEnCiclo = false;

          i2s_zero_dma_buffer(I2S_PORT);
          audioSampleIndex = 0;
          amplitudPromedio = 0;
          grabandoAudio = true;

          estadoActual = CAPTURA_VOZ;
          inicioCapturaVoz = millis(); // Inicia el cronómetro de escucha
          actualizarPantalla();

          // Esperar a que se libere el botón (sin bloquear FreeRTOS)
          while (digitalRead(BOTON_PIN) == LOW) {
            vTaskDelay(pdMS_TO_TICKS(10));
          }
        }
      }
      estadoBotonAnterior = lecturaBoton;
      vTaskDelay(pdMS_TO_TICKS(10));
      break;
    }
    case CAPTURA_VOZ: {
      unsigned long transcurrido = millis() - inicioCapturaVoz;
      if (transcurrido < TIEMPO_ESCUCHA_MS) {
        // Sigue dentro de los 4 segundos de escucha
        dibujarPantallaMicrofono(TIEMPO_ESCUCHA_MS - transcurrido);
      } else {
        // Se cumplió el tiempo: detener captura de audio y disparar cámara de inmediato
        grabandoAudio = false;
        vTaskDelay(pdMS_TO_TICKS(60));
        Serial.printf("[FSM] Tiempo de escucha finalizado (%u muestras capturadas) -> MUESTREO\n", (unsigned int)audioSampleIndex);
        audioListo = true;
        if (semFSM != NULL) {
          xSemaphoreGive(semFSM);
        }
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(16, 28);
        display.println("PREGUNTA CAPTURADA");
        display.display();

        // Disparar la cámara inmediatamente después de capturar la voz (superpuesto al mensaje)
        enviarDisparoCamara();

        estadoActual = MUESTREO;
        actualizarPantalla();
      }
      vTaskDelay(pdMS_TO_TICKS(40));
      break;
    }
    case MUESTREO: {
      ejecutarMuestreo();
      // Al terminar el muestreo (muestreoListo == true), avanza automáticamente a ENVIO_HTTP
      if (muestreoListo) {
        Serial.println("[FSM] Avanzando automaticamente: MUESTREO -> ENVIO_HTTP");
        estadoActual = ENVIO_HTTP;
        actualizarPantalla();
      }
      break;
    }
    case ENVIO_HTTP: {
      ejecutarEnvioHTTP();
      // Al recibir la respuesta HTTP (httpListo == true), avanza automáticamente a RESULTADO
      if (httpListo) {
        Serial.println("[FSM] Avanzando automaticamente: ENVIO_HTTP -> RESULTADO");
        estadoActual = RESULTADO;
        actualizarPantalla();
      }
      break;
    }
    case RESULTADO: {
      mostrarResultadoYFinalizar();
      // Regresa automáticamente a STANDBY tras reproducir el audio (lo hace mostrarResultadoYFinalizar)
      break;
    }
  }
}
