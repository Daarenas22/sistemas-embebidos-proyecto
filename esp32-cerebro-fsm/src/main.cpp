#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>

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

// --- PINES I2S PARA MICRÓFONO INMP441 (LADO DERECHO) ---
#define I2S_WS   15   // Word Select (LRCK) - Pin marcado D15
#define I2S_SCK   4   // Serial Clock (BCLK) - Pin marcado D4
#define I2S_SD   19   // Serial Data (DOUT del mic) - Pin marcado D19
#define I2S_PORT I2S_NUM_0

// Umbral de sonido calibrado para detección de audio
#define SOUND_THRESHOLD 2200000
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

// --- VARIABLES PARA VISUALIZACIÓN DEL MICRÓFONO ---
#define MIC_SAMPLES 256
int32_t micBuffer[MIC_SAMPLES];
int32_t amplitudPromedio = 0;

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
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// --- INICIALIZACIÓN DE HARDWARE I2S ---
void initI2SMic() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 32000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
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

// --- DISPARO DIGITAL A LA CÁMARA (ESP32-S3-CAM) ---
void enviarDisparoCamara() {
  Serial.printf("[WROOM] Activando pulso digital hacia ESP32-S3-CAM (GPIO %d)...\n", TRIGGER_CAM_PIN);
  digitalWrite(TRIGGER_CAM_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(200));
  digitalWrite(TRIGGER_CAM_PIN, LOW);
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

// --- LECTURA DEL MICRÓFONO I2S PICO-A-PICO ---
void leerMicrofono() {
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
  }
}

// --- PANTALLA: CAPTURA DE VOZ ---
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

  leerMicrofono();

  // Indicador de estado del micrófono (Detecta si hay señal activa)
  display.setCursor(4, 13);
  if (amplitudPromedio > 150000) { // Umbral bajo solo para confirmar que entra voz
    display.print("MIC: [REC] Captando");
  } else {
    display.print("MIC: [ON]  Habla...");
  }

  // Barra de nivel de volumen en tiempo real
  int anchoBarraOLED = map(constrain(amplitudPromedio, 0, SOUND_THRESHOLD), 0, SOUND_THRESHOLD, 0, 120);
  display.drawRect(4, 24, 120, 8, SSD1306_WHITE);
  if (anchoBarraOLED > 2) {
    display.fillRect(6, 26, anchoBarraOLED - 4, 4, SSD1306_WHITE);
  }

  // Historial gráfico inferior (espectrograma de barras)
  int histH = map(constrain(amplitudPromedio, 0, SOUND_THRESHOLD), 0, SOUND_THRESHOLD, 0, 24);
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
  imprimirBarraTerminal(amplitudPromedio, SOUND_THRESHOLD);
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
  
  // 1. Activar pulso digital en GPIO para disparar ESP32-S3-CAM
  enviarDisparoCamara();

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

// --- ETAPA: ENVÍO HTTP (SIMULACIÓN DE TELEMETRÍA Y RESPUESTA) ---
void ejecutarEnvioHTTP() {
  Serial.println("[FSM] -> Entrando a estado ENVIO HTTP.");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESTADO: ENVIO HTTP");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(4, 16);
  display.println("Conectando al servidor...");
  display.setCursor(4, 28);
  display.println("POST /api/telemetria");
  display.setCursor(4, 40);
  display.println("Enviando audio y FFT...");
  display.display();

  // Simulación de latencia de red mediante vTaskDelay
  vTaskDelay(pdMS_TO_TICKS(1500));

  display.setCursor(4, 52);
  display.println("Respuesta: 200 OK!");
  display.display();
  vTaskDelay(pdMS_TO_TICKS(600));

  // Sincronización: señalar que la comunicación HTTP terminó
  httpListo = true;
  if (semFSM != NULL) {
    xSemaphoreGive(semFSM);
  }
}

// --- ETAPA: RESULTADO (VEREDICTO EN OLED DURANTE 4 SEGUNDOS) ---
void mostrarResultadoYFinalizar() {
  Serial.println("[FSM] -> Entrando a estado RESULTADO (Dictamen).");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESTADO: DICTAMEN");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setCursor(4, 14);
  display.printf("Freq. Pico: %.1f Hz\n", frecuenciaDominante);

  display.setCursor(4, 25);
  display.printf("THD: %.1f %% | OK (200)\n", porcentajeTHD);

  display.setCursor(4, 37);
  display.setTextSize(2);
  if (esIncandescente || frecuenciaDominante == 0.0) {
    display.println("INCANDESC.");
  } else {
    display.println("OTRA CARGA");
  }

  display.setTextSize(1);
  display.setCursor(4, 55);
  display.println("Regresando a Standby...");
  display.display();

  // Requisito estricto: mostrar veredicto durante 4 segundos usando vTaskDelay (sin delay())
  vTaskDelay(pdMS_TO_TICKS(4000));

  Serial.println("[FSM] Tiempo de dictamen cumplido. Regresando a STANDBY.");

  // Reiniciar banderas de sincronización para el siguiente ciclo
  audioListo = false;
  muestreoListo = false;
  httpListo = false;

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

  // Inicializar micrófono I2S
  initI2SMic();
  Serial.println("[SETUP] Microfono I2S INMP441 y Modulo DSP inicializados.");

  memset(barHistory, 0, sizeof(barHistory));
  
  actualizarPantalla();
}

// --- LOOP PRINCIPAL (CONTROL DE LA MÁQUINA DE ESTADOS) ---
void loop() {
  switch (estadoActual) {
    case STANDBY: {
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
        // Se cumplió el tiempo: avanzar automáticamente a MUESTREO
        Serial.println("[FSM] Tiempo de escucha finalizado -> MUESTREO");
        audioListo = true;
        if (semFSM != NULL) {
          xSemaphoreGive(semFSM);
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(16, 28);
        display.println("PREGUNTA CAPTURADA");
        display.display();
        vTaskDelay(pdMS_TO_TICKS(500));

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
      // Regresa automáticamente a STANDBY tras los 4 segundos
      break;
    }
  }
}
