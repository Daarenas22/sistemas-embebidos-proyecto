#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// Credenciales Wi-Fi del Hotspot de Windows
const char* ssid = "PC";
const char* password = "12345678";
String serverName = "http://192.168.137.1:5000/upload";

// Pin de activación (GPIO 2 del lado derecho)
#define TRIGGER_PIN GPIO_NUM_2 

// Pines GOOUUU ESP32-S3-CAM
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n[S3-CAM] ¡Despertado exitosamente por la señal del WROOM!");

  // 1. Esperar a que el WROOM baje la señal a LOW para evitar re-disparos erráticos
  pinMode(TRIGGER_PIN, INPUT);
  while (digitalRead(TRIGGER_PIN) == HIGH) {
    delay(10);
  }

  // 2. Inicializar la cámara
  Serial.println("[1/4] Inicializando módulo de cámara...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("[ERROR] No se pudo inicializar la cámara.");
  } else {
    Serial.println("[OK] Cámara configurada correctamente.");

    // 3. Conectar a Wi-Fi
    Serial.println("[2/4] Conectando a la red Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 30) {
      delay(300);
      Serial.print(".");
      intentos++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[OK] Wi-Fi conectado exitosamente.");
      
      // 4. Tomar foto y realizar HTTP POST
      Serial.println("[3/4] Capturando imagen y enviando al servidor Flask...");
      camera_fb_t * fb = esp_camera_fb_get();
      if (fb) {
        HTTPClient http;
        http.begin(serverName);
        http.addHeader("Content-Type", "image/jpeg");
        
        int httpCode = http.POST(fb->buf, fb->len);
        if (httpCode > 0) {
          Serial.printf("[ÉXITO] Foto enviada al servidor (Respuesta HTTP: %d)\n", httpCode);
        } else {
          Serial.printf("[ERROR] Fallo al enviar HTTP: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
        esp_camera_fb_return(fb);
      } else {
        Serial.println("[ERROR] Falló la captura del cuadro de la cámara.");
      }
    } else {
      Serial.println("\n[ERROR] Tiempo agotado: No se pudo conectar al Wi-Fi.");
    }
  }

  // 5. Configurar despertador y entrar en reposo profundo
  Serial.println("[4/4] Configurando despertador y entrando en Deep Sleep...");
  esp_sleep_enable_ext0_wakeup(TRIGGER_PIN, 1); // Despierta cuando GPIO 2 sea HIGH
  esp_deep_sleep_start();
}

void loop() {
  // El ciclo permanece vacío porque al despertar ejecuta setup() desde el inicio
}