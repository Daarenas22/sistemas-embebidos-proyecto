#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

// Credenciales Wi-Fi del Hotspot de Windows y servidor FastAPI
const char* ssid = "PC";
const char* password = "12345678";
const char* serverHost = "192.168.137.1";
const uint16_t serverPort = 8000;
const char* serverStreamPath = "/foto-stream";

// Cantidad de fotos en ráfaga (efecto video en tiempo real)
#define TOTAL_FOTOS_RAFAGA 28

// Pin de activación (GPIO 14 del lado derecho) – cambio para evitar pins de strapping
#define TRIGGER_PIN GPIO_NUM_14
#define TRIGGER_PIN GPIO_NUM_2

// Tiempo mínimo de estabilización al arrancar antes de leer el pin de disparo.
// Evita falsas capturas provocadas por el boot/reset del módulo.
#define BOOT_IGNORE_MS 3000

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

bool wifiConectado = false;
bool camaraLista = false;

void asegurarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    return;
  }
  Serial.println("[S3-CAM] Conectando a la red Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 30) {
    delay(300);
    Serial.print(".");
    intentos++;
  }
  Serial.println();
  wifiConectado = (WiFi.status() == WL_CONNECTED);
  if (wifiConectado) {
    Serial.print("[OK] Wi-Fi conectado. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[ERROR] Tiempo agotado: No se pudo conectar al Wi-Fi.");
  }
}

void enviarRafagaFotos() {
  if (!camaraLista) {
    Serial.println("[ERROR] Cámara no inicializada.");
    return;
  }
  asegurarWiFi();
  if (!wifiConectado) return;

  // Descartar cuadros iniciales para estabilizar exposición y vaciar buffer
  for (int w = 0; w < 3; w++) {
    camera_fb_t * warm = esp_camera_fb_get();
    if (warm) esp_camera_fb_return(warm);
    delay(40);
  }

  Serial.printf("[CAM] Iniciando ráfaga de %d fotos hacia http://%s:%u%s ...\n",
                TOTAL_FOTOS_RAFAGA, serverHost, serverPort, serverStreamPath);

  WiFiClient client;
  String boundary = "----ESP32S3CAMBoundary7MA4YWxkTrZu0gW";
  String bodyHead = "--" + boundary + "\r\n"
                    "Content-Disposition: form-data; name=\"file\"; filename=\"captura.jpg\"\r\n"
                    "Content-Type: image/jpeg\r\n\r\n";
  String bodyTail = "\r\n--" + boundary + "--\r\n";

  int fotosEnviadas = 0;
  for (int idx = 1; idx <= TOTAL_FOTOS_RAFAGA; idx++) {
    bool esFinal = (idx == TOTAL_FOTOS_RAFAGA);

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.printf("[WARN] Frame %d/%d falló en captura.\n", idx, TOTAL_FOTOS_RAFAGA);
      delay(20);
      continue;
    }

    if (!client.connected()) {
      if (!client.connect(serverHost, serverPort)) {
        Serial.printf("[ERROR] Frame %d: no se pudo conectar a %s:%u\n", idx, serverHost, serverPort);
        esp_camera_fb_return(fb);
        delay(50);
        continue;
      }
    }

    size_t totalLen = bodyHead.length() + fb->len + bodyTail.length();

    client.printf("POST %s?frame=%d&total=%d&final=%d HTTP/1.1\r\n",
                  serverStreamPath, idx, TOTAL_FOTOS_RAFAGA, esFinal ? 1 : 0);
    client.printf("Host: %s:%u\r\n", serverHost, serverPort);
    client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary.c_str());
    client.printf("Content-Length: %u\r\n", (unsigned int)totalLen);
    client.printf("Connection: %s\r\n\r\n", esFinal ? "close" : "keep-alive");

    client.print(bodyHead);

    const size_t chunkSize = 1024;
    size_t sent = 0;
    while (sent < fb->len) {
      size_t toSend = fb->len - sent;
      if (toSend > chunkSize) toSend = chunkSize;
      size_t written = client.write(fb->buf + sent, toSend);
      if (written == 0) {
        delay(1);
        if (!client.connected()) break;
      }
      sent += written;
    }

    client.print(bodyTail);
    esp_camera_fb_return(fb);

    unsigned long timeoutStart = millis();
    unsigned long maxWait = esFinal ? 10000 : 2500;
    while (client.connected() && !client.available() && (millis() - timeoutStart < maxWait)) {
      delay(2);
    }

    int contentLength = -1;
    bool serverClose = false;
    String statusLine = "";
    if (client.available()) {
      statusLine = client.readStringUntil('\n');
      statusLine.trim();

      while (client.connected() || client.available()) {
        String hLine = client.readStringUntil('\n');
        hLine.trim();
        if (hLine.length() == 0) break;
        String lower = hLine;
        lower.toLowerCase();
        if (lower.startsWith("content-length:")) {
          contentLength = lower.substring(15).toInt();
        } else if (lower.startsWith("connection:") && lower.indexOf("close") >= 0) {
          serverClose = true;
        }
      }

      if (contentLength > 0) {
        int readBytes = 0;
        unsigned long bodyStart = millis();
        while (readBytes < contentLength && (client.connected() || client.available()) && (millis() - bodyStart < 2000)) {
          if (client.available()) {
            client.read();
            readBytes++;
          } else {
            delay(1);
          }
        }
      } else {
        while (client.available()) client.read();
      }
      fotosEnviadas++;
    }

    if (esFinal || serverClose) {
      client.stop();
    }
    Serial.printf("[CAM] Frame %d/%d -> %s\n", idx, TOTAL_FOTOS_RAFAGA, statusLine.c_str());
  }

  if (client.connected()) client.stop();
  Serial.printf("[ÉXITO] Ráfaga completada: %d/%d fotos enviadas.\n", fotosEnviadas, TOTAL_FOTOS_RAFAGA);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIGGER_PIN, INPUT); // Simple INPUT, external pull‑down resistor if needed


  Serial.println("\n[S3-CAM] Iniciando módulo de cámara...");
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
  config.jpeg_quality = 14;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("[ERROR] No se pudo inicializar la cámara.");
    camaraLista = false;
  } else {
    Serial.println("[OK] Cámara configurada correctamente.");
    camaraLista = true;
    asegurarWiFi();
  }

  Serial.println("[S3-CAM] Listo. Esperando pulso HIGH en GPIO 2 desde el WROOM...");
}

// --- VARIABLES DE DETECCIÓN DE FLANCO ---
static bool pinAnterior = LOW;
static bool listo       = false;         // false durante BOOT_IGNORE_MS
static unsigned long tArranque = 0;      // se inicializa en loop() la primera vez

void loop() {
  // Inicializar el timestamp de arranque la primera vez
  if (tArranque == 0) {
    tArranque = millis();
    Serial.println("[S3-CAM] Periodo de estabilizacion de boot activo...");
  }

  // Ignorar cualquier señal durante los primeros BOOT_IGNORE_MS ms
  if (!listo) {
    if (millis() - tArranque >= BOOT_IGNORE_MS) {
      listo      = true;
      pinAnterior = (digitalRead(TRIGGER_PIN) == HIGH); // estado real actual
      Serial.println("[S3-CAM] Listo. Escuchando flanco de subida en GPIO 2...");
    } else {
      delay(10);
      return;
    }
  }

  // Detección de FLANCO DE SUBIDA: solo actúa en la transición LOW -> HIGH
  bool pinActual = (digitalRead(TRIGGER_PIN) == HIGH);

  if (pinActual && !pinAnterior) {
    // ¡Flanco de subida detectado!
    Serial.println("\n[S3-CAM] ¡Flanco de subida detectado en GPIO 2! Iniciando rafaga...");
    enviarRafagaFotos();
    Serial.println("[S3-CAM] Rafaga terminada. Esperando proximo disparo en GPIO 2...");
  }

  pinAnterior = pinActual;
  delay(5);  // poll cada 5 ms (~200 Hz), suficiente para un pulso de 200 ms
}