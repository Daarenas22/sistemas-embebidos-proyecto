#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <string.h>

// Credenciales Wi-Fi del Hotspot de Windows y servidor FastAPI
const char* ssid = "PC";
const char* password = "12345678";
const char* serverHost = "192.168.137.1";
const uint16_t serverPort = 8000;
const char* serverStreamPath = "/foto-stream";
const char* serverCamWaitPath = "/cam-esperar";
const char* serverCamStatePath = "/cam-estado";

// Cantidad de fotos en ráfaga (efecto video en tiempo real)
#define TOTAL_FOTOS_RAFAGA 28

// Pin de activación hardware desde el ESP32-WROOM (GPIO 17 -> GPIO 2)
#define TRIGGER_PIN GPIO_NUM_2

// Tiempo mínimo de estabilización al arrancar antes de leer el pin de disparo.
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

// Estado de sincronización remota con el servidor (respaldo del disparo GPIO)
WiFiClient pollClient;
bool pollRequestActive = false;
unsigned long pollStartedMs = 0;
unsigned long lastPollRetryMs = 0;
long ultimoTriggerIdRemoto = -1;
unsigned long ultimaRafagaMs = 0;

// Variables de detección de flanco hardware
static bool pinAnterior = false;
static bool listo = false;
static unsigned long tArranque = 0;

long extraerEnteroJson(const String& json, const char* clave, long valorDefecto) {
  String patron = "\"";
  patron += clave;
  patron += "\"";
  int idx = json.indexOf(patron);
  if (idx < 0) return valorDefecto;
  int colon = json.indexOf(':', idx + patron.length());
  if (colon < 0) return valorDefecto;

  int i = colon + 1;
  while (i < (int)json.length() && (json[i] == ' ' || json[i] == '\t')) i++;
  if (i >= (int)json.length()) return valorDefecto;

  bool neg = false;
  if (json[i] == '-') {
    neg = true;
    i++;
  }
  long val = 0;
  bool hayDigito = false;
  while (i < (int)json.length() && json[i] >= '0' && json[i] <= '9') {
    hayDigito = true;
    val = val * 10 + (json[i] - '0');
    i++;
  }
  if (!hayDigito) return valorDefecto;
  return neg ? -val : val;
}

bool extraerBoolJson(const String& json, const char* clave) {
  String patron = "\"";
  patron += clave;
  patron += "\"";
  int idx = json.indexOf(patron);
  if (idx < 0) return false;
  int colon = json.indexOf(':', idx + patron.length());
  if (colon < 0) return false;
  String resto = json.substring(colon + 1, min((int)json.length(), colon + 12));
  resto.trim();
  return resto.startsWith("true");
}

bool inicializarCamara() {
  camera_config_t config = {};
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
  config.sccb_i2c_port = 0;

  if (psramFound()) {
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  for (int intento = 1; intento <= 3; intento++) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      Serial.println("[OK] Cámara configurada correctamente.");
      camaraLista = true;
      return true;
    }
    Serial.printf("[WARN] esp_camera_init intento %d/3 falló (0x%x).\n", intento, err);
    esp_camera_deinit();
    delay(300);
  }

  Serial.println("[ERROR] No se pudo inicializar la cámara tras 3 intentos.");
  camaraLista = false;
  return false;
}

void asegurarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    return;
  }
  Serial.println("[S3-CAM] Conectando a la red Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.disconnect(false);
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

String leerRespuestaCortaHTTP(WiFiClient& client, unsigned long timeoutMs) {
  client.setTimeout(1500);
  unsigned long waitStart = millis();
  while (client.connected() && !client.available() && (millis() - waitStart < timeoutMs)) {
    delay(2);
  }
  if (!client.available()) return "";

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();

  int contentLength = -1;
  while (client.connected() || client.available()) {
    String hLine = client.readStringUntil('\n');
    hLine.trim();
    if (hLine.length() == 0) break;
    String lower = hLine;
    lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      contentLength = lower.substring(15).toInt();
    }
  }

  String body = "";
  unsigned long bodyStart = millis();
  if (contentLength > 0) {
    while ((int)body.length() < contentLength && (client.connected() || client.available()) &&
           (millis() - bodyStart < 1500)) {
      if (client.available()) {
        char c = (char)client.read();
        if (body.length() < 512) body += c;
      } else {
        delay(1);
      }
    }
  } else {
    while (client.available() && body.length() < 512) {
      body += (char)client.read();
    }
  }
  return body;
}

void sincronizarTriggerServidor() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClient c;
  if (!c.connect(serverHost, serverPort, 800)) return;

  c.printf("GET %s HTTP/1.1\r\n", serverCamStatePath);
  c.printf("Host: %s:%u\r\n", serverHost, serverPort);
  c.print("Connection: close\r\n\r\n");

  String body = leerRespuestaCortaHTTP(c, 1500);
  c.stop();

  long tid = extraerEnteroJson(body, "trigger_id", -1);
  if (tid >= 0) {
    ultimoTriggerIdRemoto = tid;
    Serial.printf("[S3-CAM] Sincronizado trigger_id inicial del servidor: %ld\n", ultimoTriggerIdRemoto);
  }
}

void enviarRafagaFotos() {
  if (!camaraLista && !inicializarCamara()) {
    Serial.println("[ERROR] Cámara no inicializada.");
    return;
  }
  asegurarWiFi();
  if (!wifiConectado) return;

  // Cerrar long-poll antes de abrir la conexión de ráfaga
  if (pollRequestActive || pollClient.connected()) {
    pollClient.stop();
    pollRequestActive = false;
  }

  // Descartar cuadros iniciales para estabilizar exposición y vaciar buffer
  for (int w = 0; w < 3; w++) {
    camera_fb_t * warm = esp_camera_fb_get();
    if (warm) esp_camera_fb_return(warm);
    delay(30);
  }

  Serial.printf("[CAM] Iniciando ráfaga de %d fotos hacia http://%s:%u%s ...\n",
                TOTAL_FOTOS_RAFAGA, serverHost, serverPort, serverStreamPath);

  WiFiClient client;
  client.setTimeout(2500);
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
      delay(30);
      fb = esp_camera_fb_get();
      if (!fb && esFinal) {
        delay(50);
        fb = esp_camera_fb_get();
      }
    }
    if (!fb) {
      Serial.printf("[WARN] Frame %d/%d falló en captura.\n", idx, TOTAL_FOTOS_RAFAGA);
      delay(20);
      continue;
    }

    if (!client.connected()) {
      if (!client.connect(serverHost, serverPort, 1500)) {
        Serial.printf("[ERROR] Frame %d: no se pudo conectar a %s:%u\n", idx, serverHost, serverPort);
        esp_camera_fb_return(fb);
        delay(40);
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
    unsigned long writeStart = millis();
    while (sent < fb->len) {
      size_t toSend = fb->len - sent;
      if (toSend > chunkSize) toSend = chunkSize;
      size_t written = client.write(fb->buf + sent, toSend);
      if (written == 0) {
        if (!client.connected() || (millis() - writeStart > 2500)) {
          break;
        }
        delay(2);
      } else {
        writeStart = millis();
        sent += written;
      }
    }

    bool envioCompleto = (sent == fb->len);
    if (envioCompleto) {
      client.print(bodyTail);
    }
    esp_camera_fb_return(fb);

    if (!envioCompleto) {
      Serial.printf("[WARN] Frame %d/%d envío incompleto (%u bytes), reiniciando socket.\n",
                    idx, TOTAL_FOTOS_RAFAGA, (unsigned int)sent);
      client.stop();
      continue;
    }

    unsigned long timeoutStart = millis();
    unsigned long maxWait = esFinal ? 10000 : 2500;
    while (client.connected() && !client.available() && (millis() - timeoutStart < maxWait)) {
      delay(2);
    }

    int contentLength = -1;
    bool serverClose = false;
    String statusLine = "";
    String respBody = "";
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
        while (readBytes < contentLength && (client.connected() || client.available()) && (millis() - bodyStart < 2500)) {
          if (client.available()) {
            char c = (char)client.read();
            readBytes++;
            if (esFinal && respBody.length() < 512) {
              respBody += c;
            }
          } else {
            delay(1);
          }
        }
      } else {
        while (client.available()) {
          char c = (char)client.read();
          if (esFinal && respBody.length() < 512) {
            respBody += c;
          }
        }
      }

      if (esFinal) {
        long tid = extraerEnteroJson(respBody, "trigger_id", -1);
        if (tid > ultimoTriggerIdRemoto) {
          ultimoTriggerIdRemoto = tid;
        }
      }
      fotosEnviadas++;
    } else {
      Serial.printf("[WARN] Frame %d/%d sin respuesta HTTP, reiniciando socket.\n", idx, TOTAL_FOTOS_RAFAGA);
      client.stop();
    }

    if (esFinal || serverClose) {
      client.stop();
    }
    Serial.printf("[CAM] Frame %d/%d -> %s\n", idx, TOTAL_FOTOS_RAFAGA, statusLine.c_str());
  }

  if (client.connected()) client.stop();
  ultimaRafagaMs = millis();
  sincronizarTriggerServidor();
  pinAnterior = (digitalRead(TRIGGER_PIN) == HIGH);
  Serial.printf("[ÉXITO] Ráfaga completada: %d/%d fotos enviadas.\n", fotosEnviadas, TOTAL_FOTOS_RAFAGA);
}

void revisarTriggerRemoto() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastPollRetryMs > 4000) {
      lastPollRetryMs = millis();
      asegurarWiFi();
      if (wifiConectado && ultimoTriggerIdRemoto < 0) {
        sincronizarTriggerServidor();
      }
    }
    return;
  }

  if (ultimoTriggerIdRemoto < 0) {
    sincronizarTriggerServidor();
    if (ultimoTriggerIdRemoto < 0) ultimoTriggerIdRemoto = 0;
  }

  if (!pollRequestActive) {
    if (millis() - lastPollRetryMs < 300) return;
    lastPollRetryMs = millis();

    if (!pollClient.connect(serverHost, serverPort, 500)) {
      lastPollRetryMs = millis() + 1500;
      return;
    }

    pollClient.printf("GET %s?ultimo_id=%ld&timeout=8 HTTP/1.1\r\n",
                      serverCamWaitPath, ultimoTriggerIdRemoto);
    pollClient.printf("Host: %s:%u\r\n", serverHost, serverPort);
    pollClient.print("Connection: close\r\n\r\n");

    pollRequestActive = true;
    pollStartedMs = millis();
    return;
  }

  if (pollClient.available()) {
    String body = leerRespuestaCortaHTTP(pollClient, 1000);
    pollClient.stop();
    pollRequestActive = false;

    bool trig = extraerBoolJson(body, "trigger");
    long tid = extraerEnteroJson(body, "trigger_id", ultimoTriggerIdRemoto);
    if (trig && tid > ultimoTriggerIdRemoto) {
      ultimoTriggerIdRemoto = tid;
      if (millis() - ultimaRafagaMs > 1500) {
        Serial.printf("\n[S3-CAM] ¡Disparo remoto recibido del servidor (trigger_id=%ld)! Iniciando ráfaga...\n", tid);
        enviarRafagaFotos();
        Serial.println("[S3-CAM] Ráfaga terminada. Esperando próximo disparo...");
      }
    }
  } else if (!pollClient.connected() || (millis() - pollStartedMs > 11000)) {
    pollClient.stop();
    pollRequestActive = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);

  Serial.println("\n[S3-CAM] Iniciando módulo de cámara...");
  inicializarCamara();
  asegurarWiFi();
  if (wifiConectado) {
    sincronizarTriggerServidor();
  }

  Serial.printf("[S3-CAM] Listo. Esperando pulso HIGH en GPIO %d o disparo Wi-Fi...\n", (int)TRIGGER_PIN);
}

void loop() {
  if (tArranque == 0) {
    tArranque = millis();
    Serial.println("[S3-CAM] Periodo de estabilizacion de boot activo...");
  }

  if (!listo) {
    if (millis() - tArranque >= BOOT_IGNORE_MS) {
      listo = true;
      pinAnterior = (digitalRead(TRIGGER_PIN) == HIGH);
      Serial.printf("[S3-CAM] Listo. Escuchando flanco en GPIO %d y /cam-esperar...\n", (int)TRIGGER_PIN);
    } else {
      delay(10);
      return;
    }
  }

  // 1. Detección de flanco de subida hardware (LOW -> HIGH) con confirmación breve
  bool pinActual = (digitalRead(TRIGGER_PIN) == HIGH);
  if (pinActual && !pinAnterior) {
    delay(15);
    if (digitalRead(TRIGGER_PIN) == HIGH && (millis() - ultimaRafagaMs > 1500)) {
      Serial.printf("\n[S3-CAM] ¡Flanco de subida detectado en GPIO %d! Iniciando ráfaga...\n", (int)TRIGGER_PIN);
      enviarRafagaFotos();
      Serial.println("[S3-CAM] Ráfaga terminada. Esperando próximo disparo...");
      delay(5);
      return;
    }
  }
  pinAnterior = pinActual;

  // 2. Respaldo por red: disparo coordinado vía servidor FastAPI (/cam-esperar)
  revisarTriggerRemoto();

  delay(5);
}
