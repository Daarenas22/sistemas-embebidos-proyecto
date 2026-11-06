#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <arduinoFFT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- PINES DE CONEXIÓN ---
#define BOTON_PIN 4        // Botón físico de FSM
#define TRIGGER_CAM_PIN 5  // Salida hacia el GPIO 2 (lado derecho) de la cámara
#define MIC_ANALOG_PIN 36  // Entradas analógicas / Micrófono (A0)

// --- CONFIGURACIÓN FFT ---
#define SAMPLES 128             // Cantidad de muestras (potencia de 2)
#define SAMPLING_FREQ 1000      // Frecuencia de muestreo Hz
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

double frecuenciaDominante = 0.0;
bool esIncandescente = false;

enum EstadoFSM {
  STANDBY,
  CAPTURA_VOZ,
  PROCESAMIENTO,
  MUESTREO_DUAL,
  RESULTADO
};

EstadoFSM estadoActual = STANDBY;
bool estadoBotonAnterior = HIGH;
unsigned long ultimoTiempoRebote = 0;
unsigned long retardoRebote = 50;

unsigned long ultimoTiempoOnda = 0;
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

void enviarDisparoCamara() {
  Serial.println("[WROOM] Enviando pulso de encendido a ESP32-S3-CAM...");
  digitalWrite(TRIGGER_CAM_PIN, HIGH);
  delay(200); // Pulso de despertador
  digitalWrite(TRIGGER_CAM_PIN, LOW);
}

void procesarFFT() {
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(MIC_ANALOG_PIN);
    vImag[i] = 0.0;
    delayMicroseconds(1000000 / SAMPLING_FREQ);
  }

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  frecuenciaDominante = FFT.majorPeak();
  
  // Criterio de clasificación (modificable según rango de tu señal)
  if (frecuenciaDominante >= 55.0 && frecuenciaDominante <= 65.0) {
    esIncandescente = true;
  } else {
    esIncandescente = false;
  }
}

void dibujarOndaSimulada() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MUESTREO AC (60Hz)");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  int amplitud = 18;       
  int centroY = 37;        
  float frecuencia = 0.15; 

  for(int x = 0; x < SCREEN_WIDTH; x++) {
    int y = centroY + (int)(amplitud * sin(frecuencia * x + faseOnda));
    display.drawPixel(x, y, SSD1306_WHITE);
  }
  
  faseOnda += 0.4; 
  display.display();
}

void actualizarPantalla() {
  if (estadoActual == MUESTREO_DUAL) {
    return; 
  }
  
  display.clearDisplay();
  
  if (estadoActual == STANDBY) {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(22, 2);
    display.println("STANDBY");
    display.drawBitmap(44, 22, epd_bitmap_spidy, 40, 40, SSD1306_WHITE);
    
  } else {
    // --- NUEVO ENCABEZADO OPTIMIZADO ---
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    switch (estadoActual) {
      case CAPTURA_VOZ:
        display.println("ESTADO: ESCUCHANDO");
        break;
      case PROCESAMIENTO:
        display.println("ESTADO: ANALIZANDO");
        break;
      case RESULTADO:
        display.println("ESTADO: DICTAMEN");
        break;
      default:
        display.println("PROYECTO EMBEBIDOS");
        break;
    }
    
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    
    // --- CUERPO DE DATOS ---
    display.setCursor(0, 18);
    
    switch (estadoActual) {
      case CAPTURA_VOZ:
        display.setTextSize(1);
        display.println("\nEsperando comando\nde voz...");
        break;

      case PROCESAMIENTO:
        display.setTextSize(1);
        display.println("Disparando S3-CAM...");
        display.println("Procesando FFT...");
        display.display();

        // Disparo físico a la cámara + Cálculo FFT
        enviarDisparoCamara();
        procesarFFT();
        break;

      case RESULTADO:
        display.setTextSize(1);
        display.print("Freq. Pico: ");
        display.print(frecuenciaDominante, 1);
        display.println(" Hz");
        
        display.setCursor(0, 34);
        display.setTextSize(2);
        if (esIncandescente) {
          display.println("INCANDESC.");
        } else {
          display.println("OTRA CARGA");
        }
        break;

      default:
        break;
    }
  }
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  
  pinMode(BOTON_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_CAM_PIN, OUTPUT);
  digitalWrite(TRIGGER_CAM_PIN, LOW);

  Wire.begin(21, 22);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error: No se encontró la pantalla OLED"));
    for(;;); 
  }
  
  actualizarPantalla();
}

void loop() {
  int lecturaBoton = digitalRead(BOTON_PIN);
  
  if (lecturaBoton != estadoBotonAnterior) {
    ultimoTiempoRebote = millis();
  }
  
  if ((millis() - ultimoTiempoRebote) > retardoRebote) {
    if (lecturaBoton == LOW) {
      estadoActual = (EstadoFSM)((estadoActual + 1) % 5);
      
      actualizarPantalla();
      
      while(digitalRead(BOTON_PIN) == LOW) {
        delay(10);
      }
    }
  }
  estadoBotonAnterior = lecturaBoton;

  if (estadoActual == MUESTREO_DUAL) {
    if (millis() - ultimoTiempoOnda > 50) { 
      dibujarOndaSimulada();
      ultimoTiempoOnda = millis();
    }
  }
}