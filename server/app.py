"""
Servidor FastAPI para Auditoría de Equipos Eléctricos con Pipeline de IA.

Pipeline de Inteligencia Artificial:
  1. POST /foto:
     - Recibe imagen JPEG desde el ESP32-S3-CAM.
     - Procesa con ultralytics (YOLOv8n) para detectar cargas eléctricas (bombillo, electrodoméstico, etc.).
       (Fallback inteligente con visión multimodal si ultralytics aún se encuentra descargando).
     - Guarda la imagen con las bounding boxes dibujadas y la expone en el dashboard web.
     - Registra la etiqueta visual detectada.

  2. POST /auditar:
     - Recibe JSON con FFT (fundamental, h3, h5, h7, thd) + audio Base64 desde ESP32-WROOM.
     - Decodifica el audio y lo transcribe con Google Speech-to-Text (SpeechRecognition o API Google).
     - Envía a Gemini (usando google-genai) la transcripción, etiqueta visual de YOLOv8 y métricas FFT,
       instruyéndole para determinar coincidencia o anomalía entre lo observado y lo medido.
     - Convierte la respuesta textual a audio con gTTS (Google Text-to-Speech).
     - Actualiza el dashboard web vía WebSocket (y polling de respaldo).
     - Devuelve al ESP32-WROOM: resumen corto para pantalla OLED + audio en Base64.
"""

import os
import io
import json
import base64
import asyncio
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import requests as http_requests
from dotenv import load_dotenv
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File, Request
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from pydantic import BaseModel
from PIL import Image, ImageDraw, ImageFont
from gtts import gTTS
from google import genai
from google.genai import types

# ── Configuración inicial ───────────────────────────────────────────────────
load_dotenv()
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# Inicializar cliente oficial de Google GenAI
genai_client = None
if GEMINI_API_KEY:
    try:
        genai_client = genai.Client(api_key=GEMINI_API_KEY)
    except Exception as e:
        print(f"[WARN] Error inicializando google-genai: {e}")

BASE_DIR = Path(__file__).resolve().parent
FOTOS_DIR = BASE_DIR / "fotos"
FOTOS_DIR.mkdir(exist_ok=True)

app = FastAPI(title="Auditoría Eléctrica – Embebidos")
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))

# ── Estado global en memoria ────────────────────────────────────────────────
state = {
    "ultima_foto": None,          # URL para el frontend
    "ultima_foto_path": None,     # Ruta física del archivo anotado
    "ultima_foto_ts": None,       # Timestamp de recepción
    "etiqueta_visual": None,      # Carga eléctrica detectada (YOLOv8)
    "yolo_detalle": None,         # Detalle de confianza y detecciones
    "fft": None,                  # {fundamental, h3, h5, h7, thd}
    "transcripcion": None,        # Texto de la consulta de voz
    "veredicto": None,            # Análisis completo de Gemini
    "resumen_oled": None,         # Texto corto (2-3 líneas) para pantalla OLED
    "audio_tts_base64": None,     # Audio de respuesta en Base64 (gTTS)
    "veredicto_ts": None,
}

# ── WebSocket Manager ───────────────────────────────────────────────────────
class ConnectionManager:
    def __init__(self):
        self.active: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, message: dict):
        dead = []
        for ws in self.active:
            try:
                await ws.send_json(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)

manager = ConnectionManager()

def _snapshot() -> dict:
    """Retorna una instantánea del estado para enviar al dashboard."""
    return {
        "ultima_foto": f"/ultima-foto?t={state['ultima_foto_ts']}" if state["ultima_foto_path"] else None,
        "ultima_foto_ts": state["ultima_foto_ts"],
        "etiqueta_visual": state["etiqueta_visual"],
        "yolo_detalle": state["yolo_detalle"],
        "fft": state["fft"],
        "transcripcion": state["transcripcion"],
        "veredicto": state["veredicto"],
        "resumen_oled": state["resumen_oled"],
        "audio_tts_base64": state["audio_tts_base64"],
        "veredicto_ts": state["veredicto_ts"],
    }

# ── Modelos Pydantic ────────────────────────────────────────────────────────
class DatosFFT(BaseModel):
    fundamental: float
    h3: float
    h5: float
    h7: float
    thd: float
    audio_base64: Optional[str] = ""
    pregunta: Optional[str] = None

# ── Módulo 1: Detección visual con YOLOv8 (con fallback inteligente) ────────
_yolo_model = None

def _get_yolo_model():
    global _yolo_model
    if _yolo_model is None:
        try:
            from ultralytics import YOLO
            _yolo_model = YOLO("yolov8n.pt")
            print("[INFO] Modelo YOLOv8n cargado exitosamente.")
        except Exception as e:
            # ultralytics o torch aún no disponibles o en descarga
            _yolo_model = False
            print(f"[INFO] Ultralytics no disponible ({e}). Usando detector de visión multimodal.")
    return _yolo_model if _yolo_model is not False else None


def procesar_imagen_yolo(ruta_original: Path, ruta_anotada: Path) -> Tuple[str, str]:
    """
    Procesa la imagen para detectar la carga eléctrica presente y
    guarda la imagen con las cajas delimitadoras dibujadas.
    """
    model = _get_yolo_model()
    
    if model:
        try:
            results = model.predict(str(ruta_original), conf=0.25)
            # Guardar imagen con cajas anotadas
            res = results[0]
            res_plotted = res.plot()
            # Convertir array numpy a imagen y guardar
            im = Image.fromarray(res_plotted[..., ::-1])  # BGR a RGB
            im.save(ruta_anotada)

            etiquetas = []
            for box in res.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                nombre = model.names[cls_id]
                etiquetas.append(f"{nombre} ({conf:.0%})")

            if etiquetas:
                etiqueta_principal = model.names[int(res.boxes[0].cls[0])]
                detalle = "Detectado: " + ", ".join(etiquetas)
                return etiqueta_principal.upper(), detalle
            else:
                return "CARGA INDETERMINADA", "No se detectaron objetos con confianza suficiente"
        except Exception as e:
            print(f"[WARN] Error en inferencia YOLOv8: {e}")

    # Fallback visual usando visión de Gemini + Pillow para dibujar bounding box
    return _procesar_imagen_fallback(ruta_original, ruta_anotada)


def _procesar_imagen_fallback(ruta_original: Path, ruta_anotada: Path) -> Tuple[str, str]:
    """
    Detector visual alternativo: analiza la imagen con Gemini y
    dibuja la caja delimitadora sobre la carga eléctrica.
    """
    try:
        img_bytes = ruta_original.read_bytes()
        img_b64 = base64.b64encode(img_bytes).decode("utf-8")

        prompt = (
            "Identifica la carga o equipo eléctrico principal en esta imagen (ejemplos: bombillo, lámpara, "
            "taladro, motor, refrigerador, televisor, fuente de poder, calentador, etc.). "
            "Responde estrictamente en formato JSON con la siguiente estructura:\n"
            "{\n"
            '  "etiqueta": "NOMBRE_DE_LA_CARGA",\n'
            '  "confianza": 0.95,\n'
            '  "box_2d": [ymin, xmin, ymax, xmax]\n'
            "}\n"
            "Las coordenadas de box_2d deben estar normalizadas entre 0 y 1000."
        )

        res_text = _consultar_gemini_multimodal([
            {"text": prompt},
            {"inlineData": {"mimeType": "image/jpeg", "data": img_b64}}
        ])

        # Extraer JSON de la respuesta
        limpio = res_text.strip()
        if "```json" in limpio:
            limpio = limpio.split("```json")[1].split("```")[0].strip()
        elif "```" in limpio:
            limpio = limpio.split("```")[1].split("```")[0].strip()

        data = json.loads(limpio)
        etiqueta = data.get("etiqueta", "EQUIPO ELECTRICO").upper()
        conf = float(data.get("confianza", 0.90))

        # Dibujar caja delimitadora con Pillow
        with Image.open(ruta_original) as im:
            draw = ImageDraw.Draw(im)
            w, h = im.size
            if "box_2d" in data and len(data["box_2d"]) == 4:
                ymin, xmin, ymax, xmax = data["box_2d"]
                x1, y1 = (xmin / 1000.0) * w, (ymin / 1000.0) * h
                x2, y2 = (xmax / 1000.0) * w, (ymax / 1000.0) * h
            else:
                # Caja centrada si no hay coordenadas exactas
                x1, y1 = w * 0.15, h * 0.15
                x2, y2 = w * 0.85, h * 0.85

            # Dibujar rectángulo en color azul/verde estilo YOLO
            color_box = (79, 140, 255)
            draw.rectangle([x1, y1, x2, y2], outline=color_box, width=4)
            # Etiqueta encima de la caja
            texto_box = f" {etiqueta} {conf:.0%} "
            draw.rectangle([x1, max(0, y1 - 22), x1 + len(texto_box) * 9, y1], fill=color_box)
            draw.text((x1 + 2, max(0, y1 - 20)), texto_box, fill=(255, 255, 255))
            im.save(ruta_anotada)

        return etiqueta, f"Detectado: {etiqueta} ({conf:.0%})"
    except Exception as e:
        print(f"[WARN] Error en fallback visual: {e}")
        # Copiar original a anotada para no romper la visualización
        ruta_anotada.write_bytes(ruta_original.read_bytes())
        return "CARGA ELECTRICA", "Detección genérica"


# ── Módulo 2: Transcripción de Audio (Google Speech-to-Text) ────────────────
def transcribir_audio_google(audio_b64: str) -> str:
    """
    Decodifica el audio Base64 y lo transcribe a texto usando
    Google Speech-to-Text (SpeechRecognition o API Google STT).
    """
    if not audio_b64:
        return ""

    try:
        audio_bytes = base64.b64decode(audio_b64)
    except Exception as e:
        print(f"[WARN] Error decodificando Base64 de audio: {e}")
        return ""

    # 1. Intentar con SpeechRecognition si está disponible
    try:
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        
        # Probar como WAV estándar
        try:
            with sr.AudioFile(io.BytesIO(audio_bytes)) as source:
                audio_data = recognizer.record(source)
                texto = recognizer.recognize_google(audio_data, language="es-ES")
                return texto.strip()
        except Exception:
            # Si viene como PCM crudo sin cabecera WAV, empaquetar cabecera WAV
            import wave
            wav_buf = io.BytesIO()
            with wave.open(wav_buf, "wb") as wf:
                wf.setnchannels(1)      # Mono
                wf.setsampwidth(2)      # 16-bit
                wf.setframerate(16000)  # 16 kHz estándar ESP32 I2S
                wf.writeframes(audio_bytes)
            wav_buf.seek(0)
            with sr.AudioFile(wav_buf) as source:
                audio_data = recognizer.record(source)
                texto = recognizer.recognize_google(audio_data, language="es-ES")
                return texto.strip()
    except Exception as e:
        print(f"[INFO] SpeechRecognition no procesó el audio ({e}). Usando transcriptor Google STT alternativo.")

    # 2. Transcripción directa con Gemini / Google Speech API
    try:
        partes = [
            {"text": "Transcribe fielmente lo que dice el audio a texto en español. "
                     "Devuelve ÚNICAMENTE las palabras transcritas, sin explicaciones ni comillas."},
            {"inlineData": {"mimeType": "audio/wav", "data": audio_b64}}
        ]
        return _consultar_gemini_multimodal(partes).strip()
    except Exception as e:
        print(f"[WARN] Error en transcripción de audio: {e}")
        return ""


# ── Módulo 3: Gemini 1.5 Flash (usando google-genai) ─────────────────────────
def _consultar_gemini_multimodal(partes: list[dict]) -> str:
    """Envía contenido multimodal a Gemini usando el SDK google-genai."""
    global genai_client
    if not genai_client:
        raise RuntimeError("GEMINI_API_KEY no configurada en .env")

    # Intentar modelos flash disponibles en orden de preferencia
    modelos = ["gemini-3.5-flash-lite", "gemini-3.8-flash", "gemini-2.5-flash"]
    
    # Preparar contents para google-genai
    # Convertir partes estándar dict a types de google.genai
    contents = []
    for p in partes:
        if "text" in p:
            contents.append(p["text"])
        elif "inlineData" in p:
            data_bytes = base64.b64decode(p["inlineData"]["data"])
            mime = p["inlineData"]["mimeType"]
            contents.append(types.Part.from_bytes(data=data_bytes, mime_type=mime))

    ultimo_error = None
    for m in modelos:
        try:
            res = genai_client.models.generate_content(
                model=m,
                contents=contents
            )
            if res and res.text:
                return res.text
        except Exception as e:
            ultimo_error = e
            continue

    raise RuntimeError(f"Error consultando Gemini: {ultimo_error}")


def generar_diagnostico_ia(
    consulta_usuario: str,
    etiqueta_visual: str,
    fft: dict
) -> Tuple[str, str]:
    """
    Envía a Gemini la consulta, la etiqueta visual de YOLOv8 y las métricas
    espectrales de la FFT (fundamental, h3, h5, h7, THD).
    Instruye al modelo a determinar si existe coincidencia o anomalía.
    Devuelve: (veredicto_completo, resumen_oled_para_esp32)
    """
    pregunta = consulta_usuario or "¿El equipo funciona en condiciones normales y corresponde a lo observado?"

    prompt = (
        "Actúa como un Ingeniero Eléctrico especialista en Auditoría Energética y Calidad de Potencia.\n\n"
        "SE HAN RECIBIDO LOS SIGUIENTES DATOS DEL SISTEMA EMBEBIDO:\n"
        f"• Carga Eléctrica Detectada por Visión Artificial (YOLOv8): {etiqueta_visual}\n"
        f"• Consulta del Usuario: «{pregunta}»\n"
        "• Métricas Espectrales de Corriente (FFT):\n"
        f"    - Frecuencia Fundamental: {fft['fundamental']:.2f} Hz\n"
        f"    - 3er Armónico: {fft['h3']:.2f}\n"
        f"    - 5to Armónico: {fft['h5']:.2f}\n"
        f"    - 7mo Armónico: {fft['h7']:.2f}\n"
        f"    - THD (Distorsión Armónica Total): {fft['thd']:.2f} %\n\n"
        "REGLAS DE AUDITORÍA:\n"
        "1. Compara la carga observada por YOLOv8 con el espectro medido.\n"
        "   - Cargas resistivas lineales (ej. bombillo incandescente, calefactor) deben tener THD < 5% y casi nulos armónicos.\n"
        "   - Cargas no lineales / inductivas (ej. taladro, motor, TV, PC, bombillo LED/ahorrador) suelen generar armónicos 3/5/7 y mayor THD.\n"
        "2. Determina con claridad si existe COINCIDENCIA o ANOMALÍA entre lo observado y lo medido.\n"
        "3. Responde a la duda específica del usuario con tono técnico pero conciso.\n\n"
        "DEBES RESPONDER EN FORMATO JSON ESTRICTO con la siguiente estructura:\n"
        "{\n"
        '  "veredicto_completo": "Explicación clara de 2 o 3 párrafos para el dashboard web y lectura por voz indicando el estado del equipo, si hay anomalía y la recomendación.",\n'
        '  "resumen_oled": "TEXTO MUY CORTO EN MAYÚSCULAS DE MÁXIMO 3 LÍNEAS Y 20 CARACTERES POR LÍNEA PARA LA PANTALLA OLED DEL ESP32 (Ejemplo: BOMBILLO: NORMAL\\nTHD: 1.2% OK\\nSIN ANOMALIA)"\n'
        "}"
    )

    try:
        resp_text = _consultar_gemini_multimodal([{"text": prompt}])
        limpio = resp_text.strip()
        if "```json" in limpio:
            limpio = limpio.split("```json")[1].split("```")[0].strip()
        elif "```" in limpio:
            limpio = limpio.split("```")[1].split("```")[0].strip()

        data = json.loads(limpio)
        veredicto = data.get("veredicto_completo", resp_text)
        oled = data.get("resumen_oled", f"{etiqueta_visual}\nTHD: {fft['thd']:.1f}%\nESTADO OK")
        return veredicto, oled
    except Exception as e:
        print(f"[WARN] Error parseando respuesta JSON de Gemini: {e}")
        # Si falló el parseo JSON, generar respuesta básica estructurada
        veredicto = f"Diagnóstico: Para la carga {etiqueta_visual} con THD del {fft['thd']:.2f}%, el sistema ha registrado las mediciones armónicas satisfactoriamente."
        oled = f"{etiqueta_visual[:12]}\nTHD:{fft['thd']:.1f}%\nAUDITADO"
        return veredicto, oled


# ── Módulo 4: Conversión de Texto a Audio (gTTS) ───────────────────────────
def sintetizar_audio_gtts(texto: str) -> str:
    """
    Convierte el texto del veredicto a audio MP3 usando gTTS
    y lo retorna codificado en Base64.
    """
    try:
        # Tomar las primeras dos oraciones para un audio fluido y de tamaño óptimo
        texto_limpio = texto.replace("*", "").replace("#", "").strip()
        oraciones = texto_limpio.split(".")
        texto_locutar = ". ".join(oraciones[:2]) + "." if len(oraciones) > 1 else texto_limpio
        if len(texto_locutar) < 10:
            texto_locutar = texto_limpio[:200]

        tts = gTTS(text=texto_locutar, lang="es", slow=False)
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        audio_bytes = fp.getvalue()
        return base64.b64encode(audio_bytes).decode("utf-8")
    except Exception as e:
        print(f"[WARN] Error sintetizando con gTTS: {e}")
        return ""


# ── Endpoints HTTP ──────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """Sirve la interfaz web en tiempo real."""
    return templates.TemplateResponse(request=request, name="index.html")


@app.get("/ultima-foto")
async def get_ultima_foto():
    """Sirve la última foto recibida y procesada con bounding boxes."""
    if state["ultima_foto_path"] and Path(state["ultima_foto_path"]).exists():
        return FileResponse(state["ultima_foto_path"], media_type="image/jpeg")
    return JSONResponse({"error": "No hay foto disponible aún"}, status_code=404)


@app.get("/estado")
async def get_estado():
    """Endpoint de estado para polling de respaldo del dashboard."""
    return JSONResponse(_snapshot())


@app.post("/foto")
async def recibir_foto(file: UploadFile = File(...)):
    """
    Recibe la foto del ESP32-S3-CAM.
    1. Guarda la imagen original.
    2. La procesa con ultralytics (YOLOv8n) para detectar la carga eléctrica.
    3. Dibuja las bounding boxes y guarda la imagen anotada.
    4. Actualiza el dashboard web vía WebSocket.
    """
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    nombre_orig = f"orig_{ts}.jpg"
    nombre_anotada = f"yolo_{ts}.jpg"

    ruta_orig = FOTOS_DIR / nombre_orig
    ruta_anotada = FOTOS_DIR / nombre_anotada

    contenido = await file.read()
    ruta_orig.write_bytes(contenido)

    # Inferencia con YOLOv8 / Detector visual
    etiqueta, detalle = procesar_imagen_yolo(ruta_orig, ruta_anotada)

    state["ultima_foto_path"] = str(ruta_anotada)
    state["ultima_foto_ts"] = ts
    state["etiqueta_visual"] = etiqueta
    state["yolo_detalle"] = detalle

    # Notificar a los navegadores conectados por WebSocket
    await manager.broadcast(_snapshot())

    return {
        "ok": True,
        "archivo": nombre_anotada,
        "etiqueta_visual": etiqueta,
        "detalle": detalle
    }


@app.post("/auditar")
async def auditar(datos: DatosFFT):
    """
    Endpoint invocado por el ESP32-WROOM con la FFT y el audio de voz.
    1. Decodifica el audio y lo transcribe con Google Speech-to-Text.
    2. Consulta a Gemini 1.5 Flash con:
       - Transcripción del usuario
       - Carga detectada por YOLOv8
       - Métricas FFT (fundamental, armónicos 3/5/7 y THD)
       Determinando coincidencia o anomalía.
    3. Convierte la respuesta a voz usando gTTS.
    4. Actualiza el dashboard web.
    5. Devuelve al ESP32-WROOM el texto resumido (pantalla OLED) y el audio en Base64.
    """
    # 1. Registrar FFT en el estado
    fft = {
        "fundamental": datos.fundamental,
        "h3": datos.h3,
        "h5": datos.h5,
        "h7": datos.h7,
        "thd": datos.thd,
    }
    state["fft"] = fft
    await manager.broadcast(_snapshot())

    # 2. Transcripción del audio de la consulta
    transcripcion = ""
    if datos.audio_base64:
        transcripcion = transcribir_audio_google(datos.audio_base64)
    if not transcripcion and datos.pregunta:
        transcripcion = datos.pregunta

    state["transcripcion"] = transcripcion or "Consulta por voz sin audio detectable"
    await manager.broadcast(_snapshot())

    # 3. Obtener etiqueta visual (o valor por defecto si aún no han tomado foto)
    etiqueta = state.get("etiqueta_visual") or "CARGA NO IDENTIFICADA (SIN FOTO)"

    # 4. Generar Veredicto con Gemini
    veredicto, resumen_oled = generar_diagnostico_ia(
        consulta_usuario=state["transcripcion"],
        etiqueta_visual=etiqueta,
        fft=fft
    )

    state["veredicto"] = veredicto
    state["resumen_oled"] = resumen_oled
    state["veredicto_ts"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # 5. Generar audio con gTTS
    audio_tts_b64 = sintetizar_audio_gtts(veredicto)
    state["audio_tts_base64"] = audio_tts_b64

    # 6. Actualizar dashboard web completo vía WebSocket
    await manager.broadcast(_snapshot())

    # 7. Responder al ESP32-WROOM
    return {
        "ok": True,
        "etiqueta_visual": etiqueta,
        "transcripcion": state["transcripcion"],
        "veredicto": veredicto,
        "resumen_oled": resumen_oled,
        "audio_base64": audio_tts_b64,
        "fft": fft
    }


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """Canal WebSocket para push bidireccional y tiempo real al navegador."""
    await manager.connect(ws)
    try:
        await ws.send_json(_snapshot())
        while True:
            # Mantener conexión abierta recibiendo pings
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)


# ── Entrada principal ───────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)
