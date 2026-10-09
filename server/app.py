import os
import requests
import base64
from dotenv import load_dotenv

# Cargar la API Key desde el archivo .env
load_dotenv()
api_key = os.getenv("GEMINI_API_KEY")

# Dirección endpoint con el modelo actualizado gemini-3.8-flash
url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-3.8-flash:generateContent?key={api_key}"

headers = {
    "Content-Type": "application/json"
}

# Leer y convertir la imagen a formato base64
with open("bombillo.jpg", "rb") as image_file:
    imagen_base64 = base64.b64encode(image_file.read()).decode('utf-8')

# Nuevo paquete de datos que incluye texto y la fotografía
payload = {
    "contents": [
        {
            "parts": [
                {"text": "Responde a la siguiente pregunta: El objeto en la imagen es un bombillo incandescente?"},
                {
                    "inlineData": {
                        "mimeType": "image/jpg",
                        "data": imagen_base64
                    }
                }
            ]
        }
    ]
}

# Realizar la petición HTTP POST
response = requests.post(url, headers=headers, json=payload)

if response.status_code == 200:
    data = response.json()
    respuesta_texto = data['candidates'][0]['content']['parts'][0]['text']
    print("Respuesta de Gemini:")
    print(respuesta_texto)
else:
    print(f"Error {response.status_code}:")
    print(response.text)