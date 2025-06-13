from flask import Flask, request, jsonify
from flask import send_from_directory, render_template


import face_recognition
import os
import io
from datetime import datetime
from PIL import Image

app = Flask(__name__)

#MEMORIA TEMPORANEA PER TENERE TRACCIA DEGLI ACCESSI NELLA SESSIONE CORRENTE
access_log = []  # Lista di dizionari: [{'filename': ..., 'name': ...}]
UPLOAD_FOLDER = "received_images"
KNOWN_FOLDER = "known_face"

os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Carica volti noti all'avvio
known_face_encodings = []
known_face_names = []

def load_known_faces():
    for filename in os.listdir(KNOWN_FOLDER):
        if filename.lower().endswith(('.jpg', '.jpeg', '.png')):
            path = os.path.join(KNOWN_FOLDER, filename)
            image = face_recognition.load_image_file(path)
            encodings = face_recognition.face_encodings(image)
            if encodings:
                known_face_encodings.append(encodings[0])
                known_face_names.append(os.path.splitext(filename)[0])
    print(f"[INFO] {len(known_face_encodings)} known faces loaded.")

load_known_faces()

def recognize_face(image_bytes):
    # Carica immagine ricevuta
    unknown_image = face_recognition.load_image_file(io.BytesIO(image_bytes))
    unknown_encodings = face_recognition.face_encodings(unknown_image)

    if not unknown_encodings:
        return False, "No face found"

    unknown_encoding = unknown_encodings[0]

    # Calcola la distanza tra volti
    face_distances = face_recognition.face_distance(known_face_encodings, unknown_encoding)
    best_match_index = face_distances.argmin()
    best_distance = face_distances[best_match_index]

    # Soglia più severa
    if best_distance < 0.4:
        return True, known_face_names[best_match_index]
    else:
        return False, "Unknown"


@app.route('/upload', methods=['POST'])
def upload_image():
    try:
        image_bytes = request.data
        if not image_bytes:
            return jsonify({'error': 'No data received'}), 400

        # Salva immagine ricevuta con timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(UPLOAD_FOLDER, f"image_{timestamp}.jpg")
        image = Image.open(io.BytesIO(image_bytes))
        image.save(image_path)

        # Riconoscimento facciale
        recognized, name_or_msg = recognize_face(image_bytes)
        result = {
            'timestamp': timestamp,
            'filename': f"image_{timestamp}.jpg",
            'recognized': recognized,
            'name': name_or_msg if recognized else "Unknown"
        }
        access_log.append(result)

        if recognized:
            return jsonify({'status': 'ok', 'name': name_or_msg}), 200
        else:
            return jsonify({'status': 'not ok', 'reason': name_or_msg}), 200

    except Exception as e:
        return jsonify({'error': 'File non valido o danneggiato', 'detail': str(e)}), 400
    

@app.route('/images/<filename>')
def get_image(filename):
    return send_from_directory(UPLOAD_FOLDER, filename)

@app.route('/dashboard')
def dashboard():
    return render_template("dashboard.html",log=access_log)


@app.route('/api/log')
def get_log():
    return jsonify(access_log)


if __name__ == '__main__':
    app.run(host='192.168.28.111', port=5000)
