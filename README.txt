# Progetto: Riconoscimento Facciale con ESP32, Flask e STM32

## 🧠 Descrizione

Questo progetto realizza un sistema di riconoscimento facciale usando una telecamera ESP32, un server Flask per l'elaborazione e interfaccia web, e un modulo STM32 per l'interazione hardware. Le immagini vengono inviate via HTTP e confrontate con una galleria di volti noti.

## ⚙️ Configurazione

- **Dashboard web**: http://192.168.28.111:5000/dashboard
- **Wi-Fi ESP32**:
  - SSID: Gianlu
  - Password: 12345678
- **Indirizzo server impostato sulla scheda ESP32**: 192.168.28.111:5000
- **Server Flask in esecuzione su**: 192.168.28.111:5000

---

## 📁 Struttura del progetto

esp32/
└── LASTsketch/ # Script per ESP32

server_flask/
├── know_face/ # Immagini dei volti noti
├── received_images/ # Immagini ricevute dalla ESP32 via POST
├── templates/ # Template HTML per l'interfaccia web
├── server.py # Server Flask principale
└── README.txt # Questo file

stm32/
└── progettostm32/ # Codice per STM32

---

## 📂 Descrizione cartelle

- **know_face/**
  Inserire qui le immagini dei volti noti.
  Ogni immagine deve essere rinominata con il nome del soggetto, così il nome verrà mostrato una volta riconosciuto.

- **received_images/**
  Cartella in cui vengono salvate le immagini ricevute tramite richiesta POST dalla telecamera ESP32.

- **LASTsketch/**
  Contiene lo sketch per la programmazione dell'ESP32.

- **templates/**
  Contiene i file HTML usati dal server Flask per il rendering delle pagine web.

- **server.py**
  Script principale del server Flask. Gestisce le richieste, l’elaborazione delle immagini e l’interfaccia web.

## 🧩 Dipendenze

Per far funzionare il server Flask, installare le seguenti librerie:

- flask
- opencv-python
- face-recognition
- numpy

Puoi installarle tutte con:

pip install -r requirements.txt

## 👤 Autori
Fontanella Gianluca
Iovine Giovanna
Grandioso Nicola
Esposito Fabio




