# Identity-Safe-Deposit: Facial Recognition with ESP32, Flask, and STM32

## 🧠 Description

This project implements a facial recognition system using an ESP32 camera, a Flask server for processing and web interface, and an STM32 module for hardware interaction. Images are sent via HTTP and compared with a gallery of known faces.

## ⚙️ Configuration

- **Web dashboard**: http://192.168.28.111:5000/dashboard

- **ESP32 Wi-Fi**:

  - SSID: Gianlu

  - Password: 12345678

- **Server address set on the ESP32 board**: 192.168.28.111:5000

- **Flask server running on**: 192.168.28.111:5000

---

## 📁 Project Structure

esp32/

└── LASTsketch/ # Script for ESP32

server_flask/

├── know_face/ # Images of known faces

├── received_images/ # Images received from ESP32 via POST

├── templates/ # HTML templates for the web interface

├── server.py # Main Flask server script

└── README.txt # This file

stm32/

└── progettostm32/ # Code for STM32

---

## 📂 Folder Description

- **know_face/**

  Place images of known faces here.

  Each image must be renamed with the subject's name, so the name will be displayed once recognized.

- **received_images/**

  Folder where images received via POST request from the ESP32 camera are saved.

- **LASTsketch/**

  Contains the sketch for programming the ESP32.

- **templates/**

  Contains the HTML files used by the Flask server for rendering web pages.

- **server.py**

  Main script for the Flask server. Manages requests, image processing, and the web interface.

## 🧩 Dependencies

To run the Flask server, install the following libraries:

- flask

- opencv-python

- face-recognition

- numpy

You can install them all with:

pip install -r requirements.txt

## 👤 Authors

Fontanella Gianluca

Iovine Giovanna

Grandioso Nicola

Esposito Fabio
