# Pi Camera Capture
import time
from picamera2 import Picamera2, Preview
from pyzbar import pyzbar
import numpy as np

# Decode QR code
from PIL import Image
from pyzbar.pyzbar import decode

# Send UDP
import socket

UDP_IP = "192.168.1.25"
UDP_PORT = 8080

def find_qr_code(image):
    """Detect QR codes in the given image."""
    decoded_objects = pyzbar.decode(image)
    return decoded_objects

def capture_image_when_qr_detected(picam2):
    while True:
        # Capture an image to memory (numpy array)
        image = picam2.capture_array()
        if find_qr_code(image):
            # QR code detected, save the image
            metadata = picam2.capture_file("QR.jpg")
            print("QR Code detected, image saved.")
            print(metadata)
            break
        else:
            print("No QR code detected. Trying again...")
        time.sleep(2)  # Add a delay between captures


while(1):
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(main={"size": (800, 600)})
    picam2.configure(preview_config)

    # picam2.start_preview(Preview.QTGL)
    picam2.start()

    try:
        capture_image_when_qr_detected(picam2)
    finally:
        picam2.close()

    image_path = 'QR.jpg'  
    img = Image.open(image_path)

    data = decode(img)

    QR_decoded = ""

    # Check if QR code exists
    if data:
        QR_decoded = data[0].data.decode('utf-8')
        # print(data[0].data.decode('utf-8'))
    else:
        print("No QR code found")
        
    time.sleep(0.25)

    # MESSAGE = bytes(QR_decoded + "," + "0", 'utf-8')
    MESSAGE = bytes(QR_decoded, 'utf-8')

    print(MESSAGE)

    # Send payload to server
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    time.sleep(4)  # Add a delay between captures