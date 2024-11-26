import serial
import time
import requests
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from io import BytesIO
from roboflow import Roboflow
import supervision as sv
import math
import os

def get_ip_address(ser):
    """Wait for and receive the IP address from the STM32 over serial."""
    import re
    ip_pattern = re.compile(r'^(\d{1,3}\.){3}\d{1,3}$')
    ip_address = ''
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                if ip_pattern.match(line):
                    ip_address = line
                    break
    except serial.SerialException as e:
        print(f"Serial exception: {e}")
        
    return ip_address

def fetch_image(ip_address):
    url = f"http://{ip_address}/image.jpg"
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        return response.content
    except requests.RequestException as e:
        return None

def display_image(image_data):
    """Display the image using matplotlib."""
    image = Image.open(BytesIO(image_data))
    plt.ion()  # Enable interactive mode
    plt.imshow(image)
    plt.axis('off')  # Hide axis
    plt.show()
    plt.pause(0.001)
    plt.clf()  # Clear the figure for the next image
    
def cvstuff(image):
    # for balls
    # rfb = Roboflow(api_key="cFhwJYkoxK6oElI6NL9a")
    # projectb = rfb.workspace().project("billiard-balls-kjqyt")
    # modelb = projectb.version(9).model

    # for pockets
    rfp = Roboflow(api_key="cFhwJYkoxK6oElI6NL9a")
    projectp = rfp.workspace().project("pooltablepocket")
    modelp = projectp.version(1).model
    
    class_name_to_id = {"pocket": 0, "white": 1}
    
    image_path = "esp32_cam_image.jpg"
    image = cv2.imread(image_path)
    resized_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_path = f"temp.jpg"
    cv2.imwrite(image_path, cv2.cvtColor(resized_image, cv2.COLOR_RGB2BGR))

    # resultb = modelb.predict(image_path, confidence=0.2, overlap=0.5).json()
    resultp = modelp.predict(image_path, confidence=0.5, overlap=0.5).json()
    
    detections = []
    confidence_scores = []
    labels = []
    pocket_centers = []
    white_centers = []

    # for prediction in resultb["predictions"]:
    #     class_name = prediction["class"]
    #     if class_name != "white":
    #         continue  
    #     x, y, width, height = (
    #         prediction["x"],
    #         prediction["y"],
    #         prediction["width"],
    #         prediction["height"],
    #     )
    #     confidence = prediction["confidence"]

    #     x_min = int(x - width / 2)
    #     y_min = int(y - height / 2)
    #     x_max = int(x + width / 2)
    #     y_max = int(y + height / 2)

    #     detections.append([x_min, y_min, x_max, y_max])
    #     confidence_scores.append(confidence)
    #     labels.append(class_name)

    #     white_centers.append((x, y))

    for prediction in resultp["predictions"]:
        class_name = prediction["class"]
        if class_name != "pocket":
            continue  
        x, y, width, height = (
            prediction["x"],
            prediction["y"],
            prediction["width"],
            prediction["height"],
        )
        confidence = prediction["confidence"]

        x_min = int(x - width / 2)
        y_min = int(y - height / 2)
        x_max = int(x + width / 2)
        y_max = int(y + height / 2)

        detections.append([x_min, y_min, x_max, y_max])
        confidence_scores.append(confidence)
        labels.append(class_name)

        pocket_centers.append((x, y))

    if len(detections) == 0:
        print("No detections in image")

    detection_boxes = sv.Detections(
        xyxy=np.array(detections),
        confidence=np.array(confidence_scores)
    )
    detection_boxes.class_id = np.array([class_name_to_id[label] for label in labels])
    
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    annotated_image = image.copy()
    annotated_image = box_annotator.annotate(scene=annotated_image, detections=detection_boxes)
    annotated_image = label_annotator.annotate(scene=annotated_image, detections=detection_boxes, labels=labels)
    
    # Annotate lines and calculate angle
    if pocket_centers:
        # white_center = white_centers[0]
        white_center = ((image.shape[1] // 2) + 40, image.shape[0])
        pocket_center = pocket_centers[0]

        cv2.line(annotated_image, (int(white_center[0]), int(white_center[1])),
                (int(pocket_center[0]), int(pocket_center[1])), (0, 255, 0), 2)

        vertical_end_point = (int(white_center[0]), 0)
        cv2.line(annotated_image, (int(white_center[0]), int(white_center[1])),
                vertical_end_point, (255, 0, 0), 2)

        vector_line1_x = pocket_center[0] - white_center[0]
        vector_line1_y = pocket_center[1] - white_center[1]

        angle_rad = math.atan2(-vector_line1_x, -vector_line1_y)
        angle_deg = math.degrees(angle_rad)

        print(f"Angle between lines: {angle_deg:.2f} degrees")

        angle_text = f"Angle: {angle_deg:.2f} deg"
        cv2.putText(annotated_image, angle_text, (int(white_center[0]), int(white_center[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        cv2.imwrite('esp32_ann_image.jpg', annotated_image)
    else: 
        angle_deg = 0
        print(f"White ball or pocket not detected")
    
    if os.path.exists(image_path):
        os.remove(image_path)
    
    return round(angle_deg)

def main():
    serial_port = "COM13"
    ser = serial.Serial(serial_port, baudrate=115200, timeout=5)
    ser.flush()
    ip_address = get_ip_address(ser)    
    image_data = fetch_image(ip_address)
    
    if not image_data:
        print("Failed to fetch image. Retrying in 1 second...")
        time.sleep(1)
    
    with open('esp32_cam_image.jpg', 'wb') as f:
        f.write(image_data)

    # Read the image using cv2.imread
    img_rgb = cv2.imread('esp32_cam_image.jpg')
        
    degmove = cvstuff(img_rgb)
    degmove = max(min(degmove, 25), -25)    
    
    message = f"{degmove}\n"
    ser.write(message.encode())
    
    time.sleep(2)
    
if __name__ == "__main__":
    main()
