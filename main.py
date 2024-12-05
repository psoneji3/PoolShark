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
from ultralytics import YOLO

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
    
def round_away_from_zero(x):
    if x > 0:
        return int(np.ceil(x))
    elif x < 0:
        return int(np.floor(x))
    else:
        return 0
    
def cvstuff(image):
    modelb = YOLO('balls.pt')    # For balls
    modelp = YOLO('pockets.pt')  # For pockets

    annotated_images = []
    class_name_to_id = {"pocket": 0, "ball": 1}
    
    image_path = "esp32_cam_image.jpg"
    image = cv2.imread(image_path)
    resized_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_path = f"temp.jpg"
    cv2.imwrite(image_path, cv2.cvtColor(resized_image, cv2.COLOR_RGB2BGR))

    # Get predictions from both models
    resultb = modelb.predict(image_path, conf=0.6)[0]
    resultp = modelp.predict(image_path, conf=0.5)[0]

    detections = []
    confidence_scores = []
    labels = []
    class_ids = []
    ball_centers = []
    pocket_centers = []
    white_center = None
    target_center = None
    target_diameter = None

    # Process predictions from the balls model
    boxes = resultb.boxes.xyxy.cpu().numpy()
    confidences = resultb.boxes.conf.cpu().numpy()

    if len(boxes) > 0:
        y_centers = []
        ball_diameters = []
        for box in boxes:
            x_min, y_min, x_max, y_max = box
            y_center = (y_min + y_max) / 2
            y_centers.append(y_center)
            diameter = x_max - x_min  # Assuming the ball is roughly circular
            ball_diameters.append(diameter)
        white_ball_index = np.argmax(y_centers)
        target_ball_index = np.argmin(y_centers)

        for idx, (box, conf) in enumerate(zip(boxes, confidences)):
            x_min, y_min, x_max, y_max = box.astype(int)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            diameter = x_max - x_min  # Ball diameter in pixels

            detections.append([x_min, y_min, x_max, y_max])
            confidence_scores.append(conf)
            labels.append('ball')
            class_ids.append(class_name_to_id['ball'])
            ball_centers.append((x_center, y_center))

            if idx == white_ball_index:
                white_center = (x_center, y_center)
                labels[-1] = 'white'  # Change label to 'white' for annotation
            elif idx == target_ball_index:
                target_center = (x_center, y_center)
                target_diameter = diameter
                labels[-1] = 'target'  # Change label to 'target' for annotation

    # Process predictions from the pockets model
    boxes_p = resultp.boxes.xyxy.cpu().numpy()
    confidences_p = resultp.boxes.conf.cpu().numpy()

    if len(boxes_p) > 0:
        for box, conf in zip(boxes_p, confidences_p):
            x_min, y_min, x_max, y_max = box.astype(int)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            detections.append([x_min, y_min, x_max, y_max])
            confidence_scores.append(conf)
            labels.append('pocket')
            class_ids.append(class_name_to_id['pocket'])
            pocket_centers.append((x_center, y_center))

    # Create Detections object
    detection_boxes = sv.Detections(
        xyxy=np.array(detections),
        confidence=np.array(confidence_scores),
        class_id=np.array(class_ids)
    )

    # Annotate image
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    annotated_image = resized_image.copy()
    # annotated_image = box_annotator.annotate(scene=annotated_image, detections=detection_boxes)
    # annotated_image = label_annotator.annotate(scene=annotated_image, detections=detection_boxes, labels=labels)

    # Draw a circle around the white ball to highlight it
    if white_center is not None:
        cv2.circle(annotated_image, (int(white_center[0]), int(white_center[1])), 10, (0, 255, 255), 2)

    # Draw a circle around the target ball to highlight it
    if target_center is not None:
        cv2.circle(annotated_image, (int(target_center[0]), int(target_center[1])), 10, (255, 0, 0), 2)

    # Annotate lines and calculate angle
    if white_center is not None and target_center is not None and pocket_centers:
        pocket_center = pocket_centers[0]

        # Draw line between target ball and pocket
        cv2.line(annotated_image, (int(target_center[0]), int(target_center[1])),
                 (int(pocket_center[0]), int(pocket_center[1])), (0, 255, 0), 3) 

        # Compute vector from target ball to pocket
        vec_target_to_pocket = np.array([pocket_center[0] - target_center[0],
                                         pocket_center[1] - target_center[1]])
        norm_vec = np.linalg.norm(vec_target_to_pocket)
        if norm_vec != 0:
            unit_vec = vec_target_to_pocket / norm_vec
        else:
            unit_vec = np.array([0, 0])  # Avoid division by zero

        # Compute ghost ball location by extending the line beyond the target ball
        ghost_center = target_center - unit_vec * (target_diameter * 0.4)

        # Draw a circle at the ghost ball location
        cv2.circle(annotated_image, (int(ghost_center[0]), int(ghost_center[1])), 10, (0, 0, 255), 2)  # Red circle

        # Draw line from white ball to ghost ball
        cv2.line(annotated_image, (int(white_center[0]), int(white_center[1])),
                 (int(ghost_center[0]), int(ghost_center[1])), (255, 0, 255), 3)  # Purple line

        # Draw vertical line from white ball upwards
        vertical_end_point = (int(white_center[0]), 0)
        cv2.line(annotated_image, (int(white_center[0]), int(white_center[1])),
                 vertical_end_point, (255, 0, 0), 1)

        # Calculate angle between the two lines (white to ghost ball and vertical)
        vector_line1 = np.array([ghost_center[0] - white_center[0], ghost_center[1] - white_center[1]])
        vector_line2 = np.array([0, -1])  # Vertical line upwards

        # Adjust for zero division
        if np.linalg.norm(vector_line1) != 0:
            angle_rad = np.arctan2(
                vector_line1[0] * vector_line2[1] - vector_line1[1] * vector_line2[0],
                vector_line1[0] * vector_line2[0] + vector_line1[1] * vector_line2[1]
            )
            angle_deg = np.degrees(angle_rad)
        else:
            angle_deg = 0.0


        print(f"Image: Angle between lines (white to ghost ball): {angle_deg:.2f} degrees")

        # Annotate angle on image
        angle_text = f"Angle: {angle_deg:.2f} deg"
        cv2.putText(annotated_image, angle_text, (int(white_center[0]), int(white_center[1]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        cv2.imwrite('esp32_ann_image.jpg', annotated_image)
    
    if os.path.exists(image_path):
        os.remove(image_path)
    
    return round_away_from_zero(angle_deg)

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
    
    print("got image")
        
    degmove = cvstuff(img_rgb)
    degmove = max(min(degmove, 28), -28)
    
    message = f"{degmove}\n"
    ser.write(message.encode())
    
    time.sleep(2)
    
if __name__ == "__main__":
    main()
