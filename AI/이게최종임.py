import cv2
import torch
from ultralytics import YOLO
import socket
import json

model = YOLO('ha.pt')

cap = cv2.VideoCapture(0)

UDP_IP = "10.150.150.144"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    count = {"scratch": 0}

    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls_id = int(box.cls)
            label = result.names[cls_id]
            confidence = box.conf.item()  
            if label in count:
                count[label] += 1

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    data = str(count["scratch"])
    try:
        sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"전송실패: {e}")

    cv2.imshow('캠', frame)
    key = cv2.waitKey(30) & 0xFF

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()