import cv2
from ultralytics import YOLO
import socket
import json

model = YOLO('bestvs.pt')

cap = cv2.VideoCapture(0)

UDP_IP = "10.150.150.144"
UDP_PORT_SEND = 5005
UDP_PORT_RECEIVE = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
sock.bind(('', UDP_PORT_RECEIVE)) 

capthreframe = None
capture = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Webcam', frame)
    cv2.CAP_PROP_FRAME_WIDTH

    try:
        sock.settimeout(0.1)
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8')
        if message == "1":
            capture = True
            print("1받았어여")
    except socket.timeout:
        pass

    if capture:
        capthreframe = frame.copy()
        capture = False

        results = model(capthreframe)

        count = {"break": 0, "missing parts": 0, "scratch": 0}

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls)
                label = result.names[cls_id]
                confidence = box.conf.item()
                if label in count:
                    count[label] += 1

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(capthreframe, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(capthreframe, f'{label} {confidence:.2f}', (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                

        data = str(count["break"]) + "," + str(count["missing parts"]) + "," + str(count["scratch"])
        #json_data = json.dumps(count)

        try:
            sock.sendto(data.encode(), (UDP_IP, UDP_PORT_SEND))
            print("유니티전송완", data)
        except Exception as e:
            print(f"실패{e}")

    if capthreframe is not None:
        cv2.imshow('캡쳐', capthreframe)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
