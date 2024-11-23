import cv2
from ultralytics import YOLO
import socket
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np

model = YOLO('best4.pt')

cap = cv2.VideoCapture(1)

UDP_IP = "192.168.0.100"
UDP_PORT_SEND = 5005
UDP_PORT_RECEIVE = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
sock.bind(('', UDP_PORT_RECEIVE)) 

capture_frame = None

capture = False

root = tk.Tk()
root.title("screen")

root.geometry("800x400")

frame = tk.Frame(root)
frame.pack()

camera = tk.Label(frame)
camera.grid(row=0, column=0, pady=100)

capture_ = tk.Label(frame)
capture_.grid(row=0, column=1, padx=20, pady=100)

IMG_WIDTH = 380
IMG_HEIGHT = 240

capframe = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
capframe = Image.fromarray(cv2.cvtColor(capframe, cv2.COLOR_BGR2RGB))
cap_imgtk = ImageTk.PhotoImage(image=capframe)
capture_.imgtk = cap_imgtk
capture_.configure(image=cap_imgtk)

def update_frame():
    global capture_frame, capture

    ret, frame = cap.read()
    if not ret:
        return

    frame_resized = cv2.resize(frame, (IMG_WIDTH, IMG_HEIGHT))

    img = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(img)
    imgtk = ImageTk.PhotoImage(image=img)
    camera.imgtk = imgtk
    camera.configure(image=imgtk)

    try:
        sock.settimeout(0.1)
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8')
        if message == "1":
            capture = True
            print("1 받았어여")
    except socket.timeout:
        pass

    if capture:
        results = model(frame_resized)

        count = {"scratch": 0, "break": 0}

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls)
                label = result.names[cls_id]
                confidence = box.conf.item()
                if label in count:
                    count[label] += 1

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame_resized, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        data = str(count["scratch"]) + ", " + str(count["break"])
        try:
            sock.sendto(data.encode(), (UDP_IP, UDP_PORT_SEND))
            print("유니티 전송완:", data)
        except Exception as e:
            print(f"실패다실패: {e}")

        capture_frame = frame_resized.copy()
        display_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        display_img = Image.fromarray(display_frame)
        display_imgtk = ImageTk.PhotoImage(image=display_img)
        capture_.imgtk = display_imgtk
        capture_.configure(image=display_imgtk)
        
        capture = False 

    root.after(10, update_frame)

update_frame()
root.mainloop()