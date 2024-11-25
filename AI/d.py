import cv2
import numpy as np
import random

canvas = np.ones((600, 800, 3), dtype=np.uint8) * 255  
drawing = False 
start_x, start_y = -1, -1 
count = 0  

def draw_circle(event, x, y, flags, param):
    global start_x, start_y, drawing, canvas, count  
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_x, start_y = x, y  

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            temp_canvas = canvas.copy()
            radius = int(((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5) 
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) 
            cv2.circle(temp_canvas, (start_x, start_y), radius, random_color, 2) 
            cv2.imshow('Canvas', temp_canvas)

    elif event == cv2.EVENT_LBUTTONUP:  
        drawing = False
        radius = int(((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5) 
        if radius > 0:
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            cv2.circle(canvas, (start_x, start_y), radius, random_color, 2)
            count += 1  # count 증가
            cv2.putText(canvas, f"Count: {count}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('Canvas', canvas)
            
cv2.namedWindow('Canvas')
cv2.setMouseCallback('Canvas', draw_circle)

cv2.imshow('Canvas', canvas)

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord("e"):
        canvas = np.ones((600, 800, 3), dtype=np.uint8) * 255  
        count = 0  
        cv2.imshow('Canvas', canvas)
    if key == 27: 
        break



cv2.destroyAllWindows()
