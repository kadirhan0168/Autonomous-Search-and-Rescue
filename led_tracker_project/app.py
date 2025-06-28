from flask import Flask, render_template, Response
import cv2
import numpy as np
import math
import paho.mqtt.client as mqtt
import time
import json

app = Flask(__name__)

# MQTT setup
mqtt_client = mqtt.Client()
mqtt_client.connect("", 1883, 60) # Replace with your MQTT broker address 

mqtt_client.loop_start()

# Parameters
vmin_value = 60
red_hue_max = 15
green_hue_max = 90

# Detection data
last_points = {'red': None, 'green': None, 'blue': None, 'white': None, 'yellow': None}
latest_detected_positions = {'red': None, 'green': None, 'blue': None, 'white': None, 'yellow': None}
last_seen_time = {'red': 0, 'green': 0, 'blue': 0, 'white': 0, 'yellow': 0}

stabilization_threshold = 20
last_detection_time = 0
detection_interval = 0.1 

def stabilize_point(new_point, color):  # Defines a function that stabilizes a new point based on color
    old = last_points[color]  
    if old is None: 
        last_points[color] = new_point  # Stores the new point as the initial point for this color
        return new_point  
    dist = math.hypot(new_point[0] - old[0], new_point[1] - old[1])  # Calculates the distance between the new point and the previous point
    if dist < stabilization_threshold:  
        return old  
    else: 
        last_points[color] = new_point 
        return new_point  

#LED color detection using HSV thresholding
def detect_led(hsv, color):
    if color == 'red':
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == 'green':
        mask = cv2.inRange(hsv, (45, 100, 50), (80, 255, 255))
    elif color == 'blue':
        mask = cv2.inRange(hsv, (110, 150, 50), (130, 255, 255))
    elif color == 'white':
        mask = cv2.inRange(hsv, (0, 0, 180), (180, 40, 255)) 
    elif color == 'yellow':
        mask = cv2.inRange(hsv, (10, 150, 60), (20, 255, 160))

    else:
        return None

    kernel = np.ones((3, 3), np.uint8)  # Creates a 3x3 kernel of ones for morphological operations (type: uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Performs an 'open' morphological operation (removes noise)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Finds the contours in the mask, external ones only

    if contours:  # Checks if any contours have been found
        largest = max(contours, key=cv2.contourArea)  
        if cv2.contourArea(largest) > 0:  
            M = cv2.moments(largest) 
            if M["m00"] != 0: 
                cx = int(M["m10"] / M["m00"]) 
                cy = int(M["m01"] / M["m00"]) 
                return (cx, cy) 
    return None
  
# Function to publish car data to MQTT
def publish_car_data(car_id, point_back, point_front):
    x, y = point_back
    angle = math.degrees(math.atan2(point_front[1] - point_back[1], point_front[0] - point_back[0]))
    angle = angle % 360

    data = {
        "x": x,
        "y": y,
        "angle": int(angle),
        "timestamp": time.time()
    }

    mqtt_client.publish(f"cars/orientation/{car_id}", json.dumps(data))
    
    # Publish the other car's position for navigation
def gen_frames():
    cap = cv2.VideoCapture(0)
    global last_detection_time, latest_detected_positions, last_seen_time

    while True:
        success, frame = cap.read()
        if not success:
            break

        # save
        cv2.imwrite('latest_frame.jpg', frame)  
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        
        output = frame.copy()

        current_time = time.time()
        
# Detect the yellow block
        if current_time - last_detection_time >= detection_interval:  
            last_detection_time = current_time 
            for color in latest_detected_positions: 
                point = detect_led(hsv, color) 
                if point:  
                    latest_detected_positions[color] = stabilize_point(point, color)  
                    last_seen_time[color] = current_time  
                elif color != 'yellow' and current_time - last_seen_time[color] > 1.5: 
                    latest_detected_positions[color] = None  


        # Color settings for drawing
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'white': (255, 255, 255),
            'yellow': (0, 255, 255)
        }

        for color, point in latest_detected_positions.items():
            if point:
                cv2.circle(output, point, 5, color_map[color], -1)

        p = latest_detected_positions

        # Publish car data and draw lines for the vehicles
        if p['red'] and p['green']:
            publish_car_data('red', p['red'], p['green']) # wat is voor en wat is achter 
            angle = math.degrees(math.atan2(p['green'][1] - p['red'][1], p['green'][0] - p['red'][0]))
            distance = math.hypot(p['green'][0] - p['red'][0], p['green'][1] - p['red'][1])
            cv2.line(output, p['red'], p['green'], (0, 255, 255), 2)
            cv2.putText(output, f"A1 {int(angle)}° {int(distance)}px", p['red'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
#         if p['green'] and p['blue']:
        if p['blue'] and p['white']:
            publish_car_data('blue', p['blue'], p['white'])
            angle = math.degrees(math.atan2(p['white'][1] - p['blue'][1], p['white'][0] - p['blue'][0]))
            distance = math.hypot(p['white'][0] - p['blue'][0], p['white'][1] - p['blue'][1])
            cv2.line(output, p['blue'], p['white'], (0, 255, 255), 2)
            cv2.putText(output, f"A2 {int(angle)}° {int(distance)}px", p['blue'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Publish the yellow block position
        if p['yellow']:
            x, y = p['yellow']
            data = {
                "x": x,
                "y": y,
                "timestamp": time.time()
            }
            mqtt_client.publish("drone/block", json.dumps(data))  # <-- aangepast topic
            cv2.putText(output, f"Y ({x},{y})", (x+5, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        #print(f"Red: {p['red']}, Green: {p['green']}, Blue: {p['blue']}, White: {p['white']}, Yellow: {p['yellow']}")
        # Zet het resultaat om naar een jpeg-beeld voor streaming
        ret, buffer = cv2.imencode('.jpg', output)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')

## Video feed route
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=5001, debug=True)
