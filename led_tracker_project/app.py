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
#mqtt_client.connect("192.168.32.129", 1883, 60)
#mqtt_client.connect("192.168.32.129", 1883, 60)
mqtt_client.connect("172.20.10.2", 1883, 60)



mqtt_client.loop_start()

# Parameters
vmin_value = 60
red_hue_max = 15
green_hue_max = 90

# Detectie-gegevens
last_points = {'red': None, 'green': None, 'blue': None, 'white': None, 'yellow': None}
latest_detected_positions = {'red': None, 'green': None, 'blue': None, 'white': None, 'yellow': None}
last_seen_time = {'red': 0, 'green': 0, 'blue': 0, 'white': 0, 'yellow': 0}

stabilization_threshold = 20
last_detection_time = 0
detection_interval = 0.1 # Interval tussen detecties in seconden

def stabilize_point(new_point, color):  # Definieert een functie die een nieuw punt stabiliseert op basis van kleur
    old = last_points[color]  
    if old is None: 
        last_points[color] = new_point  # Slaat het nieuwe punt op als eerste punt voor deze kleur
        return new_point  
    dist = math.hypot(new_point[0] - old[0], new_point[1] - old[1])  # Berekent de afstand tussen het nieuwe en het oude punt
    if dist < stabilization_threshold:  # Controleert of de afstand kleiner is dan de stabilisatiedrempel
        return old  
    else: 
        last_points[color] = new_point  # Werk het laatst bekende punt bij naar het nieuwe punt
        return new_point  


def detect_led(hsv, color):
#    if color == 'red':
#        mask1 = cv2.inRange(hsv, (0, 150, vmin_value), (red_hue_max, 255, 255))
#        mask2 = cv2.inRange(hsv, (170, 150, vmin_value), (180, 255, 255))
#        mask = cv2.bitwise_or(mask1, mask2)

#    elif color == 'green':
#        mask = cv2.inRange(hsv, (35, 150, vmin_value), (green_hue_max, 255, 255))
#        mask = cv2.inRange(hsv, (35, 150, vmin_value), (85, 255, 255)) # van de andere code

#    elif color == 'blue':
#        mask = cv2.inRange(hsv, (116, 112, 240), (119, 255, 255))
#        mask = cv2.inRange(hsv, (110, 60, 180), (120, 255, 255)) # de is echt de beste

#    elif color == 'white':
#        mask = cv2.inRange(hsv, (0, 0, 245), (180, 10, 255))
#        mask = cv2.inRange(hsv, (0, 0, 245), (180, 30, 255))  # van de andere code
#
#    elif color == 'yellow':
#       mask = cv2.inRange(hsv, (0, 124, 224), (36, 255, 255))  # H(0-36), S(124-255), V(224-255)
#       mask = cv2.inRange(hsv, (19, 108, 118), (33, 255, 255))
#       mask = cv2.inRange(hsv, (0, 43, 232), (29, 156, 255))  # H(0-29), S(43-156), V(232-255)


    if color == 'red':
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == 'green':
        mask = cv2.inRange(hsv, (45, 100, 50), (80, 255, 255))
        # mask = cv2.inRange(hsv, (60, 100, 60), (80, 255, 255))
        #mask = cv2.inRange(hsv, (40, 80, 80), (70, 255, 255))
    elif color == 'blue':
        mask = cv2.inRange(hsv, (110, 150, 50), (130, 255, 255))
    elif color == 'white':
        mask = cv2.inRange(hsv, (0, 0, 180), (180, 40, 255)) #in donker werkt goed
        #mask = cv2.inRange(hsv, (0, 0, 245), (180, 30, 255))
    elif color == 'yellow':
        #mask = cv2.inRange(hsv, (10, 100, 20), (30, 255, 100))
        mask = cv2.inRange(hsv, (10, 150, 60), (20, 255, 160))

    else:
        return None

    kernel = np.ones((3, 3), np.uint8)  # Maakt een 3x3 kernel van enen voor morfologische bewerking (type: uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Voert een 'open' morfologische bewerking uit (verwijdert ruis)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Vindt de contouren in het masker, alleen de externe

    if contours:  # Controleert of er contouren zijn gevonden
        largest = max(contours, key=cv2.contourArea)  # Zoekt de grootste contour op basis van oppervlakte
        if cv2.contourArea(largest) > 0:  # Negeert kleine contouren (ruis of irrelevante objecten)
            M = cv2.moments(largest)  # Berekent de momentwaarden van de grootste contour
            if M["m00"] != 0:  # Voorkomt deling door nul (om zeker te zijn dat het moment bruikbaar is)
                cx = int(M["m10"] / M["m00"])  # Berekent het zwaartepunt x-coördinaat
                cy = int(M["m01"] / M["m00"])  # Berekent het zwaartepunt y-coördinaat
                return (cx, cy)  # Geeft het zwaartepunt van de contour terug als (x, y)
    return None  


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

def gen_frames():
    cap = cv2.VideoCapture(0)
    global last_detection_time, latest_detected_positions, last_seen_time

    while True:
        success, frame = cap.read()
        if not success:
            break

        # save
        cv2.imwrite('latest_frame.jpg', frame)  # Slaat het huidige frame op als 'latest_frame.jpg'
        # Converteer het frame naar HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Maak een zwart frame als achtergrond
        output = frame.copy()

        current_time = time.time()

        if current_time - last_detection_time >= detection_interval:  # Controleert of het tijd is voor een nieuwe detectieronde
            last_detection_time = current_time  # Werkt de tijd bij waarop de laatste detectie plaatsvond
            for color in latest_detected_positions:  # Loopt over alle te detecteren kleuren
                point = detect_led(hsv, color)  # Probeert een LED van deze kleur te detecteren in het HSV-beeld
                if point:  # Als een punt is gedetecteerd
                    latest_detected_positions[color] = stabilize_point(point, color)  # Stabiliseert en slaat het gedetecteerde punt op
                    last_seen_time[color] = current_time  # Werkt de laatst geziene tijd bij voor deze kleur
                elif color != 'yellow' and current_time - last_seen_time[color] > 1.5:  # Als het punt al langer dan 1.5s niet is gezien (behalve geel)
                    latest_detected_positions[color] = None  # Verwijdert het punt uit de laatst bekende posities


        # Kleurinstellingen voor het tekenen
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'white': (255, 255, 255),
            'yellow': (0, 255, 255)
        }

        # Teken de gedetecteerde kleuren op het zwarte frame
        for color, point in latest_detected_positions.items():
            if point:
                cv2.circle(output, point, 5, color_map[color], -1)

        p = latest_detected_positions

        # Publiseer auto-data en teken lijnen voor de auto's
        if p['red'] and p['green']:
            publish_car_data('red', p['red'], p['green']) # wat is voor en wat is achter 
            angle = math.degrees(math.atan2(p['green'][1] - p['red'][1], p['green'][0] - p['red'][0]))
            distance = math.hypot(p['green'][0] - p['red'][0], p['green'][1] - p['red'][1])
            cv2.line(output, p['red'], p['green'], (0, 255, 255), 2)
            cv2.putText(output, f"A1 {int(angle)}° {int(distance)}px", p['red'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        if p['blue'] and p['white']:
            publish_car_data('blue', p['blue'], p['white'])
            angle = math.degrees(math.atan2(p['white'][1] - p['blue'][1], p['white'][0] - p['blue'][0]))
            distance = math.hypot(p['white'][0] - p['blue'][0], p['white'][1] - p['blue'][1])
            cv2.line(output, p['blue'], p['white'], (0, 255, 255), 2)
            cv2.putText(output, f"A2 {int(angle)}° {int(distance)}px", p['blue'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Gele LED-positie publiceren naar 'drone/block'
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

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=5001, debug=True)