from controller import Robot
import cv2
import time
import numpy as np
import paho.mqtt.client as mqtt
import json
import math
from PIL import Image
import io
import threading

# MQTT Setup
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.connect("localhost", 1883, 60)
mqtt_client.loop_start()

# Global image buffer
latest_frame = None
frame_lock = threading.Lock()

vmin_value = 60
red_hue_max = 15
green_hue_max = 90

last_points = {'red': None, 'green': None, 'blue': None, 'white': None, 'purple': None}
latest_detected_positions = {'red': None, 'green': None, 'blue': None, 'white': None, 'purple': None}
last_seen_time = {'red': 0, 'green': 0, 'blue': 0, 'white': 0, 'purple': 0}

stabilization_threshold = 20
last_detection_time = 0
detection_interval = 0.5

# MQTT image callback
def on_image_message(client, userdata, msg):
    global latest_frame
    try:
        image = Image.open(io.BytesIO(msg.payload)).convert("RGB")
        frame = np.array(image)
        with frame_lock:
            latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"Image decode error: {e}")

mqtt_client.subscribe("drone_sim/camera")
mqtt_client.on_message = on_image_message

def stabilize_point(new_point, color):
    old = last_points[color]
    if old is None:
        last_points[color] = new_point
        return new_point
    dist = math.hypot(new_point[0] - old[0], new_point[1] - old[1])
    if dist < stabilization_threshold:
        return old
    else:
        last_points[color] = new_point
        return new_point

def detect_led(hsv, color):
    if color == 'red':
        mask1 = cv2.inRange(hsv, (0, 150, vmin_value), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 150, vmin_value), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == 'green':
        mask = cv2.inRange(hsv, (35, 150, vmin_value), (green_hue_max, 255, 255))
    elif color == 'blue':
        mask = cv2.inRange(hsv, (110, 60, 180), (120, 255, 255))
    elif color == 'white':
        mask = cv2.inRange(hsv, (0, 0, 245), (180, 30, 255))
    elif color == 'purple':
        mask = cv2.inRange(hsv, (130, 50, 50), (160, 255, 255))
    else:
        return None

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 10:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
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

# Kleurbereiken (HSV)
# color_ranges = {
    # 'red':          [(0, 160, 180), (10, 255, 255)],
    # 'green':        [(40, 160, 180), (80, 255, 255)],
    # 'blue':         [(100, 150, 220), (130, 255, 255)],
    # 'block':        [(130, 50, 50), (160, 255, 255)],
    # 'front_red':    [(20, 100, 100), (40, 255, 255)],
    # 'front_green':  [(10, 100, 20), (20, 255, 180)],
    # 'front_blue':   [(0, 0, 200), (180, 30, 255)]
# }

min_area = 150
min_front_area = 25

def publish_position(topic, x, y, angle=None):
    payload = {
        'x': x,
        'y': y,
        'timestamp': time.time()
    }
    if angle is not None:
        payload['angle'] = angle
    mqtt_client.publish(topic, json.dumps(payload))

# def detect_orientation(hsv_frame, color):
    # main_mask = cv2.inRange(hsv_frame, 
                            # np.array(color_ranges[color][0]), 
                            # np.array(color_ranges[color][1]))
    
    # front_key = f"front_{color}"
    # if front_key not in color_ranges:
        # return None

    # front_mask = cv2.inRange(hsv_frame, 
                             # np.array(color_ranges[front_key][0]), 
                             # np.array(color_ranges[front_key][1]))

    # contours_main, _ = cv2.findContours(main_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours_front, _ = cv2.findContours(front_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # if contours_main and contours_front:
        # c_main = max(contours_main, key=cv2.contourArea)
        # if cv2.contourArea(c_main) > min_area:
            # M_main = cv2.moments(c_main)
            # if M_main["m00"] == 0:
                # return None
            # cx_main = int(M_main["m10"] / M_main["m00"])
            # cy_main = int(M_main["m01"] / M_main["m00"])

            # min_dist = float('inf')
            # best_front = None
            # for c_front in contours_front:
                # if cv2.contourArea(c_front) < min_front_area:
                    # continue
                # M_front = cv2.moments(c_front)
                # if M_front["m00"] == 0:
                    # continue
                # cx_front = int(M_front["m10"] / M_front["m00"])
                # cy_front = int(M_front["m01"] / M_front["m00"])
                # dist = math.hypot(cx_front - cx_main, cy_front - cy_main)
                # if dist < min_dist:
                    # min_dist = dist
                    # best_front = (cx_front, cy_front)

            # if best_front:
                # cx_front, cy_front = best_front
                # angle = math.degrees(math.atan2(cy_front - cy_main, cx_front - cx_main))
                # return cx_main, cy_main, angle
    # return None

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    block_center = None

    while robot.step(timestep) != -1:
        now = time.time()
    
        with frame_lock:
            frame_copy = None if latest_frame is None else latest_frame.copy()

        if frame_copy is None:
            continue

        hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
        
        
        if now - last_detection_time >= detection_interval:
            last_detection_time = now
            for color in latest_detected_positions:
                point = detect_led(hsv, color)
                if point:
                    latest_detected_positions[color] = stabilize_point(point, color)
                    last_seen_time[color] = now
                elif color != 'yellow' and now - last_seen_time[color] > 1.5:
                    latest_detected_positions[color] = None
    
        p = latest_detected_positions
    
        if p['red'] and p['green']:
            publish_car_data('red', p['red'], p['green'])
    
        if p['blue'] and p['white']:
            publish_car_data('blue', p['blue'], p['white'])
    
        if p['purple']:
            x, y = p['purple']
            data = {
                "x": x,
                "y": y,
                "timestamp": time.time()
            }
            mqtt_client.publish("drone_sim/target_coords", json.dumps(data))
            mqtt_client.publish("drone_sim/target_found", "found")

        # Blokdetectie
        # block_mask = cv2.inRange(hsv, np.array(color_ranges['block'][0]), np.array(color_ranges['block'][1]))
        # block_contours, _ = cv2.findContours(block_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # if block_contours:
            # c = max(block_contours, key=cv2.contourArea)
            # if cv2.contourArea(c) > min_area:
                # x, y, w, h = cv2.boundingRect(c)
                # cx, cy = x + w // 2, y + h // 2
                # block_center = (cx, cy)
                # publish_position("drone_sim/target_coords", cx, cy)
                # mqtt_client.publish("drone_sim/target_found", "found")
        # else:
            # block_center = None

        # Autodetectie per kleur
        # for color in ['red', 'green', 'blue']:
            # result = detect_orientation(hsv, color)
            # if result:
                # x, y, angle = result
                # publish_position(f"cars/orientation/{color}", x, y, angle)

                # if block_center is not None:
                    # dist = math.hypot(block_center[0] - x, block_center[1] - y)
                    # mqtt_client.publish(f"cars/distance/{color}", json.dumps({
                        # "distance_pixels": dist,
                        # "timestamp": time.time()
                    # }))
