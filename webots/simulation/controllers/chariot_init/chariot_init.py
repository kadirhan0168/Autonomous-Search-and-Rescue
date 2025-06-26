from controller import Supervisor
import time
import paho.mqtt.client as mqtt
import json
import math
import threading

def pixel_to_webots(pixel_x, pixel_y, image_width, image_height, world_size=3.0):
    x = (pixel_x / image_width) * world_size - world_size / 2
    y = ((image_height - pixel_y) / image_height) * world_size - world_size / 2
    return x, y

class ChariotInitializer:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        self.image_width = 880  # camera
        self.image_height = 1030  # camera
        self.world_size = 3.0
        self.has_moved_blue = False
        self.has_moved_red = False
        self.has_moved_target = False
        
        # Get chariot nodes
        self.chariot_blue = self.supervisor.getFromDef('CHARIOT_BLUE')
        self.chariot_red = self.supervisor.getFromDef('CHARIOT_RED')
        self.target = self.supervisor.getFromDef('TARGET')
        
        if not self.chariot_blue or not self.chariot_red:
            print("ERROR: Chariots not found!")
            return
        
        # Translation and rotation fields for both chariots
        self.trans_field_blue = self.chariot_blue.getField("translation")
        self.trans_field_red = self.chariot_red.getField("translation")
        
        self.rot_field_blue = self.chariot_blue.getField("rotation")
        self.rot_field_red = self.chariot_red.getField("rotation")
        
        self.trans_field_target = self.target.getField("translation")
        
        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the MQTT broker
        self.mqtt_client.connect("172.20.10.2", 1883, 60)

        # Start MQTT loop in a separate thread to avoid blocking the main simulation
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()

        # Wait until MQTT is connected before starting the simulation loop
        self.wait_for_mqtt_connection()

        # Once connected, start the simulation loop and allow the first message to trigger the movement
        while self.supervisor.step(self.timestep) != -1:
            if self.has_moved_blue and self.has_moved_red:
                self.supervisor.simulationSetMode(self.supervisor.SIMULATION_MODE_PAUSE)
                break

    def wait_for_mqtt_connection(self):
        # Wait until the MQTT client connects to the broker
        while not self.mqtt_client.is_connected():
            time.sleep(0.1)
        print("MQTT client connected, starting simulation...")

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        client.subscribe("cars/orientation/blue")
        client.subscribe("cars/orientation/red")  # Subscribe to the red chariot's topic
        client.subscribe("drone/block")

    def on_message(self, client, userdata, msg):
        if not self.has_moved_red or not self.has_moved_blue or not self.has_moved_target:
            try:
                payload = msg.payload.decode()
                data = json.loads(payload)  #{"x": float, "y": float, "angle": float}
    
                pixel_x = data["x"]
                pixel_y = data["y"]
                # angle_deg = data["angle"]
                angle_deg = data.get("angle") if "angle" in data else 0
    
                if msg.topic == "cars/orientation/blue":
                    self.move_chariot(self.chariot_blue, self.trans_field_blue, self.rot_field_blue, pixel_x, pixel_y, angle_deg, "blue")
                elif msg.topic == "cars/orientation/red":
                    self.move_chariot(self.chariot_red, self.trans_field_red, self.rot_field_red, pixel_x, pixel_y, angle_deg, "red")
                elif msg.topic == "drone/block":
                    self.move_target(self.trans_field_target, pixel_x, pixel_y)
            except Exception as e:
                print("MQTT message error:", e)

    def move_chariot(self, chariot, trans_field, rot_field, pixel_x, pixel_y, angle_deg, chariot_color):
        # Convert pixel to Webots coordinates
        x, y = pixel_to_webots(pixel_x, pixel_y, self.image_width, self.image_height, self.world_size)
        new_position = [x, y, 0.05]
        print(f"Moving {chariot_color} chariot to position: {new_position} and angle: {angle_deg}°")

        # Adjust the angle and transform to Webots coordinates
        adjusted_angle_deg = (-angle_deg + 90) % 360
        print(f"Transformed Webots angle: {adjusted_angle_deg}°")

        # Convert angle from degrees to radians
        angle_rad = math.radians(adjusted_angle_deg)
        rotation = [0, 0, 1, angle_rad]

        # Apply translation and rotation
        trans_field.setSFVec3f(new_position)
        rot_field.setSFRotation(rotation)

        # Print out the current rotation (for debugging)
        current_rot = rot_field.getSFRotation()
        current_deg = math.degrees(current_rot[3])
        print(f"{chariot_color} chariot actual rotation: {current_deg}°")

        if chariot_color == "red":
            self.has_moved_red = True  # Mark blue chariot as moved
        else:
            self.has_moved_blue = True  # Mark red chariot as moved
            
    def move_target(self, trans_field_target, pixel_x, pixel_y):
        # Convert pixel to Webots coordinates for the target
        x, y = pixel_to_webots(pixel_x, pixel_y, self.image_width, self.image_height, self.world_size)
        new_position = [x, y, 0.00]
        print(f"Moving target to position: {new_position}")

        # Apply translation (no rotation for target)
        trans_field_target.setSFVec3f(new_position)

        # Print out the current position for the target (for debugging)
        current_position = trans_field_target.getSFVec3f()
        print(f"Target actual position: {current_position}")

        self.has_moved_target = True  # Mark target as moved

controller = ChariotInitializer()
