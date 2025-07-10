import paho.mqtt.client as mqtt
import json
import math
import sys
import threading
import time

# MQTT client for the car controller
class CarController:
    def __init__(self, color, mqtt_client):
        self.color = color
        self.position = (0, 0, 0)
        self.other_position = None
        self.target = None
        self.client = mqtt_client
        self.last_command = None
        self.blocked = False   # True if an obstacle is detected

    # is for handling incoming MQTT messages
    def on_message(self, client, userdata, msg):
        topic = msg.topic

        # is for handling status messages 
        if topic == f"cars/status/{self.color}":
            status = msg.payload.decode().strip().lower()
            print(f"{self.color.upper()}: STATUS received → '{status}'")

            if status == "obstakel":
                print(f"{self.color.upper()}: OBSTAKEL received → pausing navigation")
                self.blocked = True
                self.send_command("stop")  # Stop the car when an obstacle is detected
            elif status == "done":
                print(f"{self.color.upper()}: DONE received → resuming navigation")
                self.blocked = False
                self.navigate()
            return  

        # handling orientation messages
        try:
            data = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"{self.color.upper()}: Invalid JSON on topic {topic}")
            return

        # handling the car's own position
        if topic == f"cars/orientation/{self.color}":
            self.position = (data['x'], data['y'], data['angle'])
            self.navigate()

        # handling the other car's position
        elif topic == f"cars/orientation/{'red' if self.color == 'blue' else 'blue'}":
            self.other_position = (data['x'], data['y'], data['angle'])
            self.navigate()

        # handling drone block messages
        elif topic == "drone/block":
            self.target = (data['x'], data['y'])
            self.navigate()

    # navigating the car towards the target
    def navigate(self):
        if self.blocked:
            print(f"{self.color.upper()}: Navigation is currently blocked (waiting for 'done').")
            return

        if not self.target or not self.position:
            print("Waiting for position or target...")
            return

        # calculating the distance to the target
        x, y, angle = self.position
        tx, ty = self.target
        distance = math.hypot(tx - x, ty - y)

        # avoiding the other car
        if self.other_position:
            ox, oy, _ = self.other_position
            distance_to_other = math.hypot(ox - x, oy - y)

            if distance_to_other < 80:
                print(f"{self.color.upper()} avoiding other car ({int(distance_to_other)}px)")

                # is for calculating the avoidance direction
                dx = x - ox
                dy = y - oy
                avoid_angle = math.degrees(math.atan2(dy, dx)) % 360
                current_angle = angle % 360
                angle_diff = (avoid_angle - current_angle + 540) % 360 - 180

                direction = "right" if angle_diff > 0 else "left" if abs(angle_diff) > 20 else "forward"
                print(f"Avoiding direction: {direction.upper()}")
                self.send_command(direction)

                threading.Timer(1.5, self.navigate).start()
                return

        # is for calculating the angle to the target
        dx, dy = tx - x, ty - y
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        current_angle = angle % 360
        angle_diff = (target_angle - current_angle + 540) % 360 - 180
        
        # printing the current state
        print(f"\n{self.color.upper()} Car:")
        print(f"Position: ({x}, {y}) | Angle: {int(angle)}°")
        print(f"Target: ({tx}, {ty}) | Distance: {int(distance)}px | Direction: {int(target_angle)}°")
        print(f"Angle difference: {int(angle_diff)}°")

        # determining the command based on distance and angle
        if distance < 200:
            command = "stop"
            print("Target reached")
        elif abs(angle_diff) > 20:
            command = "right" if angle_diff > 0 else "left"
            print(f"Turning {command.upper()}")
        else:
            command = "forward"
            print("Driving forward")

        self.send_command(command)

    # sending commands to the car
    def send_command(self, action):
        if self.blocked:
            print(f"{self.color.upper()}: In blocked state (obstakel), '{action}' not sent.")
            return

        topic = f"cars/control/{self.color}/"
        self.client.publish(topic, action)
        print(f"Sent to {topic}: {action}\n")


# Start the client
def main():
    if len(sys.argv) != 2:
        print("Usage: python car_client.py <color>")
        sys.exit(1)

    color = sys.argv[1].lower()
    if color not in ['red', 'blue']:
        print("Invalid color. Choose from: red, blue")
        sys.exit(1)
        
    # initializing the MQTT client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    car = CarController(color, client)
    client.on_message = car.on_message

    try:
        client.connect("172.20.10.2", 1883, 60)
    except:
        print(f"{color.upper()}: Cannot connect to MQTT broker.")
        sys.exit(1)

    # subscribing to topics
    other_color = 'red' if color == 'blue' else 'blue'
    client.subscribe(f"cars/orientation/{color}")
    client.subscribe(f"cars/orientation/{other_color}")
    client.subscribe("drone/block")
    client.subscribe(f"cars/status/{color}")

    print(f"Car controller started for color: {color.upper()}...")
    client.loop_forever()


if __name__ == "__main__":
    main()
