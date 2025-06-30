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
        self.silent_until = 0 
        
    # Callback for incoming MQTT messages
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            data = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"{self.color.upper()}: Invalid JSON on topic {topic}")
            return
        
        # Check if the message is relevant to this car      
        if topic == f"cars/orientation/{self.color}":
            self.position = (data['x'], data['y'], data['angle'])  # car position
            self.navigate()
        elif topic == f"cars/orientation/{'red' if self.color == 'blue' else 'blue'}":
            self.other_position = (data['x'], data['y'], data['angle'])
            self.navigate()
        elif topic == "drone/block":
            self.target = (data['x'], data['y'])
            self.navigate()
        elif topic == "cars/status/red" and self.color == "red":
            status = msg.payload.decode().strip().lower()
            if status == "stop":
                print(f"{self.color.upper()}: STOP received on status topic → silence for 5 seconds")
                self.silent_until = time.time() + 5

    # Navigate towards the target position
    def navigate(self):
        if not self.target or not self.position:
            print("Waiting for position or target...")
            return
        
        # Calculate distance and angle to the target
        x, y, angle = self.position
        tx, ty = self.target
        distance = math.hypot(tx - x, ty - y)

        # Check if the other car is close enough to avoid
        if self.other_position:
            ox, oy, _ = self.other_position
            distance_to_other = math.hypot(ox - x, oy - y)
            
            # If the other car is too close, avoid it
            if distance_to_other < 80:
                print(f"{self.color.upper()} avoiding other car ({int(distance_to_other)}px)")

                # Calculate the direction to avoid the other car
                dx = x - ox
                dy = y - oy
                avoid_angle = math.degrees(math.atan2(dy, dx)) % 360
                current_angle = angle % 360
                angle_diff = (avoid_angle - current_angle + 540) % 360 - 180

                # Determine the direction to avoid
                if abs(angle_diff) > 20:
                    direction = "right" if angle_diff > 0 else "left"
                else:
                    direction = "forward"

                print(f"Avoiding direction: {direction.upper()}")
                self.send_command(direction)

                threading.Timer(1.5, self.navigate).start()
                return
        
        # Calculate the angle to the target
        dx, dy = tx - x, ty - y
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        current_angle = angle % 360
        angle_diff = (target_angle - current_angle + 540) % 360 - 180

        # Print navigation details
        print(f"\n{self.color.upper()} Car:")
        print(f"Position: ({x}, {y}) | Angle: {int(angle)} degrees")
        print(f"Target: ({tx}, {ty}) | Distance: {int(distance)} pixels | Direction: {int(target_angle)} degrees")
        print(f"Angle difference: {int(angle_diff)} degrees")

        # Determine command based on distance and angle difference
        if distance < 150:
            command = "stop"
            print("Target reached")
        elif abs(angle_diff) > 20:
            command = "right" if angle_diff > 0 else "left"
            print(f"Turning {command.upper()}")
        else:
            command = "forward"
            print("Driving forward")

        self.send_command(command)

    # Send command to the MQTT broker
    def send_command(self, action):
        if self.color == "red" and time.time() < self.silent_until:
            print(f"{self.color.upper()}: In silent mode, '{action}' not sent.")
            return

        topic = f"cars/control/{self.color}/"
        self.client.publish(topic, action)
        print(f"Sent to {topic}: {action}\n")


# Main function to run the car client
def main():
    if len(sys.argv) != 2:
        print("Usage: python car_client.py <color>")
        sys.exit(1)

    color = sys.argv[1].lower()
    if color not in ['red', 'blue']:
        print("Invalid color. Choose from: red, blue")
        sys.exit(1)
   
    # Initialize MQTT client   
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    car = CarController(color, client)
    client.on_message = car.on_message

    try:
        client.connect("172.20.10.2", 1883, 60)
    except:
        print(f"{color.upper()}: Cannot connect to MQTT broker.")
        sys.exit(1)

    # Subscribe to topics for the car's color and the other color
    other_color = 'red' if color == 'blue' else 'blue'
    client.subscribe(f"cars/orientation/{color}")
    client.subscribe(f"cars/orientation/{other_color}")
    client.subscribe("drone/block")
    client.subscribe("cars/status/red")  

    print(f"Car controller started for color: {color.upper()}...")
    client.loop_forever()

if __name__ == "__main__":
    main()
