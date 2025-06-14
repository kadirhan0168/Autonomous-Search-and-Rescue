import paho.mqtt.client as mqtt
import json
import math
import sys

class CarController:
    def __init__(self, color, mqtt_client):
        self.color = color
        self.position = (0, 0, 0)  # x, y, angle
        self.target = None
        self.client = mqtt_client

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            data = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"{self.color.upper()}: Ongeldige JSON op {topic}")
            return

        if topic == f"cars/orientation/{self.color}":
            self.position = (data['x'], data['y'], data['angle'])
        elif topic == "drone/block":
            self.target = (data['x'], data['y'])
            self.navigate()

    def navigate(self):
        if not self.target or not self.position:
            return

        tx, ty = self.target
        x, y, angle = self.position

        dx, dy = tx - x, ty - y
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        current_angle = angle % 360

        angle_diff = (target_angle - current_angle + 540) % 360 - 180
        distance = math.hypot(dx, dy)

        print(f"{self.color.upper()} Auto:")
        print(f"Positie: ({x}, {y}) | Hoek: {int(angle)}°")
        print(f"Doel: ({tx}, {ty}) | Afstand: {int(distance)}px | Richting: {int(target_angle)}°")
        print(f"Hoekverschil: {int(angle_diff)}°")

        # Besturingslogica
        command = None
        if distance < 300:
            command = "stop"
            print("Doel bereikt!")
        elif abs(angle_diff) > 20:
            command = "right" if angle_diff > 0 else "left"
            print(f"Draai {command.upper()}")
        else:
            command = "forward"
            print(f"Rijd vooruit")

        # Stuur commando naar auto
        self.send_command(command)

    def send_command(self, action):
        topic = f"cars/control/{self.color}"
        payload = json.dumps({"action": action})
        self.client.publish(topic, payload)
        print(f"Verzonden naar {topic}: {payload}\n")

def main():
    if len(sys.argv) != 2:
        print("Gebruik: python car_client.py <kleur>")
        sys.exit(1)

    color = sys.argv[1].lower()
    if color not in ['red', 'blue']:
        print("Ongeldige kleur. Kies uit: red, blue")
        sys.exit(1)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    car = CarController(color, client)
    client.on_message = car.on_message

    try:
        client.connect("localhost", 1883, 60)
    except:
        print(f"{color.upper()}: Kan geen verbinding maken met MQTT broker op localhost:1883.")
        sys.exit(1)

    client.subscribe(f"cars/orientation/{color}")
    client.subscribe("drone/block")

    print(f"Auto-controller gestart voor kleur: {color.upper()}...\n")
    client.loop_forever()

if __name__ == "__main__":
    main()
