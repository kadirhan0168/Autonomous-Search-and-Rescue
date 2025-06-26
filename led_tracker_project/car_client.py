import paho.mqtt.client as mqtt
import json
import math
import sys
import threading

class CarController:
    def __init__(self, color, mqtt_client):
        self.color = color
        self.position = (0, 0, 0)  # Eigen positie: x, y, angle
        self.other_position = None  # Positie van de andere auto
        self.target = None  # Doelpositie
        self.client = mqtt_client
        self.last_command = None  # Laatst verstuurde commando

    def on_message(self, client, userdata, msg):
        # Verwerk binnenkomende MQTT-berichten
        topic = msg.topic
        try:
            data = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            print(f"{self.color.upper()}: Ongeldige JSON op {topic}")
            return

        if topic == f"cars/orientation/{self.color}":
            # Eigen positiedata ontvangen
            self.position = (data['x'], data['y'], data['angle'])
            self.navigate()
        elif topic == f"cars/orientation/{'red' if self.color == 'blue' else 'blue'}":
            # Positiedata van andere auto ontvangen
            self.other_position = (data['x'], data['y'], data['angle'])
            self.navigate()
        elif topic == "drone/block":
            # Doelpositie ontvangen
            self.target = (data['x'], data['y'])
            self.navigate()

    def navigate(self):
        # Controleer of zowel positie als doel bekend zijn
        if not self.target or not self.position:
            print("Wachten op positie of target...")
            return

        x, y, angle = self.position
        tx, ty = self.target
        distance = math.hypot(tx - x, ty - y)

        # Voorkom botsing als de andere auto dichtbij is
        if self.other_position:
            ox, oy, _ = self.other_position
            afstand_tot_andere = math.hypot(ox - x, oy - y)
            print(f"afstand tot andere auto: {int(afstand_tot_andere)}px")

            if afstand_tot_andere < 150:
                print(f"{self.color.upper()} ontwijkt andere auto ({int(afstand_tot_andere)}px)")

                # Bereken hoek weg van de andere auto
                dx = x - ox
                dy = y - oy
                weg_hoek = math.degrees(math.atan2(dy, dx)) % 360
                huidige_hoek = angle % 360
                hoekverschil = (weg_hoek - huidige_hoek + 540) % 360 - 180

                # Kies de juiste ontwijkrichting
                if abs(hoekverschil) > 20:
                    direction = "right" if hoekverschil > 0 else "left"
                else:
                    direction = "forward"

                print(f"Uitwijkrichting: {direction.upper()}")
                self.send_command(direction)

                # Probeer na 1.5 seconde opnieuw te navigeren
                threading.Timer(1.5, self.navigate).start()
                return

        # Bereken navigatiehoek richting doel
        dx, dy = tx - x, ty - y
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        current_angle = angle % 360
        angle_diff = (target_angle - current_angle + 540) % 360 - 180

        # Toon statusinformatie
        print(f"\n{self.color.upper()} Auto:")
        print(f"Positie: ({x}, {y}) | Hoek: {int(angle)} graden")
        print(f"Doel: ({tx}, {ty}) | Afstand: {int(distance)} pixels | Richting: {int(target_angle)} graden")
        print(f"Hoekverschil: {int(angle_diff)} graden")

        # Bepaal actie richting doel
        if distance < 150:
            command = "stop"
            print("Doel bereikt")
        elif abs(angle_diff) > 20:
            command = "right" if angle_diff > 0 else "left"
            print(f"Draai {command.upper()}")
        else:
            command = "forward"
            print("Rijd vooruit")

        self.send_command(command)

    def send_command(self, action):
        # Verstuur commando alleen als het anders is dan het vorige
        if action != self.last_command:
            topic = f"cars/control/{self.color}"
            self.client.publish(topic, action)
            self.last_command = action
            print(f"Verzonden naar {topic}: {action}\n")
        else:
            print(f"Zelfde commando '{action}' wordt niet opnieuw verzonden.")

def main():
    # Check of kleurargument is meegegeven
    if len(sys.argv) != 2:
        print("Gebruik: python car_client.py <kleur>")
        sys.exit(1)

    color = sys.argv[1].lower()
    if color not in ['red', 'blue']:
        print("Ongeldige kleur. Kies uit: red, blue")
        sys.exit(1)

    # MQTT-client initialiseren
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    car = CarController(color, client)
    client.on_message = car.on_message

    try:
        # Verbinden met MQTT-broker
        client.connect("172.20.10.2", 1883, 60)
    except:
        print(f"{color.upper()}: Kan geen verbinding maken met MQTT broker.")
        sys.exit(1)

    # Topics abonneren voor beide auto's en het doel
    other_color = 'red' if color == 'blue' else 'blue'
    client.subscribe(f"cars/orientation/{color}")
    client.subscribe(f"cars/orientation/{other_color}")
    client.subscribe("drone/block")

    print(f"Auto-controller gestart voor kleur: {color.upper()}...")
    client.loop_forever()

if __name__ == "__main__":
    main()