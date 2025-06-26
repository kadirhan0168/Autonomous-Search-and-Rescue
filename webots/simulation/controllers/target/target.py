from controller import Supervisor
import paho.mqtt.client as mqtt
import threading
import json

# === Config ===
WORLD_SIZE = 3.0  # Arena is 3x3m
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
TARGET_HEIGHT = 0.0  # Now using Z for height (0 = ground level)

# === Shared State ===
target_coords = {"px": 0.0, "py": 0.0}
coords_lock = threading.Lock()

# === Pixel to World Conversion ===
def pixel_to_webots(pixel_x, pixel_y):
    """Convert pixel to Webots coordinates with correct axis mapping"""
    # X (left-right) remains the same
    x = (pixel_x / IMAGE_WIDTH) * WORLD_SIZE
    # Y becomes depth (into screen), inverted from image coordinates
    y = (1 - (pixel_y / IMAGE_HEIGHT) * WORLD_SIZE)
    return x, y

# === MQTT Callback ===
def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        px = float(payload["x"])
        py = float(payload["y"])
        with coords_lock:
            target_coords["px"] = px
            target_coords["py"] = py
        print(f"Received target pixel ({px}, {py}) → Webots {pixel_to_webots(px, py)}")
    except Exception as e:
        print(f"Invalid MQTT message: {e}")

# === MQTT Thread ===
def mqtt_thread():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect("localhost", 1883)
    client.subscribe("drone/block")
    client.loop_forever()

# Start MQTT in background
threading.Thread(target=mqtt_thread, daemon=True).start()

# === Webots Supervisor Setup ===
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

target_node = supervisor.getFromDef("TARGET")
if not target_node:
    print("ERROR: DEF TARGET not found!")
    exit(1)

translation_field = target_node.getField("translation")

# === Main loop ===
while supervisor.step(timestep) != -1:
    with coords_lock:
        px = target_coords["px"]
        py = target_coords["py"]
    
    x, y = pixel_to_webots(px, py)
    
    # Correct coordinate mapping:
    # X = x (left-right)
    # Y = y (depth) 
    # Z = height (0 for ground level)
    translation_field.setSFVec3f([x, y, TARGET_HEIGHT])