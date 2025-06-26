from controller import Robot, Motor, InertialUnit, GPS, Gyro, Camera, DistanceSensor
from math import cos, sin, sqrt
import paho.mqtt.client as mqtt
import io
import json
from PIL import Image
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ALTITUDE = 3.0
WAYPOINT_TOLERANCE = 0.2
STABILIZATION_TIME = 2.0

class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

target_found = False
target_found_time = None
target_coordinates = None
width = 1  # default fallback
height = 1

def pixel_to_webots(pixel_x, pixel_y, image_width, image_height, world_size=3.0):
    x = (pixel_x / image_width) * world_size
    y = (1 - (pixel_y / image_height)) * world_size
    return x, y

def on_message(client, userdata, msg):
    global target_found, target_coordinates, width, height

    if msg.topic == "drone/rescue":
        if msg.payload.decode().lower() == "true":
            target_found = True
            print("Rescue triggered: true")

    elif msg.topic == "drone/block":
        try:
            payload = json.loads(msg.payload.decode())
            pixel_x = float(payload["x"])
            pixel_y = float(payload["y"])
            x_world, y_world = pixel_to_webots(pixel_x, pixel_y, width, height)
            target_coordinates = Waypoint(x_world, y_world)
            print(f"Received pixel ({pixel_x}, {pixel_y}) → Webots ({x_world:.2f}, {y_world:.2f})")
        except Exception as e:
            print("Invalid JSON from drone/block:", msg.payload.decode(), e)

if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect("localhost", 1883)
    client.on_message = on_message
    client.subscribe("drone/rescue")
    client.subscribe("drone/block")
    client.loop_start()

    motors = [robot.getDevice(f"m{i}_motor") for i in range(1, 5)]
    for i, m in enumerate(motors):
        m.setPosition(float('inf'))
        m.setVelocity(-1 if i % 2 == 0 else 1)

    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    camera = robot.getDevice("camera(1)")
    camera.enable(timestep)
    width = camera.getWidth()
    height = camera.getHeight()

    for name in ["range_front", "range_left", "range_back", "range_right"]:
        robot.getDevice(name).enable(timestep)

    past_x_global = 0
    past_y_global = 0
    past_time = robot.getTime()

    PID_CF = pid_velocity_fixed_height_controller()
    height_desired = FLYING_ALTITUDE

    redirected = False
    frame_count = 0

    while robot.step(timestep) != -1:
        if robot.getTime() > STABILIZATION_TIME:
            break

    print("Starting control loop...")

    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time

        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        y_global = gps.getValues()[1]
        v_x_global = (x_global - past_x_global) / dt
        v_y_global = (y_global - past_y_global) / dt

        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0

        if target_found and target_coordinates:
            if not redirected:
                print(f"Redirecting to coordinates ({target_coordinates.x:.2f}, {target_coordinates.y:.2f})")
                redirected = True
                target_found_time = robot.getTime()
                height_desired = FLYING_ALTITUDE

            dx = target_coordinates.x - x_global
            dy = target_coordinates.y - y_global
            distance = sqrt(dx*dx + dy*dy)

            if distance > 0.05:
                vx_desired = 0.5 * dx / distance
                vy_desired = 0.5 * dy / distance
            else:
                vx_desired = 0
                vy_desired = 0

            forward_desired = vx_desired * cosyaw + vy_desired * sinyaw
            sideways_desired = -vx_desired * sinyaw + vy_desired * cosyaw

        motor_power = PID_CF.pid(
            dt, forward_desired, sideways_desired, yaw_desired,
            height_desired, roll, pitch, yaw_rate,
            altitude, v_x, v_y
        )

        motors[0].setVelocity(-motor_power[0])
        motors[1].setVelocity(motor_power[1])
        motors[2].setVelocity(-motor_power[2])
        motors[3].setVelocity(motor_power[3])

        frame_count += 1
        if frame_count % 30 == 0:
            image = camera.getImage()
            img = Image.frombytes("RGBA", (width, height), image).convert("RGB")
            buffer = io.BytesIO()
            img.save(buffer, format='JPEG')
            client.publish("drone_sim/camera", buffer.getvalue())

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
