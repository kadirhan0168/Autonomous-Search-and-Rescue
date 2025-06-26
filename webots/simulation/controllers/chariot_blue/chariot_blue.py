from controller import Robot, DistanceSensor, Motor
import paho.mqtt.client as mqtt
import time
import json
import math

class ChariotMQTTController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # MQTT settings
        self.mqtt_broker = '172.20.10.2'
        self.control_topic = 'cars/control/blue/'
        self.target_topic = 'drone/block'
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.last_command = None
        
        # Path planning variables
        self.position = (0, 0, 0)  # x, y, angle
        self.target = None
        self.avoiding = False
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('motor1')
        self.right_motor = self.robot.getDevice('motor2')
        self.top_motor = self.robot.getDevice('motor3')  # Servo motor
        
        # Configure motors
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        # Motor3 (servo) configuration
        self.top_motor.setPosition(0.0)  # Start at center position
        self.angles = [0.5, 0.0, -0.5, 0.0]  # Target angles in radians
        self.scan_speed = 1.0  # radians per second
        self.current_target_index = 0
        self.target_angle = self.angles[self.current_target_index]
        self.current_position = 0.0
        
        # Movement parameters
        self.wheel_velocity = 2.0
        
        # Initialize distance sensor
        self.distance_sensor = self.robot.getDevice('distance_sensor')
        self.distance_sensor.enable(self.time_step)
        
        self.connect_mqtt()
        
    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            print("Connected to MQTT Broker with result code "+str(rc))
            client.subscribe(self.control_topic)
            # client.subscribe(self.orientation_topic)
            client.subscribe(self.target_topic)
            
        def on_message(client, userdata, msg):
            try:
                if msg.topic == self.control_topic:
                    self.last_command = msg.payload.decode().strip().lower()
                    print(f"Received command: {self.last_command}")
                # elif msg.topic == self.orientation_topic:
                    # data = json.loads(msg.payload.decode())
                    # self.position = (data['x'], data['y'], data['angle'])
                # elif msg.topic == self.target_topic:
                    # data = json.loads(msg.payload.decode())
                    # self.target = (data['x'], data['y'])
                    # self.navigate()
            except Exception as e:
                print(f"Error processing MQTT message: {e}")
            
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.connect(self.mqtt_broker)
        self.client.loop_start()
    
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

        # print(f"\nPath Planning:")
        # print(f"Pos: ({int(x)}, {int(y)}) | Target: ({int(tx)}, {int(ty)})")
        # print(f"Distance: {int(distance)}px | angle diff: {int(angle_diff)}°")

        if distance < 300:
            self.last_command = "stop"
            print("🎯 Target Reached!")
        elif abs(angle_diff) > 20:
            self.last_command = "right" if angle_diff > 0 else "left"
        else:
            self.last_command = "forward"
    
    def update_servo_position(self):
        """Update servo position using the exact movement pattern from the example"""
        dt = self.time_step / 1000.0  # convert timestep to seconds
        
        # Move current_position towards target_angle by speed * dt
        if abs(self.target_angle - self.current_position) < self.scan_speed * dt:
            # Close enough to target, snap to target and pick next target
            self.current_position = self.target_angle
            self.current_target_index = (self.current_target_index + 1) % len(self.angles)
            self.target_angle = self.angles[self.current_target_index]
        else:
            # Move smoothly toward the target
            if self.target_angle > self.current_position:
                self.current_position += self.scan_speed * dt
            else:
                self.current_position -= self.scan_speed * dt

        self.top_motor.setPosition(self.current_position)
    
    def get_distance(self):
        return self.distance_sensor.getValue() * 100  # in cm

    def stop(self):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def forward(self):
        self.left_motor.setVelocity(self.wheel_velocity)
        self.right_motor.setVelocity(self.wheel_velocity)

    def left(self):
        self.left_motor.setVelocity(self.wheel_velocity)
        self.right_motor.setVelocity(self.wheel_velocity * 2.5)

    def right(self):
        self.left_motor.setVelocity(self.wheel_velocity * 2.5)
        self.right_motor.setVelocity(self.wheel_velocity)

    def execute_command(self):
        if self.last_command == 'forward':
            self.forward()
        elif self.last_command == 'left':
            self.left()
        elif self.last_command == 'right':
            self.right()
        elif self.last_command == 'stop':
            self.stop()
        elif self.last_command == None:
            pass
        else:
            print(f"Unknown command: {self.last_command}")
            self.stop()
    
    def avoid_obstacle(self):
        self.avoiding = True
        print("🚧 Obstacle detected! Scanning for alternative path...")
        self.stop()

        # Store current servo state
        saved_position = self.current_position
        saved_target = self.target_angle
        saved_index = self.current_target_index

        # Move to right scan position (0.5 radians ≈ 28.6 degrees right)
        self.target_angle = 0.5
        while abs(self.current_position - 0.5) > 0.01 and self.robot.step(self.time_step) != -1:
            self.update_servo_position()
        right_dist = self.get_distance()
        print(f"📏 Right: {right_dist:.1f} cm")

        # Move to left scan position (-0.5 radians ≈ 28.6 degrees left)
        self.target_angle = -0.5
        while abs(self.current_position - (-0.5)) > 0.01 and self.robot.step(self.time_step) != -1:
            self.update_servo_position()
        left_dist = self.get_distance()
        print(f"📏 Left: {left_dist:.1f} cm")

        # Return to center
        self.target_angle = 0.0
        while abs(self.current_position) > 0.01 and self.robot.step(self.time_step) != -1:
            self.update_servo_position()

        # Decide
        if right_dist > left_dist:
            print("🔀 Avoiding Right")
            self.right()
        else:
            print("🔀 Avoiding Left")
            self.left()

        # Move for 1 second
        for _ in range(int(0.5 * 1000 / self.time_step)):
            if self.robot.step(self.time_step) == -1: break
            
        self.stop()
        
        # Restore scanning motion
        self.current_target_index = saved_index
        self.target_angle = saved_target
        self.current_position = saved_position
        self.avoiding = False

    def run(self):
        while self.robot.step(self.time_step) != -1:
            distance = self.get_distance()
            # print(f"📡 Distance: {distance:.1f} cm")
            # print("chariot running")
            if distance < 20 and not self.avoiding:
                self.avoid_obstacle()
            else:
                # if not self.avoiding:
                    # self.update_servo_position()  # Continuous scanning when not avoiding
                self.execute_command()

# Launch the controller
controller = ChariotMQTTController()
controller.run()