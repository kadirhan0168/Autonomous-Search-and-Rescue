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
        self.control_topic = 'cars/control/red/'
        self.orientation_topic = 'cars/orientation/red'
        self.target_topic = 'drone/block'
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.last_command = "stop"
        
        # Path planning variables
        self.position = (0, 0, 0)  # x, y, angle
        self.target = None
        self.avoiding = False
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('motor1')
        self.right_motor = self.robot.getDevice('motor2')
        self.top_motor = self.robot.getDevice('motor3')
        
        # Configure motors
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        self.top_motor.setPosition(0.0)         # Start at center position
        self.angles = [0.5, 0.0, -0.5, 0.0]     # Target angles in radians
        self.scan_speed = 1.0                   # radians per second
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
            client.subscribe(self.target_topic)
            
        def on_message(client, userdata, msg):
            try:
                if msg.topic == self.control_topic:
                    command = msg.payload.decode().strip().lower()

                    # Only process if the command is different from the last one
                    if command != self.last_command:
                        self.last_command = command
                        print(f"Received new command: {self.last_command}")
                        if self.last_command == "stop":
                            print("Stop received, pausing")
                            time.sleep(2.5)
                    else:
                        print(f"Ignored duplicate command: {command}")
            except Exception as e:
                print(f"Error processing MQTT message: {e}")
            
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.connect(self.mqtt_broker)
        self.client.loop_start()
    
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
        else:
            print(f"Unknown command: {self.last_command}")
            self.stop()

    def run(self):
        while self.robot.step(self.time_step) != -1:
            distance = self.get_distance()
            if distance < 20 and not self.avoiding:
                self.avoid_obstacle()
            else:
                self.execute_command()

# Launch the controller
controller = ChariotMQTTController()
controller.run()