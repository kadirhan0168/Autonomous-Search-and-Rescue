import network
import time
from machine import Pin, PWM
from simply import MQTTClient  # Import MQTT client for communication

# WiFi credentials
ssid = 'Mohamed Elshakh'
password = 'mohamed1'

# MQTT broker and topic
mqtt_broker = '172.20.10.2'
mqtt_topic = 'cars/control/blue/'

# Motor PWM setup
PWM_LM = 6
PWM_RM = 7
left_motor = PWM(Pin(PWM_LM))
right_motor = PWM(Pin(PWM_RM))
left_motor.freq(50)
right_motor.freq(50)

# Servo motor for directional scanning
servo = PWM(Pin(10))
servo.freq(50)

# Set servo to a given angle
def set_servo_angle(angle):
    duty = int((angle / 180) * 8000 + 1000)
    servo.duty_u16(duty)
    time.sleep(0.5)

# Setup pins for ultrasonic sensor
trig = Pin(27, Pin.OUT)
echo = Pin(26, Pin.IN, Pin.PULL_DOWN)

# Measure distance using ultrasonic sensor
def measure_distance():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    while echo.value() == 0:
        pulse_start = time.ticks_us()
    while echo.value() == 1:
        pulse_end = time.ticks_us()
    duration = time.ticks_diff(pulse_end, pulse_start)
    distance = duration * 0.0343 / 2
    return round(distance, 0)

# Setup LEDs (optional)
fled = Pin(20, Pin.OUT)
bled = Pin(21, Pin.OUT)
# fled.value(1)
# bled.value(1)

# Stop motors
def stop_motors():
    left_motor.duty_u16(5000)
    right_motor.duty_u16(5000)

# Move forward
def move_forward():
    left_motor.duty_u16(4500)
    right_motor.duty_u16(4600)

# Turn left
def turn_left():
    left_motor.duty_u16(4500)
    right_motor.duty_u16(5150)

# Turn right
def turn_right():
    left_motor.duty_u16(5300)
    right_motor.duty_u16(4600)

# Connect to WiFi
def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    print("Connecting to WiFi...")
    for _ in range(20):
        if wlan.isconnected():
            print("Connected to WiFi:", wlan.ifconfig()[0])
            return True
        time.sleep(0.5)
    print("WiFi connection failed.")
    return False

# Handle received MQTT messages
last_command = "stop"
def handle_message(topic, msg):
    global last_command
    cmd = msg.decode().strip().lower()
    print("Received:", cmd)
    last_command = cmd

# Execute stored command
def execute_command():
    if last_command == 'forward':
        move_forward()
    elif last_command == 'left':
        turn_left()
    elif last_command == 'right':
        turn_right()
    elif last_command == 'stop':
        stop_motors()
    else:
        print("Unknown command:", last_command)
        stop_motors()

# Main loop
if connect_to_wifi():
    client = MQTTClient("pico_auto", mqtt_broker)
    client.set_callback(handle_message)
    client.connect()
    client.subscribe(mqtt_topic)
    print(f"🚗 Listening on topic: {mqtt_topic}")
    set_servo_angle(55)

    try:
        while True:
            distance = measure_distance()

            client.check_msg()
            execute_command()

            if 10 <= distance <= 18:
                print("Obstacle detected! Scanning environment...")
                stop_motors()
                client.publish(mqtt_topic, "stop")

                set_servo_angle(25)
                time.sleep(0.5)
                right_distance = measure_distance()
                print("Right:", right_distance, "cm")

                set_servo_angle(85)
                time.sleep(0.5)
                left_distance = measure_distance()
                print("Left:", left_distance, "cm")

                set_servo_angle(55)
                time.sleep(0.3)

                if right_distance > left_distance:
                    print("Decision: Turn Right")
                    turn_right()
                    client.publish(mqtt_topic, "right")
                    time.sleep(1.3)
                    move_forward()
                    client.publish(mqtt_topic, "forward")
                    time.sleep(2)
                else:
                    print("Decision: Turn Left")
                    turn_left()
                    client.publish(mqtt_topic, "left")
                    time.sleep(1.3)
                    move_forward()
                    client.publish(mqtt_topic, "forward")
                    time.sleep(2)
            else:
                client.check_msg()
                execute_command()

    except Exception as e:
        print("Error:", e)
        client.disconnect()
        stop_motors()
