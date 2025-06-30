import network
import time
from machine import Pin, PWM
from simply import MQTTClient  # Import MQTT client for communication

ssid = 'Mohamed Elshakh'  # WiFi name
password = 'mohamed1'     # WiFi password

mqtt_broker = '172.20.10.2'  # MQTT broker IP
mqtt_topic = 'cars/control/red/'  # Topic for control commands
mqtt_status_topic = 'cars/status/red'  # Topic for status updates

PWM_LM = 6
PWM_RM = 7
left_motor = PWM(Pin(PWM_LM))  # Left motor pin
right_motor = PWM(Pin(PWM_RM))  # Right motor pin
left_motor.freq(50)
right_motor.freq(50)

servo = PWM(Pin(10))  # Servo motor pin
servo.freq(50)

def set_servo_angle(angle):
    duty = int((angle / 180) * 8000 + 1000)
    servo.duty_u16(duty)
    time.sleep(0.5)

trig = Pin(27, Pin.OUT)  # Trigger pin for ultrasonic sensor
echo = Pin(26, Pin.IN, Pin.PULL_DOWN)  # Echo pin for ultrasonic sensor

def measure_distance():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    timeout = 1000000  # Timeout value in microseconds
    start = time.ticks_us()

    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > timeout:
            return 999  # Timeout: no echo received
    pulse_start = time.ticks_us()

    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), pulse_start) > timeout:
            return 999  # Timeout while waiting for end of echo
    pulse_end = time.ticks_us()

    duration = time.ticks_diff(pulse_end, pulse_start)
    distance = duration * 0.0343 / 2  # Convert to cm
    return round(distance, 0)

fled = Pin(20, Pin.OUT)  # Front LED
bled = Pin(21, Pin.OUT)  # Back LED
fled.value(1)  # Turn on front LED
bled.value(1)  # Turn on back LED

def stop_motors():
    left_motor.duty_u16(5000)
    right_motor.duty_u16(5000)

def move_forward():
    left_motor.duty_u16(4550)
    right_motor.duty_u16(5150)

def turn_left():
    left_motor.duty_u16(4600)
    right_motor.duty_u16(4600)

def turn_right():
    left_motor.duty_u16(5100)
    right_motor.duty_u16(5100)

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

last_command = "stop"  # Default state

def handle_message(topic, msg):
    global last_command
    cmd = msg.decode().strip().lower()
    print("Received:", cmd)
    last_command = cmd

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

if connect_to_wifi():
    client = MQTTClient("red", mqtt_broker)
    client.set_callback(handle_message)
    client.connect()
    client.subscribe(mqtt_topic)
    print(f"🚗 Listening on topic: {mqtt_topic}")
    set_servo_angle(55)

    try:
        while True:
            distance = measure_distance()  # Read distance from ultrasonic sensor

            client.check_msg()             # Check for incoming MQTT messages
            execute_command()             # Execute the last received movement command

            if 10 <= distance <= 15:
                print("Obstacle detected! Scanning...")
                stop_motors()
                client.publish(mqtt_status_topic, "stop")
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
                    client.publish(mqtt_topic, "right")
                    turn_right()
                    time.sleep(1.1)
                    client.publish(mqtt_topic, "forward")
                    move_forward()
                    time.sleep(1.9)
                else:
                    print("Decision: Turn Left")
                    client.publish(mqtt_topic, "left")
                    turn_left()
                    time.sleep(1.1)
                    client.publish(mqtt_topic, "forward")
                    move_forward()
                    time.sleep(1.9)
            else:
                client.check_msg()
                execute_command()

    except Exception as e:
        print("Error:", e)
        client.disconnect()
        stop_motors()
