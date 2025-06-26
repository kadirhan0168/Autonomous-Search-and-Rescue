from controller import Robot
import math

class CrazyflieAltitudeController:
    def __init__(self):
        # Initialize robot and get motor devices
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera(1)")  # Replace with actual name
        self.camera.enable(self.timestep)
        
        # Motor initialization
        self.motors = [
            self.robot.getDevice('m1_motor'),
            self.robot.getDevice('m2_motor'),
            self.robot.getDevice('m3_motor'),
            self.robot.getDevice('m4_motor')
        ]
        
        # Configure motors
        for motor in self.motors:
            motor.setPosition(float('inf'))  # Velocity control mode
            motor.setVelocity(0)  # Start with zero velocity
        
        # Sensor setup
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        
        # PID constants for altitude control
        self.kp = 12.0  # Proportional gain
        self.ki = 3.0   # Integral gain
        self.kd = 8.0    # Derivative gain
        
        # Control variables
        self.target_altitude = 1.0  # meters
        self.integral_error = 0.0
        self.previous_error = 0.0
        # self.base_thrust = 55.36808  # Hover thrust value
        self.base_thrust = 48.0  # Hover thrust value
        
    def run(self):
        while self.robot.step(self.timestep) != -1:
            # Get current altitude from GPS
            current_position = self.gps.getValues()
            x, y, z = current_position
            current_altitude = z  # Webots uses Z as altitude

            
            # Calculate altitude error
            error = self.target_altitude - current_altitude
            
            # PID calculations
            self.integral_error += error * (self.timestep / 1000.0)
            derivative_error = (error - self.previous_error) / (self.timestep / 1000.0)
            
            # Calculate PID output (thrust adjustment)
            pid_output = (self.kp * error + 
                         self.ki * self.integral_error + 
                         self.kd * derivative_error)
            
            # Apply limits to PID output
            pid_output = max(min(pid_output, 20.0), -20.0)  # Limit to ±20
            
            # Calculate motor velocities
            thrust = self.base_thrust + pid_output
            
            # Set motor velocities (motor mixing for Crazyflie)
            # Motor order: m1 (front left), m2 (front right), 
            # m3 (rear left), m4 (rear right)
            self.motors[0].setVelocity(-thrust)  # m1 (CCW)
            self.motors[1].setVelocity(thrust)   # m2 (CW)
            self.motors[2].setVelocity(-thrust)  # m3 (CCW)
            self.motors[3].setVelocity(thrust)   # m4 (CW)
            
            # Print current altitude for monitoring
            print(f"GPS Position -> X: {x:.2f}, Y: {y:.2f}, Z (altitude): {z:.2f} | Target Z: {self.target_altitude:.2f} | Thrust: {thrust:.2f}")
  
            # Store error for next derivative calculation
            self.previous_error = error

# Create and run the controller
if __name__ == "__main__":
    controller = CrazyflieAltitudeController()
    controller.run()