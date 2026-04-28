"""
Example: Multi-axis motor control with binary packets.

This demonstrates how to control multiple motors (axes) separately
using binary packet protocol.
"""
from re import M
import time
from motor_driver import Controller, MotorControl
# Create driver instance (shared for all axes)

driver = Controller(host="192.168.10.86", port=4949, timeout=5.0)

if driver.connect():
    print("Connected to motor driver")
    
    # Create motor control instances for different axes
    # Axis 1
    motor1 = MotorControl(
        driver=driver,
        axis_number=1,  # First axis
        max_speed=20,
        position_units="degrees"
    )
    
    # Axis 2
    motor2 = MotorControl(
        driver=driver,
        axis_number=2,  # Second axis
        max_speed=20,
        position_units="degrees"
    )
    
    print("\n=== Start Test ===")
    # Enable axis 1
    print("Setting camera zoom to 10")
    motor1.set_camera_zoom(1)
    print("Setting camera resolution to 1280x720")
    motor1.set_camera_resolution((1280, 720))
    print("Setting camera FOV to 41.5x23.5")
    motor1.set_camera_fov((41.5, 23.5))
    motor1.set_tracking_status(1)
    print("Setting stream data pixels to target_id=0, classification=0, confidance=0, x1=10, y1=10, x2=20, y2=20")
    motor1.set_camera_kp(6)
    motor1.set_camera_ki(2)
    motor1.set_camera_kd(1)
    motor1.set_camera_acceleration(10)
    motor1.set_movement_mode('position')

