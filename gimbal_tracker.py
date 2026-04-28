"""
Gimbal Object Tracker

Tracks an object detected in a camera frame by commanding a 2-axis gimbal
(pan / tilt) so the object stays centered.  Uses proportional control:
gimbal speed is proportional to the pixel error from the frame center.

Usage:
    Call tracker.track(object_x, object_y) once per frame from your own
    camera / detection loop.
"""

import time
from typing import Optional, Tuple
from dataclasses import dataclass
import struct
from motor_driver import Controller, MotorControl


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class TrackerConfig:
    """Tunable parameters for gimbal tracking behaviour."""

    # Camera frame dimensions (pixels). Used to compute the center point.
    frame_width: int = 1280
    frame_height: int = 720

    # Proportional gain: degrees/s of gimbal speed per pixel of error.
    # Higher values make the correction more aggressive.
    pan_gain: float = 0.05
    tilt_gain: float = 0.05

    # Deadzone: errors smaller than this (pixels) are ignored to prevent
    # jitter when the object is nearly centred.
    deadzone_pixels: float = 10.0

    # Maximum tracking speed (degrees/s).  Output is clamped to this.
    max_pan_speed: float = 3
    max_tilt_speed: float = 3

    # Speeds below this threshold are rounded to zero so the motor
    # does not creep or fight friction.
    min_speed_threshold: float = 0.3

    # Direction inversion.  Set to -1 if the gimbal moves opposite to
    # the expected direction for a given axis.
    invert_pan: int = 1    # 1 or -1
    invert_tilt: int = 1   # 1 or -1


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------
class GimbalTracker:
    """
    Proportional-control tracker for a 2-axis gimbal.

    Each call to ``track(object_x, object_y)`` computes the pixel error
    from the frame centre and converts it to a speed command for pan and
    tilt.  When the object is centred the speed goes to zero.
    """

    def __init__(
        self,
        controller: Controller,
        pan_axis: int = 1,
        tilt_axis: int = 2,
        config: Optional[TrackerConfig] = None,
    ):
        """
        Args:
            controller: Shared Controller (TCP connection) for the gimbal.
            pan_axis:   Axis number for horizontal / pan motor.
            tilt_axis:  Axis number for vertical / tilt motor.
            config:     TrackerConfig instance.  Defaults are used when None.
        """
        self.controller = controller
        self.config = config or TrackerConfig()

        self.pan_motor = MotorControl(controller, axis_number=pan_axis)
        self.tilt_motor = MotorControl(controller, axis_number=tilt_axis)

        self._center_x: float = 0.0
        self._center_y: float = 0.0
        self._recompute_center()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def setup(self) -> None:
        """Enable both axes and switch them to speed mode."""
        self.pan_motor.axis_on()
        self.tilt_motor.axis_on()
        self.pan_motor.set_movement_mode("speed")
        self.tilt_motor.set_movement_mode("speed")
        self.pan_motor.set_speed(20) #degrees/s
        self.tilt_motor.set_speed(20) #degrees/s    
        self.pan_motor.set_acceleration(2000) #degrees/s^2
        self.tilt_motor.set_acceleration(2000) #degrees/s^2

    def set_frame_size(self, width: int, height: int) -> None:
        """Update frame dimensions (e.g. when camera resolution changes)."""
        self.config.frame_width = width
        self.config.frame_height = height
        self._recompute_center()

    # ------------------------------------------------------------------
    # Core tracking
    # ------------------------------------------------------------------
    def track(
        self,
        object_x: float,
        object_y: float,
    ) -> Tuple[float, float]:
        """
        Command the gimbal to centre the object in the frame.

        Call this once per frame with the (x, y) pixel coordinates of the
        detected object's centre.

        Args:
            object_x: Horizontal pixel coordinate of the object centre.
            object_y: Vertical pixel coordinate of the object centre.

        Returns:
            (pan_speed, tilt_speed) actually commanded, in degrees/s.
        """
        cfg = self.config

        # --- pixel error from frame centre ---
        err_x = object_x - self._center_x
        err_y = object_y - self._center_y

        # --- deadzone ---
        if abs(err_x) < cfg.deadzone_pixels:
            err_x = 0.0
        if abs(err_y) < cfg.deadzone_pixels:
            err_y = 0.0

        # --- proportional control ---
        pan_speed = cfg.pan_gain * err_x * cfg.invert_pan
        tilt_speed = cfg.tilt_gain * err_y * cfg.invert_tilt

        # --- clamp to max speed ---
        pan_speed = max(-cfg.max_pan_speed, min(cfg.max_pan_speed, pan_speed))
        tilt_speed = max(-cfg.max_tilt_speed, min(cfg.max_tilt_speed, tilt_speed))

        # --- zero out tiny speeds ---
        if abs(pan_speed) < cfg.min_speed_threshold:
            pan_speed = 0.0
        if abs(tilt_speed) < cfg.min_speed_threshold:
            tilt_speed = 0.0

        # --- send to gimbal ---
        '''
        self.pan_motor.set_speed(pan_speed)
        self.pan_motor.update()
        self.tilt_motor.set_speed(tilt_speed)
        self.tilt_motor.update()
        '''
        #Simultanelly 2 axis moveing
        packed_bytes_axis_1 = list(struct.pack('>h', int(pan_speed) * 100))
        packed_bytes_axis_2 = list(struct.pack('>h', int(tilt_speed) * 100))    

        packet = [
            0x50, 0x54,           # Header
            0x08,                 # Length (data portion after header)
            0x00,                 # Type/Flag
            0x00,                 # Movement mode (Always)
            0x07,                 # Command type_1
            0x26,                 # Command type_2
        ]
        packet.extend(packed_bytes_axis_1) #speed_axis_1
        packet.extend(packed_bytes_axis_2) #speed_axis_2    
        packet.append(sum(packet[2:]) % 256)
        self.controller.socket.sendall(bytes(packet))

        return pan_speed, tilt_speed

    # ------------------------------------------------------------------
    # Stop / teardown
    # ------------------------------------------------------------------
    def stop(self) -> None:
        """Set both axes to zero speed and send update."""
        '''
        self.pan_motor.set_speed(0.0)
        self.pan_motor.update()
        self.tilt_motor.set_speed(0.0)
        self.tilt_motor.update()
        ''' 
        packet = [
        0x50, 0x54, # Header
        0x08,       # Length (data portion after header)
        0x00,       # Type/Flag
        0x00,       # Movement mode (Always)
        0x07,       # Command type_1
        0x26,       # Command type_2
        0x00, 0x00, #pan speed
        0x00, 0x00, #tilt speed
        0x35        #Checksum
        ]

        self.controller.socket.sendall(bytes(packet))
        '''
        # Send update command
        packet = [
            0x50, 0x54,           # Header
            0x04,                 # Length (data portion after header)
            0x00,                 # Type/Flag
            0x01,                 # Movement mode (Always)
            0x01,                 # Command type
            0x34,                 # OPcode Low
            0x3A,                 # Checksum
        ]
        self.controller.socket.sendall(bytes(packet))
        '''
        return 
    
    def disable(self) -> None:
        """Stop movement then disable both motors."""
        self.stop()
        self.pan_motor.axis_off()
        self.tilt_motor.axis_off()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _recompute_center(self) -> None:
        self._center_x = self.config.frame_width / 2.0
        self._center_y = self.config.frame_height / 2.0


# ---------------------------------------------------------------------------
# Convenience wrapper
# ---------------------------------------------------------------------------
def track_object(
    tracker: GimbalTracker,
    object_x: float,
    object_y: float,
) -> Tuple[float, float]:
    """
    Convenience function: forward (x, y) to an existing GimbalTracker.

    Args:
        tracker:  An initialised GimbalTracker instance.
        object_x: Horizontal pixel coordinate of the object centre.
        object_y: Vertical pixel coordinate of the object centre.

    Returns:
        (pan_speed, tilt_speed) in degrees/s.
    """
    return tracker.track(object_x, object_y)


# ---------------------------------------------------------------------------
# Example
# ---------------------------------------------------------------------------
def example_tracking_loop():
    """
    Demonstrates GimbalTracker with simulated object coordinates.

    Replace the simulated (x, y) values with real detections from your
    camera / detection pipeline (OpenCV, YOLO, etc.).
    """
    config = TrackerConfig(
        frame_width=1920,
        frame_height=1080,
        pan_gain=0.03,
        tilt_gain=0.03,
        deadzone_pixels=15.0,
        max_pan_speed=12.0,
        max_tilt_speed=12.0,
    )

    controller = Controller(host="192.168.10.120", port=4949, timeout=5.0)
    if not controller.connect():
        print("Failed to connect to controller")
        return

    tracker = GimbalTracker(controller, pan_axis=1, tilt_axis=2, config=config)
    tracker.setup()

    center_x = config.frame_width / 2
    center_y = config.frame_height / 2

    try:
        for i in range(10):
            # Simulated object drifting around the centre
            obj_x = center_x + 50 * (i % 3 - 1)   # -50, 0, +50
            obj_y = center_y + 30 * (i % 2)        #   0, +30

            pan, tilt = tracker.track(obj_x, obj_y)
            print(
                f"[{i}] object=({obj_x:.0f}, {obj_y:.0f})  "
                f"pan={pan:+.2f} deg/s   tilt={tilt:+.2f} deg/s"
            )
            time.sleep(0.5)
    finally:
        tracker.stop()
        tracker.disable()
        controller.disconnect()
        print("Tracker shut down.")


if __name__ == "__main__":
    example_tracking_loop()
