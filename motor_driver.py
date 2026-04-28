"""
Ethernet Motor Driver Control Module

This module provides TCP/IP communication with motor drivers for controlling
speed, position, and reading encoder values. The implementation is protocol-agnostic
and can be adapted to specific driver requirements.
"""

import socket
import struct
import time
import threading
from typing import Optional, Union, Callable, Dict, Any


class Controller:
    """
    Low-level TCP/IP communication interface for controller.
    
    This class handles the socket connection and provides methods for sending
    commands and receiving responses. Protocol-specific formatting can be
    customized via callback functions.
    """
    
    def __init__(self, host: str = "192.168.10.120", port: int = 4949, 
                 timeout: float = 5.0, auto_reconnect: bool = True):
        """
        Initialize the controller connection.
        
        Args:
            host: IP address of the controller
            port: TCP port number (default 4949 for controller)
            timeout: Connection timeout in seconds
            auto_reconnect: Enable automatic reconnection on disconnect
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Protocol customization callbacks
        self.command_formatter: Optional[Callable] = None
        self.response_parser: Optional[Callable] = None
        
    def connect(self) -> bool:
        """
        Establish TCP connection to the controller.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            if self.connected:
                return True
                
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to controller at {self.host}:{self.port}")
            return True
            
        except socket.timeout:
            print(f"Connection timeout to controller at {self.host}:{self.port}")
            self.connected = False
            return False
        except socket.error as e:
            print(f"Connection error: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"Unexpected error during connection: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Close the TCP connection."""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                finally:
                    self.socket = None
                    self.connected = False
                    print("Disconnected from motor driver")
    
    def is_connected(self) -> bool:
        """Check if currently connected to the driver."""
        return self.connected and self.socket is not None
        
    def send_raw_bytes(self, command: Union[bytes, list, tuple]) -> bool:
        """
        Send raw binary data without any modification (no terminators, no encoding).
        
        Args:
            command: Raw bytes to send. Can be:
                  - bytes object: b'\\x50\\x54\\x04...'
                  - list of integers: [0x50, 0x54, 0x04, ...]
                  - tuple of integers: (0x50, 0x54, 0x04, ...)
        
        Returns:
            True if sent successfully, False otherwise
        
        Example:
            # Send hex bytes: 50, 54, 04, 00, 01, 01, 08, 0E
            driver.send_raw_bytes([0x50, 0x54, 0x04, 0x00, 0x01, 0x01, 0x08, 0x0E])
            # Or
            driver.send_raw_bytes(b'\\x50\\x54\\x04\\x00\\x01\\x01\\x08\\x0E')
        """
        if isinstance(command, (list, tuple)):
            command = bytes(command)
        elif not isinstance(command, bytes):
            raise TypeError(f"Expected bytes, list, or tuple, got {type(command)}")
        
        return self.socket.sendall(bytes(command))
    
    def receive_response(self, buffer_size: int = 1024, 
                        timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Receive response from the controller.
        
        Args:
            buffer_size: Maximum bytes to receive
            timeout: Optional timeout override
            
        Returns:
            Response bytes or None if error/timeout
        """
        if not self.is_connected():
            print("Not connected to controller")
            return None
        
        try:
            with self.lock:
                if timeout:
                    old_timeout = self.socket.gettimeout()
                    self.socket.settimeout(timeout)
                
                response = self.socket.recv(buffer_size)
                
                if timeout:
                    self.socket.settimeout(old_timeout)
                
                # Parse response if parser is set
                if self.response_parser:
                    response = self.response_parser(response)
                #print('packet received: ' + ", ".join([hex(i) for i in response])) # for debugging
                return response
                
        except socket.timeout:
            print("Timeout while receiving response from controller")
            return None
        except socket.error as e:
            print(f"Error receiving response from controller: {e}")
            self.connected = False
            if self.auto_reconnect:
                self.connect()
            return None
        except Exception as e:
            print(f"Unexpected error receiving response from controller: {e}")
            return None
    
    def send_and_receive(self, command: Union[str, bytes, list], 
                        buffer_size: int = 1024, sleep_time: float = 0.03) -> Optional[bytes]:
        """
        Send a command and receive the response in one call.
        
        Args:
            command: Command to send (bytes)
            buffer_size: Maximum response size
            sleep_time: Time to sleep after sending the command (default is 0.03 seconds)
        Returns:
            Response bytes or None if error

        """
        # Handle list of integers (binary packet)
        #print('packet sent: ' + ", ".join([hex(i) for i in command])) # for debugging
        if isinstance(command, list):
            self.socket.sendall(bytes(command))
            time.sleep(sleep_time)  # Small delay for driver processing

            return self.receive_response(buffer_size)
        return None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


class MotorControl:
    """
    High-level motor control interface using binary packet protocol.
    
    Provides methods for speed control, position control, and encoder readout.
    Uses binary command packets with struct packing. Supports multiple axes.
    """
    
    def __init__(self, driver: Controller, 
                 axis_number: int = 1,
                 max_position: float = 400,
                 max_speed: float = 20,
                 max_acceleration: float = 120,
                 position_units: str = "degrees",
                 speed_unit: str = "degrees/s",
                 acceleration_unit: str = "degrees/s^2"):
        """
        Initialize motor control interface.
        
        Args:
            driver: Controller instance for communication
            axis_number: Axis number (1, 2, 3, etc.) - determines 5th byte in packet
            max_position: Maximum position in units
            max_speed: Maximum speed in units
            position_units: Units for position ("degrees", "radians", "units", etc.)
            speed_unit: Units for speed ("degrees/s", "radians/s", "units/s", etc.)
            acceleration_unit: Units for acceleration ("degrees/s^2", "radians/s^2", "units/s^2", etc.)
            deceleration_unit: Units for deceleration ("degrees/s^2", "radians/s^2", "units/s^2", etc.)
            """
        self.driver = driver
        self.Ack_response = bytes([6])
        self.axis_number = axis_number
        self.axis_byte = axis_number  # 0x01 for axis 1, 0x02 for axis 2, etc.
        self.max_position = max_position
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.position_units = position_units
        self.speed_unit = speed_unit
        self.acceleration_unit = acceleration_unit
        self.enabled = False
        
        # Binary packet structure:
        # Byte 0-1: Header (0x50, 0x54)
        # Byte 2: Length
        # Byte 3: Type/Flag (0x00)
        # Byte 4: Axis number (0x01, 0x02, etc.)
        # Byte 5+: Command-specific data
        # Byte 13: Checksum
        
        # Command codes (byte 5 and beyond)
        self.command_codes = {
            # === Movement Commands ===
            'enable': '013C', 
            'disable': '013D',
            'reset': '013E',  
            'set_acceleration': '0130',
            'set_speed': '0131',
            'set_position': '0132',
            'read_speed': '010A',
            'update': '0134', # this is used to update the motor position
            'speed_mode': '013A',
            'position_mode': '013B',
            'relative_movement': '0138',    # this is used to set the relative movement
            'absolute_movement': '0139',  
            'homing': '0135',
            'MOT_set_TUM': '013F', # this is used to set the MOT_TUM
            # === Limit Switch Commands ===
            'set_SWLS_positive': '0137', # this is used to set the SWLS positive
            'set_SWLS_negative': '0136', # this is used to set the SWLS negative
            'get_SWLS_positive': '010D',
            'get_SWLS_negative': '010C',
            'set_SWLS_activate': '0146',
            'get_SWLS_activate': '0110',
            'get_SWLS_handler': '0177',
            # === Configuration Commands ===
            'set_CFG_pos_range': '0C70',
            'get_CFG_pos_range': '0C71',
            # === Read Commands ===
            'read_position_load': '0109', # this is used to read the position load
            'read_position_motor': '0108', # this is used to read the position motor
            'is_motion_complete': '0E0B', # this is used to check if the motion is complete
            # === SIMULTANUSLY 2 AXES MOVEMENT ===
            'ssl_position': '0725',
            'ssl_speed': '0726',
            # === Scanner Commands ===
            'SCN_set_yaw_min_angle': '0400',
            'SCN_set_yaw_max_angle': '0401',
            'SCN_set_pitch_min_angle': '0402',
            'SCN_set_num_steps': '0403',
            'SCN_set_steps_height': '0404',
            'SCN_set_scan_speed': '0405',
            'SCN_Set_short_path': '0406',
            'SCN_start_scan_mode_snake': '040D',
            'SCN_stop_scan': '0408',
            # === Tracking Commands ===
            'set_camera_zoom': '0A18',
            'set_camera_fov': '0A16',
            'set_camera_resolution': '0A14',
            'stream_data_pixels': '0A1C',
            'set_tracking_status': '0A01',
            'get_tracking_status': '0A02',
            'set_camera_kp': '0A07',
            'set_camera_ki': '0A09',
            'set_camera_kd': '0A0B',
            'set_camera_acceleration': '0A0D',
            'get_camera_kp': '0A08',
            'get_camera_ki': '0A0A',
            'get_camera_kd': '0A0C',
            'get_camera_acceleration': '0A0E',
            'save_camera_PID': '0A11',
            'load_camera_PID': '0A12',
        }   
    
    #=== BUILD THE PACKET FOR THE MOTOR ===
    def _build_packet(self, command_type: str, data_value: Union[None, float, int, list] = None,
                      data_format: str = 'none',axis_zero: bool = False, group_id: int = 0x00) -> list:
        """
        Build binary packet with struct packing.
        
        Packet structure: [0x50, 0x54, length, 0x00, axis, command_type, ...data]
        Based on example: [0x50, 0x54, 0x04, 0x00, 0x01, 0x01, 0x3C, 0x42]
        
        Args:
            command_type: Command type byte (from command_codes)
            data_value: Data value to encode (None, number, or raw bytes list)
            data_format: Format for encoding data_value:
                - 'none': No data bytes (default)
                - 'raw': data_value is already a list of bytes
                - 'float32': 32-bit floating point (big-endian)
                - 'int32': 32-bit signed integer (big-endian)
                - 'uint32': 32-bit unsigned integer (big-endian)
                - 'int16': 16-bit signed integer (big-endian)
                - 'uint16': 16-bit unsigned integer (big-endian)
            axis_zero: If True, the axis number will be set to 0x00
            group_id: The group ID of the packet
        Returns:
            List of bytes representing the packet
        """
        # Convert data_value to bytes based on format
        if isinstance(data_value, (int, float)):
            data_bytes = []
            data_bytes = self._encode_data(data_value, data_format)
            data_length = len(data_bytes)
        elif isinstance(data_value, list):
            data_bytes = []
            for i in range(len(data_value)):
                data_bytes_item = self._encode_data(data_value[i], data_format)
                data_bytes.extend(data_bytes_item)
            data_length = len(data_bytes)
        else:
            data_length = 0
            data_bytes = []

        # Calculate packet length
        # Based on example: [0x50, 0x54, 0x04, 0x00, 0x01, 0x01, 0x3C, 0x42]
        # Length byte (0x04) appears to be the count of bytes after header and length byte
        # So: length = type(1) + axis(1) + command(1) + data_bytes
        packet_length = 1 + 1 + 1 + 1 + data_length  # type(0x00) + axis + command + data
        if axis_zero:
            axis_byte = 0x00
        else:
            axis_byte = self.axis_byte
        # Build packet: [header, length, type, axis, command, ...data]
        packet = [
            0x50, 0x54,           # Header
            packet_length,        # Length (data portion after header)
            group_id,             # Group ID
            axis_byte,       # Axis number (0x01 for axis 1, 0x02 for axis 2, etc.)
            int(command_type[0:2], 16), 
            int(command_type[2:4], 16),         # Command type
        ]
        
        # Add data bytes if provided
        packet.extend(data_bytes)
        packet.append(self._calculate_checksum(packet))
        #print('packet: ' + ", ".join([hex(i) for i in packet])) # for debugging
        return packet
    
    def _encode_data(self, data_value: Union[None, float, int, list, tuple], data_format: str) -> list:
        """
        Encode data value to bytes based on the specified format.
        
        Args:
            data_value: Value to encode
            data_format: Format string ('none', 'raw', 'float32', 'int32', etc.)
            
        Returns:
            List of bytes
        """
        if data_format == 'none' or data_value is None:
            return []
        
        if data_format == 'raw':
            if isinstance(data_value, list):
                return data_value
            raise ValueError("data_format='raw' requires data_value to be a list of bytes")
        
        # Struct format mapping
        format_map = {
            'float32': '>f',   # Big-endian 32-bit float
            'int32': '>i',     # Big-endian 32-bit signed int
            'uint32': '>I',    # Big-endian 32-bit unsigned int
            'int16': '>h',     # Big-endian 16-bit signed int
            'uint16': '>H',    # Big-endian 16-bit unsigned int
            'uint8': '>B',     # Big-endian 8-bit unsigned int (unsigned char) [1]
            'bool': '>B',     # Big-endian 8-bit unsigned int (unsigned char) [1]
        }
        
        if data_format not in format_map:
            raise ValueError(f"Unknown data_format: {data_format}. "
                           f"Valid formats: {list(format_map.keys()) + ['none', 'raw']}")
        
        return list(struct.pack(format_map[data_format], data_value))
    
    def _decode_data(self, packet: Union[bytes, list], data_format: str) -> Union[None, float, int, list]:
        """
        Decode bytes from a packet to a value based on the specified format.
        
        Args:
            packet: Bytes to decode (bytes object or list of integers)
            data_format: Format string ('none', 'raw', 'float32', 'int32', etc.)
            
        Returns:
            Decoded value (float, int, or raw list), or None if format is 'none'
        """
        # Convert list to bytes if needed
        if isinstance(packet, list):
            packet = bytes(packet)
        
        if data_format == 'none':
            return None
        
        if data_format == 'raw':
            return list(packet)
        
        # Struct format mapping (must match _encode_data)
        format_map = {
            'float32': '>f',   # Big-endian 32-bit float
            'int32': '>i',     # Big-endian 32-bit signed int
            'uint32': '>I',    # Big-endian 32-bit unsigned int
            'int16': '>h',     # Big-endian 16-bit signed int
            'uint16': '>H',    # Big-endian 16-bit unsigned int
            'uint8': '>B',     # Big-endian 8-bit unsigned int
            'binary': '>H',     # Binary
        }
        
        # Size mapping for each format
        size_map = {
            'float32': 4,
            'int32': 4,
            'uint32': 4,
            'int16': 2,
            'uint16': 2,
            'uint8': 1,
            'binary': 2,
        }
        
        if data_format not in format_map:
            raise ValueError(f"Unknown data_format: {data_format}. "
                           f"Valid formats: {list(format_map.keys()) + ['none', 'raw']}")
        
        expected_size = size_map[data_format]
        
        if expected_size == 1:
            return packet

        if len(packet) < expected_size:
            raise ValueError(f"Packet too short for {data_format}: "
                           f"expected {expected_size} bytes, got {len(packet)}")
        
        # Use last N bytes (where N is the expected size for the format)
        data_bytes = packet[-expected_size:]
        #    hex_list = [hex(b) for b in data_bytes]
        #    print('response: ', hex_list)
        return struct.unpack(format_map[data_format], data_bytes)[0]
    
    def _calculate_checksum(self, packet: list) -> int:
        """
        Calculate checksum for packet (simple sum modulo 256).
        Override this method if your protocol uses a different checksum algorithm.
        
        Args:
            packet: List of bytes
            
        Returns:
            Checksum byte value
        """
        return sum(packet[2:]) % 256
    
    #=== SET COMMANDS FOR THE MOTOR ===
    def axis_on(self) -> bool:
        """
        Enable the motor.
        
        Returns:
            True if successful, False otherwise
        """
        # Build packet: [0x50, 0x54, length, 0x00, axis, 0x01, checksum_bytes]
        # Based on your example: [0x50, 0x54, 0x04, 0x00, 0x01, 0x01, 0x3C, 0x42]
        packet = self._build_packet(self.command_codes['enable'])
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            self.enabled = True
            print(f"Axis {self.axis_number} enabled")
            return True
        return False
    
    def axis_off(self) -> bool:
        
        """
        Disable the motor.
        
        Returns:
            True if successful, False otherwise
        """
        # Build packet: [0x50, 0x54, length, 0x00, axis, 0x02, checksum_bytes]
        # Based on your example: [0x50, 0x54, 0x04, 0x00, 0x01, 0x01, 0x3D, 0x43]
        packet = self._build_packet(self.command_codes['disable'])
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            self.enabled = False
            print(f"Axis {self.axis_number} disabled")
            return True
        return False

    def axis_reset(self) -> bool:
        """
        Reset the motor axis.

        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['reset'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} reset")
            return True
        return False

    def set_acceleration(self, acceleration: float) -> bool:
        """
        Set motor acceleration using binary packet.
        
        Args:
            acceleration: Target acceleration (RPM/s or units/s)
            unit: Speed unit ("RPM" or "units")
            data_format: Data encoding format ('int32', 'float32', etc.)
            
        Returns:
            True if successful, False otherwise
        """
        if abs(acceleration) > self.max_acceleration:
            print(f"Warning: Acceleration {acceleration} exceeds maximum {self.max_acceleration}")
            acceleration = max(-self.max_acceleration, min(self.max_acceleration, acceleration))
        
        # Convert to int if using integer format
        data_format = 'float32'
        
        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_acceleration'], acceleration, data_format)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} acceleration set to {acceleration} {self.acceleration_unit}")
            return True
        return False
        
    def set_speed(self, speed: float) -> bool:
        """
        Set motor speed in degrees/s using binary packet.
        
        Args:
            speed: Target speed (RPM or units)
            unit: Speed unit ("degrees/s" or "units")
            data_format: Data encoding format ('int32', 'float32', etc.)
            
        Returns:
            True if successful, False otherwise
        """
        if abs(speed) > self.max_speed:
            print(f"Warning: Speed {speed} exceeds maximum {self.max_speed}")
            speed = max(-self.max_speed, min(self.max_speed, speed))
        
        # Convert speed to 32-bit integer (adjust conversion as needed)
        # Assuming speed is in RPM, convert to counts/sec or similar
        data_format = 'float32'
        
        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_speed'], speed, data_format)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} speed set to {speed} {self.speed_unit}")
            return True
        return False
    
    def set_position(self, position: float) -> bool:
        """
        Move motor to absolute position using binary packet.
        
        Args:
            position: Target position in configured units
            unit: Position unit ("degrees" or "units")
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if abs(position) > self.max_position:
            print(f"Warning: Position {position} exceeds maximum {self.max_position}")
            position = max(-self.max_position, min(self.max_position, position))
        
        # Convert speed to 32-bit integer (adjust conversion as needed)
        # Assuming speed is in RPM, convert to counts/sec or similar
        data_format = 'float32'
        
        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_position'], position, data_format)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} position set to {position} {self.position_units}")
            return True
        return False
   
    def update(self) -> bool:
        """
        Update the motor.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        packet = self._build_packet(self.command_codes['update'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} updated")
            return True
        return False
    
    def set_movement_mode(self, movement_mode: str) -> bool:
        """
        Switch between speed mode and position mode.
        
        Args:
            mode: Movement mode - 'speed' or 'position'
            
        Returns:
            True if successful, False otherwise
        """
        mode_lower = movement_mode.lower()
        
        if mode_lower == 'speed':
            command_code = self.command_codes['speed_mode']
        elif mode_lower == 'position':
            command_code = self.command_codes['position_mode']
        else:
            print(f"Invalid mode: {movement_mode}. Use 'speed' or 'position'")
            return False
        
        packet = self._build_packet(command_code)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} set to {mode_lower} mode")
            return True
        return False

    def set_movement_type(self, movement_type: str) -> bool:
        """
        Switch between relative and absolute movement type.
        
        Args:
            movement_type: Movement type - 'relative' or 'absolute'
            
        Returns:
            True if successful, False otherwise
        """
        type_lower = movement_type.lower()
        
        if type_lower == 'relative':
            command_code = self.command_codes['relative_movement']
        elif type_lower == 'absolute':
            command_code = self.command_codes['absolute_movement']
        else:
            print(f"Invalid movement type: {movement_type}. Use 'relative' or 'absolute'")
            return False
        
        packet = self._build_packet(command_code)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} set to {type_lower} movement type")
            return True
        return False

    def set_homing(self) -> bool:
        """
        Set the homing.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['homing'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} homing set")
            return True
        return False

    def MOT_set_TUM(self) -> bool:
        """
        Set the MOT_TUM.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['MOT_set_TUM'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} MOT_TUM set")
            return True
        return False
    
    #=== SET LIMIT SWITCH COMMANDS FOR THE MOTOR ===
    def set_SWLS_positive(self, LS_positive: float) -> bool:
        """
        Set the SWLS positive.
        The LS_positive is the positive limit switch position in degrees.
        Args:
            LS_positive: Positive limit switch position in degrees
            
        Returns:
            True if successful, False otherwise
        """

        data_format = 'float32'

        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_SWLS_positive'], LS_positive, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} SWLS positive set to {LS_positive} degrees")
            return True
        return False

    def set_SWLS_negative(self, LS_negative: float) -> bool:    
        """
        Set the SWLS negative.

        Args:
            LS_negative: Negative limit switch position in degrees
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'

        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_SWLS_negative'], LS_negative, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} SWLS negative set to {LS_negative} degrees")
            return True
        return False

    def get_SWLS_positive(self) -> bool:
        """
        Get the SWLS positive.
        
        Returns:
            SWLS positive as float, or None if error
        """
        packet = self._build_packet(self.command_codes['get_SWLS_positive'])
        response = self.driver.send_and_receive(packet)
        if response:
            SWLS_positive = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} SWLS positive: {SWLS_positive} {self.position_units}")
            return SWLS_positive
        return None

    def get_SWLS_negative(self) -> bool:
        """
        Get the SWLS negative.

        Returns:
            SWLS negative as float, or None if error
        """
        packet = self._build_packet(self.command_codes['get_SWLS_negative'])
        response = self.driver.send_and_receive(packet)
        if response:
            SWLS_negative = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} SWLS negative: {SWLS_negative} {self.position_units}")
            return SWLS_negative
        return None

    def set_SWLS_activate(self, SWLS_activate: bool) -> bool:
        """
        Set the SWLS activate.
        
        Returns:
            True if SWLS activate is activated, False if SWLS activate is not activated 
        """

        data_format = 'uint8'


        # Build packet with specified data format
        packet = self._build_packet(self.command_codes['set_SWLS_activate'], SWLS_activate, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            if SWLS_activate:
                print(f"Axis {self.axis_number} SWLS activate is activated")
                return True
            else:
                print(f"Axis {self.axis_number} SWLS activate is not activated")
                return True
        return False

    def get_SWLS_activate(self) -> bool:
        """
        Get the SWLS activate.
        
        Returns:
            True if SWLS is activated, False if SWLS is not activated
        """
        packet = self._build_packet(self.command_codes['get_SWLS_activate'])
        response = self.driver.send_and_receive(packet) 
        if response:
            SWLS_activate = self._decode_data(response[7], 'uint8')
            if SWLS_activate == 1:
                print(f"Axis {self.axis_number} SWLS  is activated")
                return True
            else:
                print(f"Axis {self.axis_number} SWLS  is not activated")
                return False
        return None

    def get_SWLS_handler(self) -> bool:
        """
        Get the SWLS handler.
        
        Returns:
            SWLS handler as int, or None if error
        """
        packet = self._build_packet(self.command_codes['get_SWLS_handler'])
        response = self.driver.send_and_receive(packet) 
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} SWLS handler set")
            return True
        return False

    #=== SET CONFIGUTATIONS FOR THE MOTOR ===
    def set_CFG_pos_range(self, pos_range_type: int) -> bool:
        """
        Set the position range.
        
        Args:
            pos_range_type: Position range type (0: (-inf) - (+inf) degrees, 
                                                 1: (-360) - (+360) degrees, 
                                                 2: (-180) - (+180) degrees, 
                                                 3: (0) - (360) degrees)

            
        Returns:
            True if successful, False otherwise
        """

        pos_range_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        pos_range_type_dict = {    
            0: '-inf - +inf',
            1: '-360 - +360',
            2: '-180 - +180',
            3: '0 - +360',
        }

        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_CFG_pos_range'], pos_range_data[pos_range_type], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} position range type set to {pos_range_type_dict[pos_range_type]}")
            return True
        return False

    def get_CFG_pos_range(self) -> bool:
        """
        Get the position range type.
        
        Returns:
            True if successful, False otherwise
        """
        pos_range_type = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        pos_range_type_dict = {    
            0: '-inf - +inf',
            1: '-360 - +360',
            2: '-180 - +180',
            3: '0 - +360',
        }
        
        packet = self._build_packet(self.command_codes['get_CFG_pos_range'])
        response = self.driver.send_and_receive(packet)

        if response:
            response_data = self._decode_data(response[7], 'uint8')       
            print(f"Axis {self.axis_number} position range type: {pos_range_type_dict[response_data]}")
            return pos_range_type_dict[response_data]
        return False    

    #=== GET COMMANDS FOR THE MOTOR ===
    def get_status(self) -> Dict[str, Any]: # intern command
        """
        Get comprehensive motor status.
        
        Returns:
            Dictionary with position, speed, enabled status, etc.
        """
        status = {
            'axis_number': self.axis_number,
            'position': self.get_position(),
            'speed': self.get_speed(),
            'enabled': self.enabled,
            'SWLS_positive': self.get_SWLS_positive(),
            'SWLS_negative': self.get_SWLS_negative(),
            'SWLS_activate': self.get_SWLS_activate(),
        }
        print(f"========== MOTOR {self.axis_number} STATUS ==========")
        for key, value in status.items():
            print(f"{key}: {value}")
        print("========================================================")
        return status
    
    def get_axis_number(self) -> int:# intern command
        """
        Get the axis number for this motor control instance.
        
        Returns:
            Axis number (1, 2, 3, etc.)
        """
        return self.axis_number

    def get_position(self) -> Optional[float]:
        """
        Get current motor position using binary packet.

        Args:
            unit: Position unit ('deg' for degrees, 'rad' for radians, 'counts' for raw)

        Returns:
            Current position as float, or None if error
        """
        # Build packet: [header, length, type, axis, command_read_position_load]
        packet = self._build_packet(self.command_codes['read_position_load'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                # The position value is in response[7:11] as [41, 6f, b2, 00], per example
                # This is big-endian 32-bit float (IEEE 754)
                position_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} position: {position_value} {self.position_units}")
                return position_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing position response: {e}, raw: {response.hex()}")
                return None
        return None
    
    def get_speed(self) -> Optional[float]:
        """
        Get current motor speed using binary packet.
        
        Returns:
            Current speed in RPM or units, or None if error
        """
        packet = self._build_packet(self.command_codes['read_speed'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                speed_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} speed: {speed_value} {self.speed_unit}")
                return speed_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing speed response: {e}, raw: {response.hex()}")
                return None
        return None

    def is_motion_complete(self) -> bool:
        """
        Check if the motion is complete.
        
        Returns:
            True if motion is complete, False otherwise
        """
        packet = self._build_packet(self.command_codes['is_motion_complete'])
        response = self.driver.send_and_receive(packet)
        
        if response:
            try:
                binary_value = self._decode_data(response[7:9], 'binary')
                # 3. Convert the integer to its binary string
                # bin() returns a string prefixed with '0b', which we slice off with [2:]
                binary_string = bin(binary_value)[2:]
                # To ensure a fixed width (e.g., 16 bits for two bytes), you can use zfill()
                padded_binary_string = binary_string.zfill(16)
                if padded_binary_string[-3] == '1':
                    print(f"Axis {self.axis_number}  motion was completed")
                    return True
                else:
                    print(f"Axis {self.axis_number} motion was not completed")
                    return False
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing speed response: {e}, raw: {response.hex()}")
                return False
        return False

    #=== SIMULTANUSLY 2 AXES MOVEMENT ===
    def SSL_position(self, position_axis_1: float, position_axis_2: float, mode: int) -> bool:
        """
        Set the slave position.

        Args:
            position_axis_1: Position of axis 1
            position_axis_2: Position of axis 2
            mode: Mode of the slave position
        
        Returns:
            True if successful, False otherwise
        """   
        dict_pos_mode = {
            0: 0x00,    #last defined mode
            1: 0x01,    #Axis 1 is absolute, Axis 2 is absolute
            2: 0x02,    #Axis 1 is relative, Axis 2 is relative
            3: 0x03,    #Axis 1 is absolute, Axis 2 is relative
            4: 0x04,    #Axis 1 is relative, Axis 2 is absolute
        }
        
        packed_bytes_axis_1 = list(struct.pack('>h', position_axis_1 * 100))
        packed_bytes_axis_2 = list(struct.pack('>h', position_axis_2 * 100))    


        # Build packet: [header, length, type, axis, command, ...data]
        packet = [
            0x50, 0x54,           # Header
            0x08,                 # Length (data portion after header)
            0x00,                 # Type/Flag
            dict_pos_mode[mode],       # Movement mode
            int(self.command_codes['ssl_position'][0:2], 16),         # Command type
            int(self.command_codes['ssl_position'][2:4], 16),         # Command type
        ]

        packet.extend(packed_bytes_axis_1)
        packet.extend(packed_bytes_axis_2)
        packet.append(self._calculate_checksum(packet))
        print('packet: ' + ", ".join([hex(i) for i in packet]))

        response = self.driver.send_and_receive(packet)
        print('response: ', response)

        print(f"Axis {self.axis_number} SSL position set to {position_axis_1} {self.position_units} and \
         Axis {self.axis_number} SSL position set to {position_axis_2} {self.position_units}")
        return True

    def SSL_speed(self, speed_axis_1: float, speed_axis_2: float) -> bool:
        """
        Set the slave speed.

        Args:
            speed_axis_1: Speed of axis 1
            speed_axis_2: Speed of axis 2
            mode: Mode of the slave speed
        """

        packed_bytes_axis_1 = list(struct.pack('>h', int(speed_axis_1 * 100)))
        packed_bytes_axis_2 = list(struct.pack('>h', int(speed_axis_2 * 100)))    

        packet = [
            0x50, 0x54,           # Header
            0x08,                 # Length (data portion after header)
            0x00,                 # Type/Flag
            0x00,       # Movement mode (Always)
            int(self.command_codes['ssl_speed'][0:2], 16),         # Command type
            int(self.command_codes['ssl_speed'][2:4], 16),         # Command type
        ]

        # Build packet: [header, length, type, axis, command, ...data]

        packet.extend(packed_bytes_axis_1)
        packet.extend(packed_bytes_axis_2)
        packet.append(self._calculate_checksum(packet))
        #print('packet: ' + ", ".join([hex(i) for i in packet]))

        response = self.driver.send_and_receive(packet)
        #print('response: ', response)
        axis1_bytes = response[7:9]   # bytes 8–9
        axis2_bytes = response[9:11]  # bytes 10–11

        axis1_pos = int.from_bytes(axis1_bytes, byteorder="big", signed=True) / 100
        axis2_pos = int.from_bytes(axis2_bytes, byteorder="big", signed=True) / 100

        #print(f"| axis1_absolute_position={axis1_pos} | axis2_absolute_position={axis2_pos}")
        
        #print(f"Axis 1 speed set to {speed_axis_1} {self.speed_unit} and \
        # Axis 2 speed set to {speed_axis_2} {self.speed_unit}")
        return axis1_pos, axis2_pos
    
    #=== SCANNER COMMANDS ===
    def SCN_set_yaw_min_angle(self, angle: float) -> bool:
        """
        Set the yaw minimum angle.
        
        Args:
            angle: Target angle (degrees)
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'

        packet = self._build_packet(self.command_codes['SCN_set_yaw_min_angle'], angle, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} yaw minimum angle set to {angle} degrees")
            return True
        return False

    def SCN_set_yaw_max_angle(self, angle: float) -> bool:
        """
        Set the yaw maximum angle.
        
        Args:
            angle: Target angle (degrees)
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        
        packet = self._build_packet(self.command_codes['SCN_set_yaw_max_angle'], angle, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} yaw maximum angle set to {angle} degrees")
            return True
        return False

    def SCN_set_pitch_min_angle(self, angle: float) -> bool:
        """
        Set the pitch minimum angle.
        
        Args:
            angle: Target angle (degrees)
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        
        packet = self._build_packet(self.command_codes['SCN_set_pitch_min_angle'], angle, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} pitch minimum angle set to {angle} degrees")
            return True
        return False

    def SCN_set_num_steps(self, steps: int) -> bool:
        """
        Set the number of steps.
        
        Args:
            steps: Number of steps
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'uint8'
        
        packet = self._build_packet(self.command_codes['SCN_set_num_steps'], steps, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} number of steps set to {steps}")
            return True
        return False

    def SCN_set_steps_height(self, height: float) -> bool:
        """
        Set the steps height.
        
        Args:
            height: Height of the steps
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        
        packet = self._build_packet(self.command_codes['SCN_set_steps_height'], height, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} steps height set to {height} degrees")
            return True
        return False

    def SCN_set_scan_speed(self, speed: float) -> bool:
        """
        Set the scan speed.
        
        Args:
            speed: Speed of the scan
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        
        packet = self._build_packet(self.command_codes['SCN_set_scan_speed'], speed, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} scan speed set to {speed} degrees")
            return True
        return False

    def SCN_Set_short_path(self, short_path: bool) -> bool:
        """
        Set the short path.
        
        Args:
            short_path: Short path
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'bool'
        
        packet = self._build_packet(self.command_codes['SCN_Set_short_path'], short_path, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} short path set to {short_path}")
            return True
        return False

    def SCN_start_scan_mode_snake(self) -> bool:
        """
        Start the scan mode snake.
        
        Returns:
            True if successful, False otherwise
        """
        
        packet = self._build_packet(self.command_codes['SCN_start_scan_mode_snake'], axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} start scan mode snake")
            return True
        return False

    def SCN_stop_scan(self) -> bool:
        """
        Stop the scan.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['SCN_stop_scan'], axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} stop scan")
            return True
        return False

    # === Tracking Commands ===
    def set_camera_zoom(self, zoom: float) -> bool:
        """
        Set the zoom of the camera.
        
        Args:
            zoom: Zoom of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        
        packet = self._build_packet(self.command_codes['set_camera_zoom'], zoom, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} zoom set to {zoom}")
            return True
        return False
    
    def set_camera_fov(self, fov: tuple[float, float]) -> bool:
        """
        Set the fov of the camera.
        
        Args:
            fov: Fov of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'uint16'
        
        fov_horizontal = int(fov[0] * 100)
        fov_vertical = int(fov[1] * 100)

        packet = self._build_packet(self.command_codes['set_camera_fov'], [fov_horizontal, fov_vertical], data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} fov set to {fov}")
            return True
        return False
    
    def set_camera_resolution(self, resolution: tuple[int, int]) -> bool:
        """
        Set the resolution of the camera.
        
        Args:
            resolution: Resolution of the camera    
            
        Returns:
            True if successful, False otherwise
        """

        resolution_width = int(resolution[0])
        resolution_height = int(resolution[1])

        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['set_camera_resolution'], [resolution_width, resolution_height], data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} resolution set to {resolution[0]}x{resolution[1]}")
            return True
        return False

    def stream_data_pixels(self, target_id: int = 0, classification: int = 0, confidance: int = 0, x1: int = 0, y1: int = 0, x2: int = 0, y2: int = 0) -> bool:
        """
        Stream the data pixels.
        
        Returns:
            True if successful, False otherwise
        """
        
        x1_bytes = list(struct.pack('>h', int(x1)))
        y1_bytes = list(struct.pack('>h', int(y1)))
        x2_bytes = list(struct.pack('>h', int(x2)))
        y2_bytes = list(struct.pack('>h', int(y2)))
         
        # Build packet: [header, length, type, axis, command, ...data]
        packet = [
            0x50, 0x54,           # Header
            0x0F,                 # Length (data portion after header)
            0x00,                 # Group ID
            0x00,                 # Axis number
            int(self.command_codes['stream_data_pixels'][0:2], 16), # Command type
            int(self.command_codes['stream_data_pixels'][2:4], 16), # Command type
            0x00,                 # target_id
            0x00,                 # classification
            0x00,                 # confidance
        ]
        
        packet.extend(x1_bytes)
        packet.extend(y1_bytes)
        packet.extend(x2_bytes)
        packet.extend(y2_bytes)

        packet.append(self._calculate_checksum(packet))
        response = self.driver.send_and_receive(packet)

        if response == self.Ack_response:
            print(f"Axis {self.axis_number} stream data pixels set to {target_id}, {classification}, {confidance}, {x1}, {y1}, {x2}, {y2} successfully")
            return True
        return False
    
    def set_tracking_status(self, tracking_mode: int) -> bool:
        """
        Set the track status.
        
        Args:
            tracking_mode: Tracking mode
        """
        dict_status_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }

        tracking_status_type = {
            0: 'tracking_off',
            1: 'tracking_on',
            2: 'auto_tracking_on',
        }

        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_tracking_status'], tracking_mode, data_format, axis_zero=True)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} track status set to {tracking_status_type[tracking_mode]}")
            return True
        return False
    
    def get_tracking_status(self) -> bool:
        """
        Get the tracking status.
        
        Returns:
            True if successful, False otherwise
        """
        tracking_status_type = {
            0: 'tracking_off',
            1: 'tracking_on',
            2: 'auto_tracking_on',
        }
        packet = self._build_packet(self.command_codes['get_tracking_status'], axis_zero=True, group_id=0x01)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} tracking status get to {tracking_status_type[response]}")
            return response
        return False
    
    def set_camera_kp(self, kp: float) -> bool:
        """
        Set the kp of the camera.
        
        Args:
            kp: Kp of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'

        packet = self._build_packet(self.command_codes['set_camera_kp'], kp, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} kp set to {kp}")
            return True
        return False
    
    def set_camera_ki(self, ki: float) -> bool:
        """
        Set the ki of the camera.
        
        Args:
            ki: Ki of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_camera_ki'], ki, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} ki set to {ki}")
            return True
        return False
    
    def set_camera_kd(self, kd: float) -> bool:
        """
        Set the kd of the camera.
        
        Args:
            kd: Kd of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_camera_kd'], kd, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} kd set to {kd}")
            return True
        return False
    
    def set_camera_acceleration(self, acceleration: float) -> bool:
        """
        Set the acceleration of the camera.
        
        Args:
            acceleration: Acceleration of the camera
            
        Returns:
            True if successful, False otherwise
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_camera_acceleration'], acceleration, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} acceleration set to {acceleration}")
            return True
        return False
    
    def get_camera_kp(self) -> float:
        """
        Get the kp of the camera.
        
        Returns:
            The kp of the camera
        """
        packet = self._build_packet(self.command_codes['get_camera_kp'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                kp_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} kp: {kp_value}")
                return kp_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing kp response: {e}, raw: {response.hex()}")
                return None
        return None
    
    def get_camera_ki(self) -> float:
        """
        Get the ki of the camera.
        
        Returns:
            The ki of the camera
        """
        packet = self._build_packet(self.command_codes['get_camera_ki'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                ki_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} ki: {ki_value}")
                return ki_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing ki response: {e}, raw: {response.hex()}")
                return None
        return False
    
    def get_camera_kd(self) -> float:
        """
        Get the kd of the camera.
        
        Returns:
            The kd of the camera
        """
        packet = self._build_packet(self.command_codes['get_camera_kd'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                kd_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} kd: {kd_value}")
                return kd_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing kd response: {e}, raw: {response.hex()}")
                return None
        return None
    
    def get_camera_acceleration(self) -> float:
        """
        Get the acceleration of the camera.
        
        Returns:
            The acceleration of the camera
        """
        packet = self._build_packet(self.command_codes['get_camera_acceleration'])
        response = self.driver.send_and_receive(packet)
        if response:
            try:
                acceleration_value = self._decode_data(response[7:11], 'float32')
                print(f"Axis {self.axis_number} acceleration: {acceleration_value}")
                return acceleration_value
            except (struct.error, ValueError, IndexError) as e:
                print(f"Error parsing acceleration response: {e}, raw: {response.hex()}")
                return None
            print(f"Axis {self.axis_number} acceleration get to {response}")
            return response
        return False
    
    def save_camera_PID(self) -> bool:
        """
        Save the camera PID.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['save_camera_PID'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} camera PID saved")
            return True
        return False
    
    def load_camera_PID(self) -> bool:
        """
        Load the camera PID.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['load_camera_PID'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} camera PID loaded")
            return True
        return False  