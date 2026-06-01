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
        if not isinstance(command, list):
            return None
        if not self.is_connected():
            print("Not connected to controller")
            return None

        with self.lock:
            try:
                self.socket.sendall(bytes(command))
                time.sleep(sleep_time)
                response = self.socket.recv(buffer_size)
                if self.response_parser:
                    response = self.response_parser(response)
                return response
            except socket.timeout:
                print("Timeout while receiving response from controller")
                return None
            except socket.error as e:
                print(f"Error during send/receive: {e}")
                self.connected = False
                if self.auto_reconnect:
                    self.connect()
                return None
            except Exception as e:
                print(f"Unexpected error during send/receive: {e}")
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
            
            # === Configuration Commands Internal ===
            #Technosoft_parameters
            'get_cfg_Gear_Ratio': '0C10',
            'set_cfg_Gear_Ratio': '0C11',
            'get_cfg_Max_VDC': '0C12',
            'set_cfg_Max_VDC': '0C13',
            'get_cfg_Peak_Current': '0C14',
            'set_cfg_Peak_Current': '0C15',
            'get_cfg_slow_loop_sampling': '0C16',
            'set_cfg_slow_loop_sampling': '0C17',
            'set_cfg_micro_Steps': '0C45',
            'get_cfg_micro_Steps': '0C46',
            'set_cfg_steps_Per_Rev': '0C47',
            'get_cfg_steps_Per_Rev': '0C48',
            'set_cfg_close_loop_enable': '0CB7',
            'get_cfg_close_loop_enable': '0CB8',
            'set_cfg_Motor_Type': '0C22',
            'get_cfg_Motor_Type': '0C23',
            'set_cfg_Apos_Load_Type': '0C43',
            'get_cfg_Apos_Load_Type': '0C44',
            'set_cfg_Load_Encoder_Lines': '0C35',
            'get_cfg_Load_Encoder_Lines': '0C36',
            'get_cfg_Encoder_Lines': '0C08',
            'set_cfg_Encoder_Lines': '0C09',
            'set_encoder_Location': '0C4D',
            'get_encoder_Location': '0C4E',

            #Communication info
            'get_cfg_PC_COM': '0C00',
            'set_cfg_PC_COM': '0C01',
            'get_cfg_TS_COM_TYPE': '0C02',
            'set_cfg_TS_COM_TYPE': '0C03',
            'set_cfg_Multi_Com': '0C9A',
            'get_cfg_Multi_Com': '0C9B',
            'set_cfg_Can_Baud_rate': '0C98',
            'get_cfg_Can_Baud_rate': '0C99',
            
            #biss configuration
            'set_Biss_Resolution': '0C65',
            'get_Biss_Resolution': '0C66',
            'set_Biss_Com': '0C67',
            'get_Biss_Com': '0C68',
            'set_Abs_Enc_Offset': '0C72',
            'get_Abs_Enc_Offset': '0C73',
            'set_Abs_Enc_Reverse': '0C74',
            'get_Abs_Enc_Reverse': '0C75',

            #limits motion parameters
            'set_cfg_Max_Speed': '0C24',
            'get_cfg_Max_Speed': '0C25',
            'set_cfg_Min_Speed': '0C57',
            'get_cfg_Min_Speed': '0C58',
            'set_cfg_max_Acceleration': '0C41',
            'get_cfg_max_Acceleration': '0C42',
            'set_cfg_Min_Acceleration': '0CAD',
            'get_cfg_Min_Acceleration': '0CAE',
           
            #IMU configuration
            'get_cfg_Sensor_Type': '0C04',
            'set_cfg_Sensor_Type': '0C05',   
            'set_cfg_Reversed_IMU': '0C3F',
            'get_cfg_Reversed_IMU': '0C40',
            'set_cfg_Imu_Com': '0C59',
            'get_cfg_Imu_Com': '0C5A',
            'set_cfg_Imu_Baud_Rate': '0C5B',
            'get_cfg_Imu_Baud_Rate': '0C5C',
            'set_cfg_Imu_Freq': '0C5D',
            'get_cfg_Imu_Freq': '0C5E',
            
            #General configuration
            'set_cfg_Reversed_Axis': '0C2A',
            'get_cfg_Reversed_Axis': '0C2B',
            'set_cfg_System_Type': '0C3B',
            'get_cfg_System_Type': '0C3C',
            'get_cfg_Firmware_Version': '0C4A',
            'set_cfg_Serial_Number': '0C2E',
            'get_cfg_Serial_Number': '0C2F',     
            'save_Serial_Number': '0C6F',
            'cfg_save': '0C18',
            'cfg_load': '0C19',
            'cfg_restore_default': '0C34',
            'set_cfg_Com_Axes_TS': '0C30',
            'get_cfg_Com_Axes_TS': '0C31',
            'set_cfg_Calc_Aspd': '0CA1',
            'get_cfg_Calc_Aspd': '0CA2',
            'set_cfg_Num_Of_Axes': '0C32',
            'get_cfg_Num_Of_Axes': '0C33',

            
            # === IP communication Commands ===
            'Set_Controller_IP': '070A',
            'Get_Controller_IP': '070D',
            'Set_Controller_Port': '070D',
            'Get_Controller_Port': '070E',
            'Set_Controller_Subnet_Mask': '071A',
            'Get_Controller_Subnet_Mask': '071B',
            'Save_IP': '0710',
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
        #print('response: ', data_bytes)
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

    #=== Configuration Commands Internal ===
    #Parameters from Technosoft
    def get_cfg_Gear_Ratio(self) -> float:
        """
        Get the gear ratio.
        
        Returns:
            The gear ratio
        """
        packet = self._build_packet(self.command_codes['get_cfg_Gear_Ratio'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} gear ratio: {response_data}")
            return response_data
        return None
    
    def set_cfg_Gear_Ratio(self, gear_ratio: float) -> bool:
        """
        Set the gear ratio.
        
        Args:
            gear_ratio: Gear ratio
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Gear_Ratio'], gear_ratio, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} gear ratio set to {gear_ratio}")
            return True
        return False
    
    def get_cfg_Max_VDC(self) -> float:
        """
        Get the maximum voltage.
        
        Returns:
            The maximum voltage
        """
        packet = self._build_packet(self.command_codes['get_cfg_Max_VDC'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} maximum voltage: {response_data}")
            return response_data
        return None

    def set_cfg_Max_VDC(self, max_voltage: float) -> bool:
        """
        Set the maximum voltage.
        
        Args:
            max_voltage: Maximum voltage
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Max_VDC'], max_voltage, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} maximum voltage set to {max_voltage}")
            return True
        return False
   
    def get_cfg_Peak_Current(self) -> float:
        """
        Get the peak current.
        
        Returns:
            The peak current
        """
        packet = self._build_packet(self.command_codes['get_cfg_Peak_Current'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} peak current: {response_data}")
            return response_data
        return None
    
    def set_cfg_Peak_Current(self, peak_current: float) -> bool:
        """
        Set the peak current.
        
        Args:
            peak_current: Peak current
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Peak_Current'], peak_current, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} peak current set to {peak_current}")
            return True
        return False

    def get_cfg_slow_loop_sampling(self) -> float:
        """
        Get the slow loop sampling.
        
        Returns:
            The slow loop sampling
        """
        packet = self._build_packet(self.command_codes['get_cfg_slow_loop_sampling'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} slow loop sampling: {response_data}")
            return response_data
        return None
    
    def set_cfg_slow_loop_sampling(self, slow_loop_sampling: float) -> bool:
        """
        Set the slow loop sampling.
        
        Args:
            slow_loop_sampling: Slow loop sampling
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_slow_loop_sampling'], slow_loop_sampling, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} slow loop sampling set to {slow_loop_sampling}")
            return True
        return False

    def get_cfg_micro_Steps(self) -> int:
        """
        Get the micro steps.
        
        Returns:
            The micro steps
        """
        packet = self._build_packet(self.command_codes['get_cfg_micro_Steps'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:9], 'uint16')
            print(f"Axis {self.axis_number} micro steps: {response_data}")
            return response_data
        return None

    def set_cfg_micro_Steps(self, micro_steps: int) -> bool:
        """
        Set the micro steps.   
        
        Args:
            micro_steps: Micro steps
        """
        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['set_cfg_micro_Steps'], micro_steps, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} micro steps set to {micro_steps}")
            return True
        return False

    def get_cfg_steps_Per_Rev(self) -> int:
        """
        Get the steps per revolution.
        
        Returns:
            The steps per revolution
        """
        packet = self._build_packet(self.command_codes['get_cfg_steps_Per_Rev'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:9], 'uint16')
            print(f"Axis {self.axis_number} steps per revolution: {response_data}")
            return response_data
        return None
    
    def set_cfg_steps_Per_Rev(self, steps_per_rev: int) -> bool:
        """
        Set the steps per revolution.
        
        Args:
            steps_per_rev: Steps per revolution
        """
        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['set_cfg_steps_Per_Rev'], steps_per_rev, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} steps per revolution set to {steps_per_rev}")
            return True
        return False

    def get_cfg_close_loop_enable(self) -> bool:
        """
        Get the close loop enable.
        
        Returns:
            The close loop enable
        """
        close_loop_enable_data = {
            0: 0x00,
            1: 0x01,
        }
        close_loop_enable_dict = {
            0: 'Disabled',
            1: 'Enabled',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['get_cfg_close_loop_enable'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} close loop enable: {close_loop_enable_dict[response_data]}")
            return close_loop_enable_dict[response_data]
        return None
    
    def set_cfg_close_loop_enable(self, close_loop_enable: bool) -> bool:
        """
        Set the close loop enable.
        
        Args:
            close_loop_enable: Close loop enable
        """
        close_loop_enable_data = {
            0: 0x00,
            1: 0x01,
        }
        close_loop_enable_dict = {
            0: 'Disabled',
            1: 'Enabled',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_close_loop_enable'], close_loop_enable_data[close_loop_enable], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} close loop enable set to {close_loop_enable_dict[close_loop_enable]}")
            return True
        return False

    def get_cfg_Motor_Type(self) -> int:
        """
        Get the motor type.
        
        Returns:
            The motor type
        """

        motor_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }
        motor_type_dict = {
            0: 'Stepper',
            1: 'BLDC',
            2: 'DC',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Motor_Type'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')       
            print(f"Axis {self.axis_number} motor type: {motor_type_dict[response_data]}")
            return motor_type_dict[response_data]
        return None
    
    def set_cfg_Motor_Type(self, motor_type: int) -> bool:
        """
        Set the motor type.
        
        Args:
            motor_type: Motor type
        """
        motor_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }
        motor_type_dict = {
            0: 'Stepper',
            1: 'BLDC',
            2: 'DC',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Motor_Type'], motor_type_data[motor_type], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} motor type set to {motor_type_dict[motor_type]}")
            return True
        return False

    def get_cfg_Apos_Load_Type(self) -> str:
        """
        Get the Apos load type.
        
        Returns:
            The Apos load type
        """
        apos_load_type_dict = {
            0: 'Apos_load',
            1: 'Apos_SSI',
            2: 'TPOS',
            3: 'Apos_Monitor',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Apos_Load_Type'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} Apos load type: {apos_load_type_dict[response_data]}")
            return apos_load_type_dict[response_data]
        return None
    
    def set_cfg_Apos_Load_Type(self, apos_load_type: int) -> bool:
        """
        Set the Apos load type.
        
        Args:
            apos_load_type: Apos load type
        """
        apos_load_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        apos_load_type_dict = {
            0: 'Apos_load',
            1: 'Apos_SSI',
            2: 'TPOS',
            3: 'Apos_Monitor',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Apos_Load_Type'], apos_load_type_data[apos_load_type], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} Apos load type set to {apos_load_type_dict[apos_load_type]}")
            return True
        return False 
        
    def get_cfg_Load_Encoder_Lines(self) -> float:
        """
        Get the encoder lines of the load.
        
        Returns:
            The encoder lines of the load
        """
        packet = self._build_packet(self.command_codes['get_cfg_Load_Encoder_Lines'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} encoder lines of the load: {response_data}")
            return response_data
        return None
    
    def set_cfg_Load_Encoder_Lines(self, load_encoder_lines: float) -> bool:
        """
        Set the encoder lines of the load.
        
        Args:
            load_encoder_lines: Encoder lines of the load
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Load_Encoder_Lines'], load_encoder_lines, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} encoder lines of the load set to {load_encoder_lines}")
            return True
        return False

    def get_cfg_Encoder_Lines(self) -> float:
        """
        Get the encoder lines.
        
        Returns:
            The encoder lines
        """

        packet = self._build_packet(self.command_codes['get_cfg_Encoder_Lines'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} encoder lines: {response_data}")
            return response_data
        return None

    def set_cfg_Encoder_Lines(self, encoder_lines: float) -> bool:
        """
        Set the encoder lines.
        
        Args:
            encoder_lines: Encoder lines
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Encoder_Lines'], encoder_lines, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} encoder lines set to {encoder_lines}")
            return True
        return False

    def get_encoder_Location(self) -> str:
        """
        Get the encoder location.
        
        Returns:
            The encoder location
        """
        encoder_location_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }
        encoder_location_dict = {
            0: 'None',
            1: 'Motor',
            2: 'Load',
        }
        packet = self._build_packet(self.command_codes['get_encoder_Location'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} encoder location: {encoder_location_dict[response_data]}")
            return encoder_location_dict[response_data]
        return None
    
    def set_encoder_Location(self, encoder_location: int) -> bool:
        """
        Set the encoder location.
        
        Args:
            encoder_location: Encoder location
        """
        encoder_location_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }
        encoder_location_dict = {
            0: 'None',
            1: 'Motor',
            2: 'Load',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_encoder_Location'], encoder_location_data[encoder_location], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} encoder location set to {encoder_location_dict[encoder_location]}")
            return True
        return False

    #Communication info
    def get_cfp_PC_COM(self) -> bool:
        """
        Get from controller the PC communication type(Serial or Ethernet).
        
        Returns:
            The PC communication type
        """

        pc_com_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
            5: 0x05,
            6: 0x06,
        }
        pc_com_type_dict = {    
            0: 'None',
            1: 'Ethernet',
            2: 'RS232',
            3: 'RS422',
            4: 'RS485',
            5: 'TTL',
            6: 'SPI',
        }

        packet = self._build_packet(self.command_codes['get_cfg_PC_COM'])
        response = self.driver.send_and_receive(packet)

        if response:
            response_data = self._decode_data(response[7], 'uint8')       
            print(f"Axis {self.axis_number} PC communication type: {pc_com_type_dict[response_data]}")
            return pc_com_type_dict[response_data]
        return False 

    def set_cfp_PC_COM(self, pc_com_type: int) -> bool:
        """
        Set the PC communication type.
        
        Args:
            pc_com_type: PC communication type
        """
        pc_com_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
            5: 0x05,
            6: 0x06,
        }
        pc_com_type_dict = {
            0: 'None',
            1: 'Ethernet',
            2: 'RS232',
            3: 'RS422',
            4: 'RS485',
            5: 'TTL',
            6: 'SPI',
        }
        packet = self._build_packet(self.command_codes['set_cfg_PC_COM'], pc_com_type_data[pc_com_type])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} PC communication type set to {pc_com_type_dict[pc_com_type]}")
            return True
        return False

    def get_cfg_TS_COM_TYPE(self) -> str:
        """
        Get the TS communication type.
        
        Returns:
            The TS communication type
        """
        TS_com_type_dict = {
            1: 'Ethernet',
            2: 'RS232',
            9: 'CAN',
        }
        packet = self._build_packet(self.command_codes['get_cfg_TS_COM_TYPE'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} TS communication type: {TS_com_type_dict[response_data]}")
            return TS_com_type_dict[response_data]
        return None
    
    def set_cfg_TS_COM_TYPE(self, ts_com_type: int) -> bool:
        """
        Set the TS communication type.
        
        Args:
            ts_com_type: TS communication type
        """
        TS_com_type_data = {
            0: 0x01,
            1: 0x02,
            2: 0x09,
        }
        TS_com_type_dict = {
            1: 'Ethernet',
            2: 'RS232',
            9: 'CAN',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_TS_COM_TYPE'], TS_com_type_data[ts_com_type], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} TS communication type set to {TS_com_type_dict[ts_com_type]}")
            return True
        return False

    def get_cfg_Multi_Com(self) -> list:
        """
        Get the multi communication type.

        Returns:
            The list of activated communication types
        """
        multi_com_dict = {
            0: 'Ethernet',
            1: 'RS232',
            2: 'RS422',
            3: 'TTL',
        }
        data_format = 'raw'
        packet = self._build_packet(self.command_codes['get_cfg_Multi_Com'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data([response[7]], data_format)
            formatted_bin = format(response_data[0], '08b')
            activated_communications = []
            for idx, name in multi_com_dict.items():
                if formatted_bin[-(idx + 1)] == '1':  # The rightmost bit is index 0
                    activated_communications.append(name)
            if activated_communications:
                print(f"Axis {self.axis_number} activated communication types: {', '.join(activated_communications)}")
                return activated_communications
            else:
                print(f"Axis {self.axis_number} has no activated communication types.")
                return []
        return None

    def set_cfg_Multi_Com(self, multi_com: list) -> bool:
        """
        Set the multi communication type.

        Args:
            multi_com: List of four boolean values [Ethernet, RS232, RS422, TTL], each being True (on) or False (off)
        """
        if not isinstance(multi_com, list) or len(multi_com) != 4 or not all(isinstance(val, bool) for val in multi_com):
            raise ValueError("multi_com must be a list of exactly 4 boolean values corresponding to [Ethernet, RS232, RS422, TTL].")

        # Convert list of booleans to single byte
        # [Ethernet, RS232, RS422, TTL] -> bits 0,1,2,3 for the low 4 bits
        byte_val = 0
        for idx, val in enumerate(multi_com):
            if val:
                byte_val |= (1 << idx)
        data_format = 'uint8'
        # Send as single byte in list to match how get_cfg_Multi_Com parses it
        multi_com_data = [byte_val]
        packet = self._build_packet(self.command_codes['set_cfg_Multi_Com'], multi_com_data, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            status_str_list = []
            label_dict = {0: 'Ethernet', 1: 'RS232', 2: 'RS422', 3: 'TTL'}
            for idx, enabled in enumerate(multi_com):
                status_str_list.append(f"{label_dict[idx]}={'ON' if enabled else 'OFF'}")
            print(f"Axis {self.axis_number} multi communication type set to: {', '.join(status_str_list)}")
            return True
        return False

    def get_cfg_Can_Baud_rate(self) -> int:
        """
        Get the CAN baud rate.
        
        Returns:
            The CAN baud rate
        """ 
        can_baud_rate_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
        }
        can_baud_rate_dict = {
            0: 'None',
            1: '125K',
            2: '250K',
            3: '500K',
            4: '1M',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Can_Baud_rate'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} CAN baud rate: {can_baud_rate_dict[response_data]}")
            return can_baud_rate_dict[response_data]
        return None
    
    def set_cfg_Can_Baud_rate(self, can_baud_rate: int) -> bool:
        """
        Set the CAN baud rate.
        
        Args:
            can_baud_rate: CAN baud rate
        """
        can_baud_rate_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
        }
        can_baud_rate_dict = {
            0: 'None',
            1: '125K',
            2: '250K',
            3: '500K',
            4: '1M',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Can_Baud_rate'], can_baud_rate_data[can_baud_rate], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} CAN baud rate set to {can_baud_rate_dict[can_baud_rate]}")
            return True
        return False

    #biss configuration
    def get_cfg_Biss_Resolution(self) -> int:
        """
        Get the BISS resolution.
        
        Returns:
            The BISS resolution
        """
        packet = self._build_packet(self.command_codes['get_Biss_Resolution'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} BISS resolution: {response_data}")
            return response_data
        return None
    
    def set_cfg_Biss_Resolution(self, biss_resolution: int) -> bool:
        """
        Set the BISS resolution.
        
        Args:
            biss_resolution: BISS resolution
        """
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_Biss_Resolution'], biss_resolution, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} BISS resolution set to {biss_resolution}")
            return True
        return False

    def get_cfg_Biss_Com(self) -> str:
        """
        Get the BISS communication type.
        
        Returns:
            The BISS communication type
        """
        biss_com_dict = {
            0: 'None',
            1: 'SPI1',
            2: 'SPI2',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['get_Biss_Com'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} BISS communication type: {biss_com_dict[response_data]}")
            return biss_com_dict[response_data]
        return None
    
    def set_cfg_Biss_Com(self, biss_com: int) -> bool:
        """
        Set the BISS communication type.
        
        Args:
            biss_com: BISS communication type
        """
        biss_com_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
        }
        biss_com_dict = {
            0: 'None',
            1: 'SPI1',
            2: 'SPI2',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_Biss_Com'], biss_com_data[biss_com], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} BISS communication type set to {biss_com_dict[biss_com]}")
            return True
        return False

    def get_cfg_Abs_Enc_Offset(self) -> float:
        """
        Get the absolute encoder offset.
        
        Returns:
            The absolute encoder offset
        """
        packet = self._build_packet(self.command_codes['get_Abs_Enc_Offset'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} absolute encoder offset: {response_data}")
            return response_data
        return None
    
    def set_cfg_Abs_Enc_Offset(self, abs_enc_offset: float) -> bool:
        """
        Set the absolute encoder offset.
        
        Args:
            abs_enc_offset: Absolute encoder offset
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_Abs_Enc_Offset'], abs_enc_offset, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} absolute encoder offset set to {abs_enc_offset}")
            return True
        return False

    def get_cfg_Abs_Enc_Reverse(self) -> str:
        """
        Get the absolute encoder reverse.
        
        Returns:
            The absolute encoder reverse (Yes/No)
        """
        abs_enc_reverse_dict = {
            0: 'No',
            1: 'Yes',
        }
        packet = self._build_packet(self.command_codes['get_Abs_Enc_Reverse'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} absolute encoder reverse: {abs_enc_reverse_dict[response_data]}")
            return abs_enc_reverse_dict[response_data]
        return None
    
    def set_cfg_Abs_Enc_Reverse(self, abs_enc_reverse: int) -> bool:
        """
        Set the absolute encoder reverse.
        
        Args:
            abs_enc_reverse: Absolute encoder reverse
        """
        abs_enc_reverse_data = {
            0: 0x00,
            1: 0x01,
        }
        abs_enc_reverse_dict = {
            0: 'No',
            1: 'Yes',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_Abs_Enc_Reverse'], abs_enc_reverse_data[abs_enc_reverse], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} absolute encoder reverse set to {abs_enc_reverse_dict[abs_enc_reverse]}")
            return True
        return False

    #limits motion parameters
    def get_cfg_Max_Speed(self) -> float:
        """
        Get the maximum speed.
        
        Returns:
            The maximum speed
        """
        packet = self._build_packet(self.command_codes['get_cfg_Max_Speed'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')       
            print(f"Axis {self.axis_number} maximum speed: {response_data}")
            return response_data
        return None
    
    def set_cfg_Max_Speed(self, max_speed: float) -> bool:
        """
        Set the maximum speed.
        
        Args:
            max_speed: Maximum speed
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Max_Speed'], max_speed, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} maximum speed set to {max_speed}")
            return True
        return False

    def get_cfg_Min_Speed(self) -> float:
        """
        Get the minimum speed.
        
        Returns:
            The minimum speed
        """
        packet = self._build_packet(self.command_codes['get_cfg_Min_Speed'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} minimum speed: {response_data}")
            return response_data
        return None
    
    def set_cfg_Min_Speed(self, min_speed: float) -> bool:
        """
        Set the minimum speed.
        
        Args:
            min_speed: Minimum speed
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Min_Speed'], min_speed, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} minimum speed set to {min_speed}")
            return True
        return False

    def get_cfg_max_Acceleration(self) -> float:
        """
        Get the maximum acceleration.
        
        Returns:    
            The maximum acceleration
        """
        packet = self._build_packet(self.command_codes['get_cfg_max_Acceleration'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:9], 'uint16')
            print(f"Axis {self.axis_number} maximum acceleration: {response_data}")
            return response_data
        return None
    
    def set_cfg_max_Acceleration(self, max_acceleration: float) -> bool:
        """
        Set the maximum acceleration.   
        
        Args:
            max_acceleration: Maximum acceleration
        """
        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['set_cfg_max_Acceleration'], max_acceleration, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} maximum acceleration set to {max_acceleration}")
            return True
        return False

    def get_cfg_Min_Acceleration(self) -> float: 
        """
        Get the minimum acceleration.
        
        Returns:
            The minimum acceleration
        """
        packet = self._build_packet(self.command_codes['get_cfg_Min_Acceleration'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'float32')
            print(f"Axis {self.axis_number} minimum acceleration: {response_data}")
            return response_data
        return None
    
    def set_cfg_Min_Acceleration(self, min_acceleration: float) -> bool:
        """
        Set the minimum acceleration.
        
        Args:
            min_acceleration: Minimum acceleration
        """
        data_format = 'float32'
        packet = self._build_packet(self.command_codes['set_cfg_Min_Acceleration'], min_acceleration, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} minimum acceleration set to {min_acceleration}")
            return True
        return False

    #IMU configuration
    def get_cfg_Sensor_Type(self) -> str:
        """
        Get the IMU type.
        
        Returns:
            The IMU type
        """
        sensor_type_dict = {
            0: 'None',
            1: 'FOG',
            2: 'VN_100',
            3: 'VN_200',
            4: 'VN_300',
            5: 'Winner',
            6: 'Compass',
            7: 'Gladiator',
            8: 'A.Navigation',
        }
        
        packet = self._build_packet(self.command_codes['get_cfg_Sensor_Type'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data_sensor_type = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} Sensor type: {sensor_type_dict[response_data_sensor_type]}")
            response_data_sensor_type = self._decode_data(response[4], 'uint8')
            return sensor_type_dict[response_data_sensor_type]
        return None
    
    def set_cfg_Sensor_Type(self, sensor_type: int) -> bool:
        """
        Set the Sensor type.
        
        Args:
            sensor_type: Sensor type
        """
        sensor_type_dict = {
            0: 'None',
            1: 'FOG',
            2: 'VN_100',
            3: 'VN_200',
            4: 'VN_300',
            5: 'Winner',
            6: 'Compass',
            7: 'Gladiator',
            8: 'A.Navigation',
        }
        

        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Sensor_Type'], sensor_type, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} Sensor type set to {sensor_type_dict[sensor_type]}")
            return True
        return False

    def get_cfg_reversed_IMU(self) -> bool:
        """
        Get the reversed IMU.
        
        Returns:
            The reversed IMU
        """
        reversed_IMU_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        reversed_IMU_dict = {
            0: 'client',
            1: 'base_imu',
            2: 'load_imu',
            3: 'mid_imu',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Reversed_IMU'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} reversed IMU: {reversed_IMU_dict[response_data]}")
            return reversed_IMU_dict[response_data]
        return None
    
    def set_cfg_reversed_IMU(self, reversed_IMU: int) -> bool:
        """
        Set the reversed IMU.
        
        Args:
            reversed_IMU: Reversed IMU
        """
        reversed_IMU_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        reversed_IMU_dict = {
            0: 'client',
            1: 'base_imu',
            2: 'load_imu',
            3: 'mid_imu',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Reversed_IMU'], reversed_IMU_data[reversed_IMU], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} reversed IMU set to {reversed_IMU_dict[reversed_IMU]}")
            return True
        return False

    def get_cfg_Imu_Com(self) -> str:
        """
        Get the IMU communication type.
        
        Returns:
            The IMU communication type
        """
        imu_com_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
            5: 0x05,
            6: 0x06,
        }
        imu_com_dict = {
            0: 'None',
            1: 'RS232_1',
            2: 'RS232_2',
            3: 'RS232_3',
            4: 'RS485',
            5: 'RS422',
            6: 'TTL',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['get_cfg_Imu_Com'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} IMU communication type: {imu_com_dict[response_data]}")
            return imu_com_dict[response_data]
        return None
    
    def set_cfg_Imu_Com(self, imu_com: int) -> bool:
        """
        Set the IMU communication type.
        
        Args:
            imu_com: IMU communication type
        """
        imu_com_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
            4: 0x04,
            5: 0x05,
            6: 0x06,
        }    
        imu_com_dict = {
            0: 'None',
            1: 'RS232_1',
            2: 'RS232_2',
            3: 'RS232_3',
            4: 'RS485',
            5: 'RS422',
            6: 'TTL',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Imu_Com'], imu_com_data[imu_com], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} IMU communication type set to {imu_com_dict[imu_com]}")
            return True
        return False

    def get_cfg_Imu_Baud_Rate(self) -> float:
        """
        Get the IMU baud rate.
        
        Returns:
            The IMU baud rate
        """
        packet = self._build_packet(self.command_codes['get_cfg_Imu_Baud_Rate'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'uint32')
            print(f"Axis {self.axis_number} IMU baud rate: {response_data}")
            return response_data
        return None
    
    def set_cfg_Imu_Baud_Rate(self, imu_baud_rate: int) -> bool:
        """
        Set the IMU baud rate.
        
        Args:
            imu_baud_rate: IMU baud rate
        """
        data_format = 'uint32'
        packet = self._build_packet(self.command_codes['set_cfg_Imu_Baud_Rate'], imu_baud_rate, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} IMU baud rate set to {imu_baud_rate}")
            return True
        return False

    def get_cfg_Imu_Freq(self) -> float:
        """
        Get the IMU frequency.
        
        Returns:
            The IMU frequency
        """
        packet = self._build_packet(self.command_codes['get_cfg_Imu_Freq'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:9], 'uint16')
            print(f"Axis {self.axis_number} IMU frequency: {response_data}")
            return response_data
        return None
     
    def set_cfg_Imu_Freq(self, imu_freq: int) -> bool:
        """
        Set the IMU frequency.
        
        Args:
            imu_freq: IMU frequency
        """
        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['set_cfg_Imu_Freq'], imu_freq, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} IMU frequency set to {imu_freq}")
            return True
        return False

    #General configuration
    def get_cfg_Reversed_Axis(self) -> str:
        """
        Get the reversed axis.
        
        Returns:
            The reversed axis (Yes/No)
        """
        reversed_axis_dict = {
            0: 'No',
            1: 'Yes',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Reversed_Axis'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} reversed axis: {reversed_axis_dict[response_data]}")
            return reversed_axis_dict[response_data]
        return None
    
    def set_cfg_Reversed_Axis(self, reversed_axis: int) -> bool:
        """
        Set the reversed axis.
        
        Args:
            reversed_axis: Reversed axis (Yes/No)
        """
        reversed_axis_dict = {
            0: 'No',
            1: 'Yes',
        }
        reversed_axis_data = {
            0: 0x00,
            1: 0x01,
        }
        data_format = 'uint8'   
        packet = self._build_packet(self.command_codes['set_cfg_Reversed_Axis'], reversed_axis_data[reversed_axis], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} reversed axis set to {reversed_axis_dict[reversed_axis]}")
            return True
        return False

    def get_cfg_System_Type(self) -> str:
        """
        Get the system type.
        
        Returns:
            The system type
        """
        system_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        system_type_dict = {
            0: 'manual',
            1: 'stabilized',
            2: 'tracker',
            3: 'dual_gimbal',
        }
        packet = self._build_packet(self.command_codes['get_cfg_System_Type'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} system type: {system_type_dict[response_data]}")
            return system_type_dict[response_data]
        return None
    
    def set_cfg_System_Type(self, system_type: int) -> bool:
        """
        Set the system type.
        
        Args:
            system_type: System type
        """
        system_type_data = {
            0: 0x00,
            1: 0x01,
            2: 0x02,
            3: 0x03,
        }
        system_type_dict = {
            0: 'manual',
            1: 'stabilized',
            2: 'tracker',
            3: 'dual_gimbal',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_System_Type'], system_type_data[system_type], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} system type set to {system_type_dict[system_type]}")
            return True
        return False

    def get_cfg_firmware_Version(self) -> str: #special case for firmware version get string
        """
        Get the firmware version.
        
        Returns:
            The firmware version
        """
        packet = self._build_packet(self.command_codes['get_cfg_Firmware_Version'])
        response = self.driver.send_and_receive(packet)
        if response:
            fw = response[7:27].decode('utf-8', errors='ignore')            
            print(f"Axis {self.axis_number} firmware version: {fw}")
            return fw
        return None

    def get_cfg_Serial_Number(self) -> int:
        """
        Get the serial number.
        
        Returns:
            The serial number
        """
        packet = self._build_packet(self.command_codes['get_cfg_Serial_Number'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'uint32')       
            print(f"Axis {self.axis_number} serial number: {response_data}")
            return response_data
        return None
    
    def set_cfg_Serial_Number(self, serial_number: str) -> bool:
        """
        Set the serial number.
        
        Args:
            serial_number: Serial number
        """
        data_format = 'uint32'
        packet = self._build_packet(self.command_codes['set_cfg_Serial_Number'], serial_number, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} serial number set to {serial_number}")
            return True
        return False

    def save_cfg_Serial_Number(self) -> bool:
        """
        Save the serial number.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['save_cfg_Serial_Number'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} serial number saved")
            return True
        return False

    def cfg_save(self) -> bool:
        """
        Save the configuration.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['cfg_save'])
        response = self.driver.send_and_receive(packet, sleep_time=3)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} configuration saved")
            return True
        return False
    
    def cfg_load(self) -> bool:
        """
        Load the configuration.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['cfg_load'])
        response = self.driver.send_and_receive(packet, sleep_time=1)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} configuration loaded")
            return True
        return False

    def cfg_restore_default(self) -> bool:
        """
        Restore the default configuration.
        
        Returns:
            True if successful, False otherwise
        """
        packet = self._build_packet(self.command_codes['cfg_restore_default'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} configuration restored to default")
            return True
        return False
    
    def get_cfg_Com_Axes_TS(self) -> int:
        """
        Get the communication type of the axes to the TS.
        
        Returns:
            The communication type of the axes to the TS
        """
        packet = self._build_packet(self.command_codes['get_cfg_Com_Axes_TS'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} communication type of the axes to the TS: {response_data}")
            return response_data
        return None
    
    def set_cfg_Com_Axes_TS(self, com_axes_ts: int) -> bool:
        """
        Set the communication type of the axes to the TS.
        
        Args:
            com_axes_ts: Communication type of the axes to the TS
        """
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Com_Axes_TS'], com_axes_ts, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} communication type of the axes to the TS set to {com_axes_ts}")
            return True
        return False

    def get_cfg_Calc_Aspd(self) -> str:
        """
        Get the calculated acceleration.
        
        Returns:
            The calculated acceleration
        """
        calc_aspd_dict = {
            0: 'Disabled',
            1: 'Enabled',
        }
        packet = self._build_packet(self.command_codes['get_cfg_Calc_Aspd'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7], 'uint8')
            print(f"Axis {self.axis_number} calculated acceleration: {calc_aspd_dict[response_data]}")
            return calc_aspd_dict[response_data]    
        return None
    
    def set_cfg_Calc_Aspd(self, calc_aspd: int) -> bool:
        """
        Set the calculated acceleration.
        
        Args:
            calc_aspd: Calculated acceleration (Disabled/Enabled)
        """
        calc_aspd_data = {
            0: 0x00,
            1: 0x01,
        }
        calc_aspd_dict = {
            0: 'Disabled',
            1: 'Enabled',
        }
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['set_cfg_Calc_Aspd'], calc_aspd_data[calc_aspd], data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} calculated acceleration set to {calc_aspd_dict[calc_aspd]}")
            return True
        return False

    def get_cfg_Num_Of_Axes(self) -> int:
        """
        Get the number of axes.
        
        Returns:
            The number of axes
        """
        num_of_axes_dict = {
            0: 'Pan',
            1: 'Tilt',
            2: 'Roll',
            3: 'Pan_2_X',
            4: 'Tilt_2_Y',
            5: 'Roll_2_Z',
        }
        data_format = 'raw'
        packet = self._build_packet(self.command_codes['get_cfg_Num_Of_Axes'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data([response[7]], data_format)
            formatted_bin = format(response_data[0], '08b')
            num_of_axes = []
            for idx, name in num_of_axes_dict.items():
                if formatted_bin[-(idx + 1)] == '1':  # The rightmost bit is index 0
                    num_of_axes.append(name)
            if num_of_axes:
                print(f"Axis {self.axis_number} number of axes: {', '.join(num_of_axes)}")
                return num_of_axes
            else:
                print(f"Axis {self.axis_number} has no number of axes.")
                return []
        return None
    
    def set_cfg_Num_Of_Axes(self, num_of_axes: list) -> bool:
        """
        Set the number of axes.

        Args:
            num_of_axes: List of six boolean values [Pan, Tilt, Roll, Pan_2_X, Tilt_2_Y, Roll_2_Z],
                each being True (on) or False (off)
        """
        if not isinstance(num_of_axes, list) or len(num_of_axes) != 6 or not all(isinstance(val, bool) for val in num_of_axes):
            raise ValueError("num_of_axes must be a list of exactly 6 booleans corresponding to [Pan, Tilt, Roll, Pan_2_X, Tilt_2_Y, Roll_2_Z].")

        # Convert list of booleans to single byte
        # [Pan, Tilt, Roll, Pan_2_X, Tilt_2_Y, Roll_2_Z] -> bits 0,1,2,3,4,5 for the low 6 bits
        byte_val = 0
        for idx, val in enumerate(num_of_axes):
            if val:
                byte_val |= (1 << idx)
        data_format = 'uint8'
        # Send as single byte in list to match how get_cfg_Num_Of_Axes parses it
        num_of_axes_data = [byte_val]
        packet = self._build_packet(self.command_codes['set_cfg_Num_Of_Axes'], num_of_axes_data, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            status_str_list = []
            label_dict = {0: 'Pan', 1: 'Tilt', 2: 'Roll', 3: 'Pan_2_X', 4: 'Tilt_2_Y', 5: 'Roll_2_Z'}
            for idx, enabled in enumerate(num_of_axes):
                status_str_list.append(f"{label_dict[idx]}={'ON' if enabled else 'OFF'}")
            print(f"Axis {self.axis_number} number of axes set to: {', '.join(status_str_list)}")
            return True
        return False
    
    # === IP communication Commands ===

    def GetControllerIP(self) -> str:
        """
        Get the controller IP.
        
        Returns:
            The controller IP
        """
        packet = self._build_packet(self.command_codes['Get_Controller_IP'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'raw')
            print(f"Axis {self.axis_number} controller IP: {response_data}")
            return response_data
        return None
    
    def SetControllerIP(self, controller_ip: list) -> bool:
        """
        Set the controller IP.
        
        Args:
            controller_ip: Controller IP
        """
        #hex_controller_ip = [f"{x:02x}" for x in controller_ip]
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['Set_Controller_IP'], controller_ip, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} controller IP set to {controller_ip}")
            return True
        return False

    def GetControllerPort(self) -> int:
        """
        Get the controller port.
        
        Returns:
            The controller port
        """
        packet = self._build_packet(self.command_codes['Get_Controller_Port'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:9], 'uint16')
            print(f"Axis {self.axis_number} controller port: {response_data}")
            return response_data
        return None
    
    def SetControllerPort(self, controller_port: int) -> bool:
        """
        Set the controller port.
        
        Args:
            controller_port: Controller port
        """
        data_format = 'uint16'
        packet = self._build_packet(self.command_codes['Set_Controller_Port'], controller_port, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} controller port set to {controller_port}")
            return True
        return False

    def GetControllerSubnetMask(self) -> str:
        """
        Get the controller subnet mask.
        
        Returns:
            The controller subnet mask
        """
        packet = self._build_packet(self.command_codes['Get_Controller_Subnet_Mask'])
        response = self.driver.send_and_receive(packet)
        if response:
            response_data = self._decode_data(response[7:11], 'raw')
            print(f"Axis {self.axis_number} controller subnet mask: {response_data}")
            return response_data
        return None
    
    def Set_Controller_Subnet_Mask(self, controller_subnet_mask: str) -> bool:
        """
        Set the controller subnet mask.
        
        Args:
            controller_subnet_mask: Controller subnet mask
        """
        data_format = 'uint8'
        packet = self._build_packet(self.command_codes['Set_Controller_Subnet_Mask'], controller_subnet_mask, data_format)
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} controller subnet mask set to {controller_subnet_mask}")
            return True
        return False

    def SaveIP(self) -> bool:
        """
        Save the IP.
        
        Returns:
            The IP
        """
        packet = self._build_packet(self.command_codes['Save_IP'])
        response = self.driver.send_and_receive(packet)
        if response == self.Ack_response:
            print(f"Axis {self.axis_number} IP saved")
            return True
        return False 