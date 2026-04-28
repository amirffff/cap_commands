import struct
import ipaddress
import json
import numpy as np

class PacketConstructor:
    # @staticmethod
    # def calculate_checksum(packet):
    #     try:
    #         # Ensure packet is a hex string if it's a bytearray
    #         if isinstance(packet, bytearray):
    #             packet = packet.hex().upper() 
    #         elif isinstance(packet, list):
    #             # Join all elements of the list as hex strings
    #             packet = ''.join([f"{item:02X}" if isinstance(item, int) else item.upper() for item in packet])

    #         total = 0
    #         length = int(packet[4:6], 16)  # Extract length as int

    #         # Calculate where payload ends (right before checksum)
    #         payload_end_index = 6 + (length - 1) * 2  # -1 to exclude checksum

    #         # Iterate over each byte in the range and sum their values
    #         for i in range(4, payload_end_index, 2):  # stop BEFORE checksum
    #             total += int(packet[i:i + 2], 16)

    #         # Keep only the lowest byte (mod 256)
    #         checksum = total % 256
    #         # Return as a 2-character uppercase hex string
    #         return f"{checksum:02X}"

    #     except Exception as e:
    #         print(f"Error calculating checksum: {e}")
    #         raise ValueError

    @staticmethod
    def calculate_checksum(packet):
        try:
            # Ensure packet is a hex string if it's a bytearray
            if isinstance(packet, bytearray):
                packet = packet.hex().upper() 
            elif isinstance(packet, list):
                # Join all elements of the list as hex strings
                packet = ''.join([f"{item:02X}" if isinstance(item, int) else item.upper() for item in packet])

            total = 0
            length = int(packet[4:6], 16)  # Extract length as int
            # Iterate over each byte in the range and sum their values
            for i in range(4, 6 + length * 2, 2):  # *2 because each byte is represented by two hex characters
                total += int(packet[i:i + 2], 16)
            # Keep only the lowest byte (mod 256)
            checksum = total % 256
            # Return as a 2-character uppercase hex string
            return f"{checksum:02X}"

        except Exception as e:
            print(f"Error calculating checksum: {e}")
            raise ValueError
            # return '00'
        
    @staticmethod
    def unpack_float32(data):
        # print('in unpack float 32')
        """Unpack a 32-bit floating point from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!f', bytes_data)[0]
    
    @staticmethod
    def unpack_uint32(data):
        """Unpack an unsigned 32-bit integer from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!I', bytes_data)[0]
    
    @staticmethod
    def unpack_uint16(data):
        """Unpack an unsigned 16-bit integer from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!H', bytes_data)[0]
    
    @staticmethod
    def unpack_int8(data):
        """Unpack a signed 8-bit integer from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!b', bytes_data)[0]
    
    @staticmethod
    def unpack_uint8(data):
        """Unpack an unsigned 8-bit integer from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!B', bytes_data)[0]
    
    @staticmethod 
    def unpack_ascii_string(data):
        """Unpack an ASCII string from hex string"""
        bytes_data = bytes.fromhex(data)
        return bytes_data.decode('ascii')

    @staticmethod
    def unpack_ip(data):
        # Split into 4 bytes and convert to decimal
        octets = [int(data[i:i+2], 16) for i in range(0, len(data), 2)]
        # Convert to IP format (e.g., '192.168.1.1')
        ip_address = '.'.join(str(octet) for octet in octets)
        return ip_address
    
    @staticmethod
    def pack_ip(ip):
        """Pack an IP address string into a 4-byte hex string"""
        # Split the IP into 4 octets and convert each to 2-byte hex
        octets = ip.split('.')
        hex_data = ''.join(f'{int(octet):02x}' for octet in octets)
        return hex_data
    
    @staticmethod
    def pack_float16(value):
        """Pack a 16-bit floating point into a hex string (IEEE 754 half-precision)"""
        bytes_data = np.float16(value).tobytes()
        return bytes_data.hex()

    @staticmethod
    def unpack_float16(data):
        """Unpack a 16-bit floating point from a hex string (IEEE 754 half-precision)"""
        bytes_data = bytes.fromhex(data)
        return np.frombuffer(bytes_data, dtype=np.float16)[0].item()
    
    @staticmethod
    def pack_float32(value):
        """Pack a 32-bit floating point into a hex string"""
        bytes_data = struct.pack('!f', value)
        return bytes_data.hex()

    @staticmethod
    def pack_uint32(value):
        """Pack an unsigned 32-bit integer into a hex string"""
        bytes_data = struct.pack('!I', value)
        return bytes_data.hex()

    @staticmethod
    def pack_uint16(value):
        """Pack an unsigned 16-bit integer into a hex string"""
        bytes_data = struct.pack('!H', value)
        return bytes_data.hex()

    @staticmethod
    def pack_int8(value):
        """Pack a signed 8-bit integer into a hex string"""
        bytes_data = struct.pack('!b', value)
        return bytes_data.hex()

    @staticmethod
    def pack_uint8(value):
        """Pack an unsigned 8-bit integer into a hex string"""
        bytes_data = struct.pack('!B', value)
        return bytes_data.hex()
    
    @staticmethod
    def pack_int16(value):
        """Pack a signed 16-bit integer into a hex string"""
        bytes_data = struct.pack('!h', value)
        return bytes_data.hex()

    @staticmethod
    def unpack_int16(data):
        """Unpack a signed 16-bit integer from hex string"""
        bytes_data = bytes.fromhex(data)
        return struct.unpack('!h', bytes_data)[0]
    
    @staticmethod 
    def pack_ascii_string(value):
        """Pack an ASCII string into a hex string"""
        bytes_data = value.encode('ascii')
        return bytes_data.hex()
    
    @staticmethod
    def is_valid_ip(ip):
        try:
            ipaddress.ip_address(ip)
            return True
        except ValueError:
            return False
        
    @staticmethod
    def calculate_packet_length(packet):
        """Calculate length of the packet excluding the first 3 bytes and last byte, return as hex string."""
        try:
            if isinstance(packet, str):
                packet = bytearray.fromhex(packet)
            elif isinstance(packet, list):
                packet = bytearray(packet)

            if len(packet) < 4:
                return "00"  

            length = len(packet[3:-1])

            return f"{length:02X}"

        except Exception as e:
            print(f"Error calculating length: {e}")
            return "00"  

    @staticmethod
    def exceeded_id(id):
        return id % 256
    
    @staticmethod
    def extract_data(hex_str):
        try:
            data_index = 3
            constant_data = 4 
            packet = bytearray.fromhex(hex_str)
            data_length_hex = packet[2:3] 
            data_length = int(data_length_hex.hex(), 16)  
            data_start_index = data_index 
            data_end_index = data_start_index + data_length  
            data = packet[data_start_index+constant_data:data_end_index]  
            return data.hex()

        except Exception as e:
            print(f"Error extracting data: {e}, hex str: {hex_str}")
            return None
    
    @staticmethod
    def extract_full_data(hex_str):
        try:
            # Convert the hex string into a bytearray
            packet = bytearray.fromhex(hex_str)

            # Extract the length (3rd byte) from index 3
            data_length_hex = packet[2:3]  # The length is at index 2 (3rd byte in 0-based index)
            data_length = int(data_length_hex.hex(), 16)  # Convert the length to an integer

            # Extract the data from index 4 onwards based on the length
            data_start_index = 3  # The 4th byte is at index 3 (0-based index)
            data_end_index = data_start_index + data_length  # The end index is length bytes after start

            data = packet[data_start_index:data_end_index]  # Extract the data bytes

            # Return the extracted data as a hexadecimal string
            return data.hex()

        except Exception as e:
            print(f"Error extracting full data: {e}")
            return None
    
    @staticmethod
    def insert_target_degree(id=0, classification=0, confidance=100, error_pan=0, error_tilt=0):
        if 0 <= confidance <= 1:
            print(confidance)
            confidance = int(confidance*100)
            print(confidance)
        if id>256:
            id = PacketConstructor.exceeded_id(id)

        converted_error_pan = PacketConstructor.pack_float32(error_pan)
        converted_error_tilt = PacketConstructor.pack_float32(error_tilt)
        converted_id = PacketConstructor.pack_uint8(id)
        converted_classification = PacketConstructor.pack_uint8(classification)
        converted_confidance = PacketConstructor.pack_uint8(confidance)
        pkt_extention = converted_id + converted_classification + converted_confidance + converted_error_pan + converted_error_tilt
        return pkt_extention
    
    #with x1,y1,x2,y2 error
    @staticmethod
    def insert_target_pixel(id=0, classification=0, confidance=100, x1=0, y1=0, x2=0, y2=0):
        try:
            if 0 <= confidance <= 1:
                confidance = int(confidance*100)
            # if id>256:
            #     id = PacketConstructor.exceeded_id(id)
            id = id % 256
            converted_x1 = PacketConstructor.pack_uint16(x1)
            converted_y1 = PacketConstructor.pack_uint16(y1)
            converted_x2 = PacketConstructor.pack_uint16(x2)
            converted_y2 = PacketConstructor.pack_uint16(y2)
            converted_id = PacketConstructor.pack_uint8(id)
            converted_classification = PacketConstructor.pack_uint8(classification)
            converted_confidance = PacketConstructor.pack_uint8(confidance)
            pkt_extention = converted_id + converted_classification + converted_confidance + converted_x1 + converted_y1 + converted_x2 + converted_y2
            # print('ext:', pkt_extention)
            return pkt_extention
        except Exception as e:
            print('e in insert data packet: ', e, id, classification, confidance, x1, y1, x2, y2)
    
    @staticmethod
    def construct_answer_packet(pkt, length, opcode, data_to_send, camera_id=0, axis_group_id=0):
        #bytearrey
        try:
            checksum = '00'
            # print('construct_answer_packet')
            pkt[2] = length if isinstance(length, int) else int(length, 16)
            pkt[3] = camera_id if isinstance(camera_id, int) else int(camera_id, 16)
            pkt[4] = axis_group_id if isinstance(axis_group_id, int) else int(axis_group_id, 16)
            pkt[-1:-1] = bytearray.fromhex(data_to_send)
            pkt[5:7] = bytearray.fromhex(opcode)
            try:
                checksum = PacketConstructor.calculate_checksum(pkt)
            except Exception as e:
                print('e: ', e)
            # if checksum is None:
            #     return checksum
            # pkt[-1] = int(checksum, 16)
            pkt[-1] = checksum if isinstance(checksum, int) else int(checksum, 16)
            return pkt
        except Exception as e:
            print('e construct_answer_packet: ', e)
            return 'E6' 
    
    @staticmethod
    def construct_data_packet(prefix, length,
                            opcode, data_to_send, 
                            camera_id=0, axis_group_id=0):
        #str
        try:
            # print('construct_data_packet')
            checksum = '00'
            camera_id = PacketConstructor.pack_int8(camera_id)
            axis_group_id = PacketConstructor.pack_int8(axis_group_id)
            packet = prefix+length+camera_id+axis_group_id+opcode+data_to_send+checksum
            # print('packet: ', packet)
            # packet = prefix+length+axis_camera_id+camera_group_id+opcode+data_to_send+checksum
            # print('construct_data_packet packet: ', packet, opcode)
            try:
                checksum = PacketConstructor.calculate_checksum(packet)
            except Exception as e:
                print('e checksum: ', e)
            packet = packet[:-2] + checksum
            # print('construct_data_packet packet: ', packet, opcode)
            return packet
        except Exception as e:
            print('e expectation: ', e)
            return 'E6' 
    
    @staticmethod
    def construct_no_data_packet(prefix, length,
                            opcode, camera_id=0, 
                            axis_group_id=0):
        try:
            checksum = '00'
            camera_id = PacketConstructor.pack_int8(camera_id)
            axis_group_id = PacketConstructor.pack_int8(axis_group_id)
            packet = prefix+length+camera_id+axis_group_id+opcode+checksum
            try:
                checksum = PacketConstructor.calculate_checksum(packet)
            except Exception as e:
                print('e: ', e)            # if checksum is None:
            #     return checksum
            packet = packet[:-2] + checksum
            return packet
        except Exception as e:
            print('e: ', e)
            return 'E6' 
    
    @staticmethod
    def pack_float32_list(input):
        # print(input,type(input))
        if input == 0:
            c = ['00', '00', '00', '00']
            return(c)
        else:
            #######STEP 1: convert int to hex##########
            a = hex(struct.unpack('<I', struct.pack('<f', input))[0])
            #######STEP 2: split hex into pairs##########
            a = a[2:] if len(a) % 2 == 0 else "0" + a[2:]
            b = " ".join(a[i:i + 2] for i in range(0, len(a), 2))
            #######STEP 3: convert sliced hex into list ##########
            c = list(b.split(" "))
            return(c) 

        
# print(PacketConstructor.int)