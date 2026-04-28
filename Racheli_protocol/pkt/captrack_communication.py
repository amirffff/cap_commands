
import socket
import struct
import time
from ctypes import *
import numpy as np
import copy
from packet_constructor import PacketConstructor
from collections import deque
# from concurrent.futures import ThreadPoolExecutor

class CaptrackConnection:
    def __init__(self, captrack_ip, captrack_port) -> None:
        print('initiating captrack')
        self.captrack_ip = captrack_ip
        self.captrack_port = captrack_port
        self.client_socket = socket.socket()
        self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.client_socket.connect((self.captrack_ip, self.captrack_port))
        self.default_packet = '5054040000000000'
        self.prefix = '5054'
        self.checksum = '00'
        self.opcodes_answer_prefix = {
            '0A1C': '0', #stream_out_data_pixel
            '0A03': '0', #stream_out_data_degree
            '0103': '0', #position_movement
            '0A16': '0', #set_camera_fov
            '0A18': '0', #set_camera_zoom
            '0A1A': '0', #move_by_pixels 
            '0A01': '0', #set_tracking_on/off
            '0A01': '0', #set_auto_tracking_on/off
            '0A14': '0', #set_camera_resolution
            '0F47': '0', #get_camera_center_enabled 
            '0F35': '0', #get_camera_aim_enabled 
            '0F38': '0', #get_camera_aim_pixels 
            '0F4A': '0', #get_camera_center_pixels 
            '0A02': '0', #get_motion_track_status 
            '0A05': '0', #get_tracking_error 
            '0720': '0', #get_packets_per_sec 
            '0300': '0', #set_lrf_manual_distance 
            '030D': '0', #get_lrf_manual_distance 
            '0108': '0', #get_apos 
            '010A': '0', #get_aspd 
            '0F33': '0', #reset_aim 
            '0F3B': '0', #set_aim_by_px 
            '0F3C': '0', #set_manual_aim_ofset on/off
            '0A28': '0', #get_set_point_px 
            '301D': '0', #axis_off
            '013C': '0', #axis_on
            '013E': '0', #reset_axis
            #ballistic states -------------changes all to 0f3c with data 0-3 86=0 3c=1 84=2
            '0F86': '0', #set_ballistic_multi_calib #0/1
            '0F87': '0', #get_ballistic_multi_calib
            '0F84': '0', #set_ballistic_table #0/1
            '0F85': '0', #get_ballistic_table
            '0F3C': '0', #set_ballistic_manual #0/1
            '0F3D': '0', #get_ballistic_manual

            # changing to-
            '0F3C': '0', #set_ballistic_type 0-2
            '0F3C': '0', #set_ballistic_multi_calib #index 0
            '0F3C': '0', #set_ballistic_manual #index 1
            '0F3C': '0', #set_ballistic_table #index 2
            '0F3D': '0', #get_ballistic_type 0-2
            #ballistics values
            '0F30': '0', #get_aim_degrees
            #---manual offset---
            '0F3E': '0', #set_ballistic_manual_offset_degrees
            '0F3F': '0', #get_ballistic_manual_offset_degrees
            #---balistic table---
            '0300': '0', #set_lrf_manual_distance 
            '030D': '0', #get_lrf_manual_distance 
            '0F80': '0', #set_wind
            '0F81': '0', #get_wind
            '0F82': '0', #set_weapon_type #0-MAG,1-M2,2-M4A1
            '0F83': '0', #get_weapon_type
            #---multi calib---
            '0F84': '0', #get_multi_calib #by index(in the place of axis 0-4) uint16|uint16|uint16 distance|tilt|pan(all needs to *100)
            '0F06': '0', #speed_movement_both_axis
            '0400': '0', #speed_movement
            '0110': '0', #is_active_swls
            '010C': '0', #get_negative_SW
            '010D': '0', #get_positive_Sw
            # '013D': '0', #axes_off
            # '013C': '0', #axes_on
            # '013E': '0', #reset_axes
        }
        self.last_send_time = None

    def send_response(self, response):
        # print('sending: ', response)
        # print('-------------------------------')
        if isinstance(response, str):
            self.client_socket.send(bytes.fromhex(response)) 
        elif isinstance(response, (bytearray, list)):
            self.client_socket.send(bytes(response))
    
    def send_response_wait_response(self, response):
        try:
            if isinstance(response, str):
                self.client_socket.send(bytes.fromhex(response))
            elif isinstance(response, list):
                response = bytes([int(b) if isinstance(b, str) else b for b in response])  # Convert str to int in list
                self.client_socket.send(response)
            elif isinstance(response, bytearray):
                self.client_socket.send(bytes(response))  # Convert bytearray to bytes
            elif isinstance(response, (bytearray, list)):
                self.client_socket.send(bytes(response))

            t = time.time()
            answer = bytes.hex(self.client_socket.recv(4096))
            sum = time.time() - t
            # print('sum: ', sum)
            if sum>1:
                print('sum res ans: ', sum, response , answer)
            # answer = bytes.hex(self.client_socket.recv(100))
            # print('answer from captrack ', answer, type(answer))
            # # print('pkt: ', response.hex().upper())
            # if '06' not in answer :
            #     print('!!!!!!answer:', answer, 'packet send: ', response)
            #     if isinstance(response, bytearray):
            #         response = response.hex().upper() 
            #         print('!!!!!!!!!packet hex: ', response)
            # print('answer: ', answer)
            return answer
        except Exception as e:
            ('failed to send captrack data: ', e)
            
    def stream_out_data_pixel(self, pkt):
            # opcode = 0x0A03 
            checksum = 0x00 
            opcode = 0x0A1C 
            # print('pkt: ', pkt, len(pkt), )
            pkt[5:7] = opcode.to_bytes(2, 'big')  # Convert to 2 bytes and insert it into pkt
            try:
                checksum = PacketConstructor.calculate_checksum(pkt)
            except Exception as e:
                print('e: ', e)
            pkt[-1] = int(checksum, 16)
            answer = self.send_response_wait_response(pkt)
            # self.send_data_exe.submit(self.send_response_wait_response, pkt)
            # if answer != '06':
            # print('pkt: ', pkt)
            # print('ans ', answer, 'stream pixels')
            # print('pkt: ', pkt.hex().upper(), 'stream pixels', answer)
            # self.send_response(pkt)
    
    def stream_out_data_pixel_empty(self, camera_id=0, axis=0):
        opcode = '0A1C' 
        # length = '15'
        length = '0F'
        # data='0000000000000000000000'
        data = PacketConstructor.insert_target_pixel(id=0, classification=0, confidance=0, x1=0, y1=0, x2=0, y2=0)
        # print('data: ', data, type(data))
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('answer: ', answer)
    
    def stream_out_data_degree(self, pkt):
        opcode = 0x0A03 
        checksum = 0x00 
        # opcode = 0x0A1C 
        pkt[5:7] = opcode.to_bytes(2, 'big')  # Convert to 2 bytes and insert it into pkt
        try:
            checksum = PacketConstructor.calculate_checksum(pkt)
        except Exception as e:
            print('e: ', e)
        pkt[-1] = int(checksum, 16)
        answer = self.send_response_wait_response(pkt)
        # print('pkt: ', pkt.hex().upper())
        # self.send_response(pkt)

    def position_movement(self, axis, cpos, cspd, acc, camera_id=0):
        """
        Handles position movement for both pan (axis=1) and tilt (axis=2),
        using the standard PacketConstructor interface.
        Sequence:
            1. MOT_SetTum (0x013F)
            2. MOT_SetPositionRelative (0x0138)
            3. MOT_SetPositionMode (0x013B)
            4. MOT_SetAcceleration (0x0130)
            5. MOT_SetSpeed (0x0131)
            6. MOT_SetPosition (0x0132)
            7. MOT_Update (0x0134)
        """

        print(f'[position_movement] axis={axis} pos={cpos} cspd={cspd} acc={acc}')

        # --- Common opcodes & lengths ---
        no_data_length = '04'
        float_length   = '08'

        # --- Step 1–3: Mode setup ---
        MOT_SetTum = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='013F',
            camera_id=camera_id, axis_group_id=axis)

        MOT_SetPositionRelative = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='0138',
            camera_id=camera_id, axis_group_id=axis)

        MOT_SetPositionMode = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='013B',
            camera_id=camera_id, axis_group_id=axis)

        # --- Step 4–6: Data packets ---
        acc_data  = PacketConstructor.pack_float32(acc)
        cspd_data = PacketConstructor.pack_float32(cspd)
        cpos_data = PacketConstructor.pack_float32(cpos)

        MOT_SetAcceleration = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0130',
            data_to_send=acc_data, camera_id=camera_id, axis_group_id=axis)

        MOT_SetSpeed = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0131',
            data_to_send=cspd_data, camera_id=camera_id, axis_group_id=axis)

        MOT_SetPosition = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0132',
            data_to_send=cpos_data, camera_id=camera_id, axis_group_id=axis)

        # --- Step 7: Update command ---
        MOT_Update = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='0134',
            camera_id=camera_id, axis_group_id=axis)

        # --- Queue and send all packets sequentially ---
        packets = deque([
            MOT_SetTum,
            MOT_SetPositionRelative,
            MOT_SetPositionMode,
            MOT_SetAcceleration,
            MOT_SetSpeed,
            MOT_SetPosition,
            MOT_Update
        ])

        for pkt in packets:
            ans = self.send_response_wait_response(pkt)
            # print(f'→ Sent: {pkt} | Ans: {ans}')

    def speed_movement(self, axis: int, speed: float, acc: float = 120.0, camera_id: int = 0):
        """
        Set speed mode + acceleration + speed for a single axis, then MOT_Update.
        axis: 1 = pan, 2 = tilt
        """
        print(f'[speed_movement] axis={axis} speed={speed} acc={acc}')

        # --- Constants ---
        no_data_length = '04'
        float_length   = '08'

        # --- Step 1–2: mode setup ---
        MOT_SetTum = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='013F',
            camera_id=camera_id, axis_group_id=axis)

        MOT_SetSpeedMode = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='013A',
            camera_id=camera_id, axis_group_id=axis)

        # --- Step 3–4: acceleration and speed ---
        acc_data = PacketConstructor.pack_float32(acc)
        spd_data = PacketConstructor.pack_float32(speed)

        MOT_SetAcceleration = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0130',
            data_to_send=acc_data, camera_id=camera_id, axis_group_id=axis)

        MOT_SetSpeed = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0131',
            data_to_send=spd_data, camera_id=camera_id, axis_group_id=axis)

        # --- Step 5: update ---
        MOT_Update = PacketConstructor.construct_no_data_packet(
            self.prefix, length=no_data_length, opcode='0134',
            camera_id=camera_id, axis_group_id=axis)

        # --- Send sequentially ---
        packets = [MOT_SetTum, MOT_SetSpeedMode, MOT_SetAcceleration, MOT_SetSpeed, MOT_Update]

        for pkt in packets:
            ans = self.send_response_wait_response(pkt)
            print(f'→ Sent: {pkt} | Ans: {ans}')
    
    def speed_movement_both_axis(self, pan_speed=0, tilt_speed=0, acc_pan=120.0, acc_tilt=120.0, camera_id=0):
        """
        Set acceleration per axis and a combined pan/tilt speed in one go.
        Matches PacketConstructor-style command structure.
        """
        print(f'[speed_movement_both_axis] pan={pan_speed} tilt={tilt_speed} acc_pan={acc_pan} acc_tilt={acc_tilt}')

        # --- Lengths ---
        float_length = '08'
        group_length = '08'

        # --- Acceleration packets (per-axis) ---
        acc_pan_data  = PacketConstructor.pack_float32(acc_pan)
        acc_tilt_data = PacketConstructor.pack_float32(acc_tilt)

        MOT_SetAcceleration_pan = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0130',
            data_to_send=acc_pan_data, camera_id=camera_id, axis_group_id=0x01)

        MOT_SetAcceleration_tilt = PacketConstructor.construct_data_packet(
            self.prefix, length=float_length, opcode='0130',
            data_to_send=acc_tilt_data, camera_id=camera_id, axis_group_id=0x02)

        # --- Combined speed packet ---
        pan_u16  = PacketConstructor.pack_int16(int(pan_speed  * 100))
        tilt_u16 = PacketConstructor.pack_int16(int(tilt_speed * 100))
        data = pan_u16 + tilt_u16  # concatenated hex string

        MOT_SetSpeed_both = PacketConstructor.construct_data_packet(
            self.prefix, length=group_length, opcode='0F06',
            data_to_send=data, camera_id=camera_id, axis_group_id=0x00)

        # --- Send packets sequentially ---
        packets = [MOT_SetAcceleration_pan, MOT_SetAcceleration_tilt, MOT_SetSpeed_both]

        for pkt in packets:
            ans = self.send_response_wait_response(pkt)
            print(f'→ Sent: {pkt} | Ans: {ans}')

    def set_camera_fov(self, h_fov, v_fov, camera_id=0, axis=0):
        length = '08'
        opcode = '0A16'
        print('set_camera_fov: ', h_fov, v_fov)
        h_fov = int(h_fov*100)
        v_fov = int(v_fov*100)
        data = PacketConstructor.pack_uint16(h_fov) + PacketConstructor.pack_uint16(v_fov)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_camera_fov', answer, opcode, packet)

    def set_camera_zoom(self, zoom, camera_id=0, axis=0):
        opcode = '0A18' 
        length = '08'
        data = PacketConstructor.pack_float32(zoom)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('[set_camera_zoom]', answer, opcode, zoom)

    def move_by_pixels(self, x, y, camera_id=0, axis=0):
        opcode = '0A1A' 
        length = '08'
        data = PacketConstructor.pack_uint16(x)+PacketConstructor.pack_uint16(y)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print(answer, opcode, packet)
    
    def set_tracking_on(self, data=1, camera_id=0, axis=0):
        opcode = '0A01' 
        length = '05'
        data = PacketConstructor.pack_uint8(data)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_tracking_on, ', packet, answer)
    
    def set_auto_tracking_on(self, data=2, camera_id=0, axis=0):
        opcode = '0A01' 
        length = '05'
        data = PacketConstructor.pack_uint8(data)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_auto_tracking_on, ', packet, answer)
    
    def set_tracking_off(self, data=0, camera_id=0, axis=0):
        opcode = '0A01' 
        length = '05'
        data = PacketConstructor.pack_uint8(data)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_tracking_off, ', packet, answer)
    

    
    def set_camera_resolution(self, width, height, camera_id=0, axis=0):
        opcode = '0A14' 
        length = '08'
        data = PacketConstructor.pack_uint16(width)+ PacketConstructor.pack_uint16(height)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_camera_resolution, ', packet, answer, width, height)

    def get_camera_center_enabled(self, camera_id=0, axis=0):
        opcode = '0F47' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        # print('get_camera_center_enabled', packet, answer)
        try:
            if PacketConstructor.unpack_uint8(data_recievd) == 1:
                return True
        except:
            return False
        return False

    def set_camera_center_on(self, camera_id=0, axis=0, data=1):
        opcode = '0F46' 
        length = '05'
        data = PacketConstructor.pack_uint8(data)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('data: ', data)
        print('set_camera_center', packet, answer)
    
    def set_camera_center_off(self, camera_id=0, axis=0, data=0):
        opcode = '0F46' 
        length = '05'
        data = PacketConstructor.pack_uint8(data)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('data: ', data)
        print('set_camera_center', packet, answer)

    def get_camera_aim_enabled(self, camera_id=0, axis=0):
        opcode = '0F35' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        # print('get_camera_center_enabled', packet, answer)
        try:
            if PacketConstructor.unpack_uint8(data_recievd) == 1:
                return True
        except:
            return False
        return False
    
    def set_camera_aim_on(self, camera_id=0, axis=0):
        opcode = '0F34' 
        length = '05'
        value = 1
        data = PacketConstructor.pack_uint8(value)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
    
    def set_camera_aim_off(self, camera_id=0, axis=0):
        opcode = '0F34' 
        length = '05'
        value = 0
        data = PacketConstructor.pack_uint8(value)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)

    def get_camera_aim_pixels(self, camera_id=0, axis=0):
        opcode = '0F38' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                        opcode=opcode, 
                                                        camera_id=camera_id,
                                                        axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, 'sent:', packet, opcode)
        data_recievd = PacketConstructor.extract_data(answer)

        if data_recievd is None:
            print("[get_camera_aim_pixels] Warning: No data received.")
            return None, None

        if len(data_recievd) < 4:
            print("[get_camera_aim_pixels] Error: Data too short.")
            return None, None

        x = PacketConstructor.unpack_uint16(data_recievd[:4])
        y = PacketConstructor.unpack_uint16(data_recievd[4:8])

        return x, y
    
    def get_camera_center_pixels(self, camera_id=0, axis=0):
        opcode = '0F4A' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('get_camera_center_pixels ans: ', answer, opcode, packet)
        data_recievd = PacketConstructor.extract_data(answer)
        x, y = PacketConstructor.unpack_uint16(data_recievd[:4]), PacketConstructor.unpack_uint16(data_recievd[4:])
        return [x,y]

    def get_motion_track_status(self, camera_id=0, axis=0):
        opcode = '0A02' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        status = PacketConstructor.unpack_uint8(data_recievd)
        return status
    
    def get_motion_auto_track(self, camera_id=0, axis=0):
        opcode = '0A04' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        status = PacketConstructor.unpack_uint8(data_recievd)
        return status
      
    def get_tracking_error(self, camera_id=0, axis=0):
        opcode = '0A05' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        data_recievd = PacketConstructor.extract_data(answer)
        error_degrees = PacketConstructor.unpack_float32(data_recievd)
        return error_degrees

    def get_packets_per_sec(self, camera_id=0, axis=0):
        opcode = '0720' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        packet_sum= PacketConstructor.unpack_uint32(data_recievd)
        return packet_sum

    def set_lrf_manual_distance(self, distance, camera_id=0, axis=0):
        opcode = '0300' 
        length = '08'
        data = PacketConstructor.pack_float32(distance)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('set_lrf_manual_distance, ', packet, answer)
    
    def get_lrf_manual_distance(self, camera_id=0, axis=0):
        opcode = '030D' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        lrf_manual_distance = PacketConstructor.unpack_float32(data_recievd)
        return lrf_manual_distance
        
    # --- Internal Ballistics: mode toggles ---------------------------------
    # def set_ballistic_multi_calib(self, enabled=0, camera_id=0, axis=0):
    #     # opcode, length = '0F86', '05'
    #     opcode, length = '0F83C', '05'
    #     # data = PacketConstructor.pack_uint8(1 if enabled else 0)
    #     data = PacketConstructor.pack_uint8(enabled)
    #     pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
    #     return self.send_response_wait_response(pkt)

    # def get_ballistic_multi_calib(self, camera_id=0, axis=0) -> bool:
    #     opcode, length = '0F87', '04'
    #     pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
    #     ans = self.send_response_wait_response(pkt)
    #     data = PacketConstructor.extract_data(ans)
    #     return PacketConstructor.unpack_uint8(data) == 1 if data else False

    # def set_ballistic_table(self, enabled=2, camera_id=0, axis=0):
    #     # opcode, length = '0F84', '05'
    #     opcode, length = '0F3C', '05'
    #     # data = PacketConstructor.pack_uint8(1 if enabled else 0)
    #     data = PacketConstructor.pack_uint8(enabled)
    #     pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
    #     return self.send_response_wait_response(pkt)

    # def get_ballistic_table(self, camera_id=0, axis=0) -> bool:
    #     opcode, length = '0F85', '04'
    #     pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
    #     ans = self.send_response_wait_response(pkt)
    #     data = PacketConstructor.extract_data(ans)
    #     return PacketConstructor.unpack_uint8(data) == 1 if data else False

    # def set_ballistic_manual(self, enabled=1, camera_id=0, axis=0):
    #     opcode, length = '0F3C', '05'
    #     # data = PacketConstructor.pack_uint8(1 if enabled else 0)
    #     data = PacketConstructor.pack_uint8(enabled)
    #     pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
    #     return self.send_response_wait_response(pkt)

    # def get_ballistic_manual(self, camera_id=0, axis=0) -> bool:
    #     opcode, length = '0F3D', '04'
    #     pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
    #     ans = self.send_response_wait_response(pkt)
    #     data = PacketConstructor.extract_data(ans)
    #     return PacketConstructor.unpack_uint8(data) == 1 if data else False
    
    
    # --- Unified Ballistic: set/get ballistic "type" (index) ---------------
    # Protocol: opcode 0x0F3C (set type) and 0x0F3D (get type)
    #   index 0 = multi calibration
    #   index 1 = manual offsets
    #   index 2 = ballistic table

    def set_ballistic_type(self, index, camera_id=0, axis=0):
        """Send opcode 0x0F3C with one uint8 index (0–2)."""
        try:
            idx = int(index) & 0xFF
        except Exception:
            idx = 0
        opcode, length = '0F3C', '05'
        data = PacketConstructor.pack_uint8(idx)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis
        )
        return self.send_response_wait_response(pkt)

    def get_ballistic_type(self, camera_id=0, axis=0):
        """
        Send opcode 0x0F3D and return uint8 index (0–2) or None on failure.
        Returns: int or None
        """
        opcode, length = '0F3D', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length=length, opcode=opcode,
            camera_id=camera_id, axis_group_id=axis
        )
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        if not data:
            return None
        try:
            return PacketConstructor.unpack_uint8(data)
        except Exception:
            return None

    # --- Back-compat thin wrappers (still callable elsewhere) --------------
    def set_ballistic_multi_calib(self, camera_id=0, axis=0):
        return self.set_ballistic_type(0, camera_id=camera_id, axis=axis)

    def set_ballistic_manual(self, camera_id=0, axis=0):
        return self.set_ballistic_type(1, camera_id=camera_id, axis=axis)

    def set_ballistic_table(self, camera_id=0, axis=0):
        return self.set_ballistic_type(2, camera_id=camera_id, axis=axis)

    def get_ballistic_multi_calib(self, camera_id=0, axis=0):
        idx = self.get_ballistic_type(camera_id=camera_id, axis=axis)
        return idx == 0 if idx is not None else False

    def get_ballistic_manual(self, camera_id=0, axis=0):
        idx = self.get_ballistic_type(camera_id=camera_id, axis=axis)
        return idx == 1 if idx is not None else False

    def get_ballistic_table(self, camera_id=0, axis=0):
        idx = self.get_ballistic_type(camera_id=camera_id, axis=axis)
        return idx == 2 if idx is not None else False


    # --- SWLS -------------------
    def is_active_swls(self, camera_id=0, axis=0) -> bool:
        opcode, length = '0110', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        print('data: ', data)
        return PacketConstructor.unpack_uint8(data) == 1 if data else False
    
    def get_negative_SW(self, axis: int, camera_id: int = 0):
        """
        axis: 1 = pan, 2 = tilt
        Sends one float32 (degrees) for the requested axis.
        """
        opcode, length = '010D', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        negative_sw = PacketConstructor.unpack_float32(data)
        return negative_sw

    def get_positive_SW(self, axis: int, camera_id: int = 0):
        """
        axis: 1 = pan, 2 = tilt
        Sends one float32 (degrees) for the requested axis.
        """
        opcode, length = '010D', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        positive_sw = PacketConstructor.unpack_float32(data)
        return positive_sw
    
    # --- Motor error register (uint16, by axis) ---
    def get_motor_error_reg(self, axis: int, camera_id: int = 0):
        """
        Read motor error register bits for a given axis.
        axis: 1 = pan, 2 = tilt
        Returns: int (0..65535) or None on failure.
        """
        opcode, length = '0E0B', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length=length, opcode=opcode,
            camera_id=camera_id, axis_group_id=axis
        )
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        if not data or len(data) < 4:
            return None
        return PacketConstructor.unpack_uint16(data[:4])

    @staticmethod
    def _bit(val: int, i: int) -> bool:
        return ((val >> i) & 1) == 1

    # ---- Convenience helpers for specific bits ----
    def err_fault(self, axis):              reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 0)
    def axis_is_on(self, axis):             reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 1)
    def motion_completed(self, axis):       reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 2)
    def enable_inactive(self, axis):        reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 3)
    def under_voltage(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 4)
    def over_voltage(self, axis):           reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 5)
    def i2t(self, axis):                    reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 6)
    def over_current(self, axis):           reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 7)
    def lsn_active(self, axis):             reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 8)   # HW LSN
    def lsp_active(self, axis):             reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 9)   # HW LSP
    def control_error(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 10)
    def short_circuit(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 11)
    def encoder_broken_wire(self, axis):    reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 12)
    def sw_lsn_active(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 13)  # SW LSN
    def sw_lsp_active(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 14)  # SW LSP
    def can_bus_error(self, axis):          reg = self.get_motor_error_reg(axis); return (reg is not None) and self._bit(reg, 15)

    # --- Aim degrees (pan/tilt in degrees, float32 each) -------------------
    def get_aim_degree(self, axis: int, camera_id: int = 0):
        """
        axis: 1 = pan, 2 = tilt
        Returns a float (degrees) or None on failure.
        """
        opcode, length = '0F30', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        print(opcode, pkt)
        print('data', data)
        if not data or len(data) < 4:
            return None
        data_recv =  PacketConstructor.unpack_float32(data)
        return data_recv

    def get_aim_degrees(self, camera_id: int = 0):
        """
        Backward compatible helper: does two calls:
        axis=1 -> pan, axis=2 -> tilt
        """
        pan  = self.get_aim_degree(axis=1, camera_id=camera_id)
        tilt = self.get_aim_degree(axis=2, camera_id=camera_id)
        return pan, tilt

    # --- Manual offset (degrees) -------------------------------------------
    def set_ballistic_manual_offset_degree(self, axis: int, value_deg: float, camera_id: int = 0):
        """
        axis: 1 = pan, 2 = tilt
        Sends one float32 (degrees) for the requested axis.
        """
        opcode, length = '0F3E', '08'
        data = PacketConstructor.pack_float32(value_deg)
        pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
        return self.send_response_wait_response(pkt)

    def get_ballistic_manual_offset_degree(self, axis: int, camera_id: int = 0):
        """
        axis: 1 = pan, 2 = tilt
        Returns float degrees (or None).
        """
        opcode, length = '0F3F', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        if not data or len(data) < 4:
            return None
        return PacketConstructor.unpack_float32(data)
    
    # --- Ballistic table inputs --------------------------------------------
    def set_wind(self, wind_ms: float, camera_id=0, axis=0):
        opcode, length = '0F80', '08'
        data = PacketConstructor.pack_float32(wind_ms)
        pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
        # return self.send_response_wait_response(pkt)
        ans =  self.send_response_wait_response(pkt)
        print('[set_wind]: ', opcode, data, wind_ms, ans)
        return ans

    def get_wind(self, camera_id=0, axis=0) -> float:
        opcode, length = '0F81', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_float32(data) if data else None

    def set_weapon_type(self, weapon_id: int, camera_id=0, axis=0):
        """weapon_id: 0=MAG, 1=M2, 2=M4A1"""
        opcode, length = '0F82', '05'
        data = PacketConstructor.pack_uint8(int(weapon_id) & 0xFF)
        pkt = PacketConstructor.construct_data_packet(self.prefix, length, opcode, data, camera_id, axis)
        return self.send_response_wait_response(pkt)

    def get_weapon_type(self, camera_id=0, axis=0) -> int:
        opcode, length = '0F83', '04'
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_uint8(data) if data else None
    
    # --- Multi calibration (read row by index, 0..4) -----------------------
    def get_multi_calib(self, camera_id=0, index=0):
        """Returns (distance_m, tilt_deg, pan_deg) for the given row index (0..4)."""
        # opcode, length = '0F84', '04' #prev 0F88
        opcode, length = '0F84', '04' 
        # axis argument conveys the index
        print('index', index)
        pkt = PacketConstructor.construct_no_data_packet(self.prefix, length=length, opcode=opcode,
                                                          camera_id=camera_id, axis_group_id=index)
        print(opcode, pkt)
        ans = self.send_response_wait_response(pkt)
        print('ans', ans)
        data = PacketConstructor.extract_data(ans)
        print('data: ', data, len(data))
        if not data or len(data) < 24:
            return None, None, None
        distance_m = PacketConstructor.unpack_float32(data[0:8])
        tilt_deg   = PacketConstructor.unpack_float32(data[8:16])
        pan_deg    = PacketConstructor.unpack_float32(data[16:24])
        return distance_m, tilt_deg, pan_deg
    
    def get_apos(self, camera_id=0, axis=0):
        opcode = '0109' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        # print(type(data_recievd), data_recievd)
        apos_degree = PacketConstructor.unpack_float32(data_recievd)
        # print('degree: ', apos_degree)
        return apos_degree
    
    def get_apos_pan(self, camera_id=0, axis=0):
        return self.get_apos(axis=0x01)
        
    def get_apos_tilt(self, camera_id=0, axis=0):
        return self.get_apos(axis=0x02)

    def get_aspd(self, camera_id=0, axis=0):
        opcode = '010A' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
        data_recievd = PacketConstructor.extract_data(answer)
        # print(type(data_recievd), data_recievd)
        motor_speed = PacketConstructor.unpack_float32(data_recievd)
        # print('motor_speed: ', motor_speed)
        return motor_speed
    
    def get_aspd_pan(self, camera_id=0, axis=0):
        return self.get_aspd(axis=0x01)
        
    def get_aspd_tilt(self, camera_id=0, axis=0):
        return self.get_aspd(axis=0x02)

    def reset_aim(self, camera_id=0, axis=0):
        opcode = '0F33' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
    
    def set_aim_by_px(self, x, y, camera_id=0, axis=0):
        opcode = '0F3B' 
        length = '08'
        data = PacketConstructor.pack_uint16(x)+PacketConstructor.pack_uint16(y)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
    
    def set_manual_aim_ofset_on(self, camera_id=0, axis=0):
        opcode = '0F3C' 
        length = '05'
        value = 1
        data = PacketConstructor.pack_uint8(value)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
    
    def set_manual_aim_ofset_off(self, camera_id=0, axis=0):
        opcode = '0F3C' 
        length = '05'
        value = 0
        data = PacketConstructor.pack_uint8(value)
        packet = PacketConstructor.construct_data_packet(self.prefix, length=length,
                                                      opcode=opcode, data_to_send=data,
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print('ans: ', answer, opcode)
    
    def get_set_point_px(self, camera_id=0, axis=0):
        opcode = '0A28' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                        opcode=opcode, 
                                                        camera_id=camera_id,
                                                        axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, 'sent:', packet, opcode)
        data_recievd = PacketConstructor.extract_data(answer)

        if data_recievd is None:
            print("[get_camera_aim_pixels] Warning: No data received.")
            return None, None

        if len(data_recievd) < 4:
            print("[get_camera_aim_pixels] Error: Data too short.")
            return None, None

        x = PacketConstructor.unpack_uint16(data_recievd[:4])
        y = PacketConstructor.unpack_uint16(data_recievd[4:8])

        return x, y
    
    # ============================================================
    #   PID CONTROL: Kp, Ki, Kd, Acceleration
    # ============================================================

    # ---------- Kp ----------
    def set_camera_kp(self, kp: float, camera_id=0, axis=1):
        opcode, length = '0A07', '08'
        data = PacketConstructor.pack_float32(kp)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        print(f"[set_camera_kp] kp={kp} → {ans}")
        return ans

    def get_camera_kp(self, camera_id=0, axis=1):
        opcode, length = '0A08', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        print('pkt: ', pkt)
        print('ans: ', ans)
        print('data: ', data)
        return PacketConstructor.unpack_float32(data) if data else None

    # ---------- Ki ----------
    def set_camera_ki(self, ki: float, camera_id=0, axis=1):
        opcode, length = '0A09', '08'
        data = PacketConstructor.pack_float32(ki)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        print(f"[set_camera_ki] ki={ki} → {ans}")
        return ans

    def get_camera_ki(self, camera_id=0, axis=1):
        opcode, length = '0A0A', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_float32(data) if data else None

    # ---------- Kd ----------
    def set_camera_kd(self, kd: float, camera_id=0, axis=1):
        opcode, length = '0A0B', '08'
        data = PacketConstructor.pack_float32(kd)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        print(f"[set_camera_kd] kd={kd} → {ans}")
        return ans

    def get_camera_kd(self, camera_id=0, axis=1):
        opcode, length = '0A0C', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_float32(data) if data else None

    # ---------- PID Acceleration ----------
    def set_camera_acc(self, acc: float, camera_id=0, axis=1):
        opcode, length = '0A0D', '08'
        data = PacketConstructor.pack_float32(acc)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        print(f"[set_camera_acc] acc={acc} → {ans}")
        return ans

    def get_camera_acc(self, camera_id=0, axis=1):
        opcode, length = '0A0E', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_float32(data) if data else None
    
    # ---------- FEEDFOWARD ----------
    def set_camera_feedfoward(self, feedfoward: float, camera_id=0, axis=1):
        opcode, length = '0A0F', '08'
        data = PacketConstructor.pack_float32(feedfoward)
        pkt = PacketConstructor.construct_data_packet(
            self.prefix, length, opcode, data, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        print(f"[set_camera_feedfoward] feedfoward={feedfoward} → {ans}")
        return ans

    def get_camera_feedfoward(self, camera_id=0, axis=1):
        opcode, length = '0A10', '04'
        pkt = PacketConstructor.construct_no_data_packet(
            self.prefix, length, opcode, camera_id, axis)
        ans = self.send_response_wait_response(pkt)
        data = PacketConstructor.extract_data(ans)
        return PacketConstructor.unpack_float32(data) if data else None


    def axis_off(self, camera_id=0, axis=0):
        opcode = '013D' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        # print('ans: ', answer, opcode)

    def axis_on(self, camera_id=0, axis=0):
        opcode = '013C' 
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        print('packet- axis on', packet)
        answer = self.send_response_wait_response(packet)
    
    def reset_axis(self, camera_id=0, axis=0):
        opcode = '013E'
        length = '04'
        packet = PacketConstructor.construct_no_data_packet(self.prefix, length=length,
                                                      opcode=opcode, 
                                                      camera_id=camera_id,
                                                      axis_group_id=axis)
        answer = self.send_response_wait_response(packet)
        print(f"Reset axis {axis}, answer: {answer}")
    
    def axes_on(self, camera_id=0, axis=0):
        self.axis_on(axis=0x01)
        self.axis_on(axis=0x02)

    def axes_off(self, camera_id=0, axis=0):
        self.axis_off(axis=0x01)
        self.axis_off(axis=0x02)

    def reset_axes(self, camera_id=0, axis=0):
        self.reset_axis(axis=0x01)
        self.reset_axis(axis=0x02)

    # #----------------differnt format of sending------------
    # def send_axis_command(self, axis, opcode, expect_response=True):
    #     """Send a command to the specified axis with checksum handling."""
    #     opcode_high = (opcode >> 8) & 0xFF  
    #     opcode_low = opcode & 0xFF   
    #     checksum = 0x00
    #     packet = [0x50, 0x54, 0x04, 0x00, axis, opcode_high, opcode_low, 0x00]
    #     try:
    #         checksum = PacketConstructor.calculate_checksum(packet)
    #     except Exception as e:
    #         print('e: ', e)
    #     packet[-1] = int(checksum, 16)
    #     answer = self.send_response_wait_response(packet)

    # def axis_control(self, opcode, message):
    #     """Generic function to send commands to both axis 1 and 2."""
    #     for axis in (0x01, 0x02):
    #         self.send_axis_command(axis, opcode)
    #     print(message)

    # def axis_off(self, axis):
    #     opcode = 0x301D  
    #     self.send_axis_command(axis, opcode)

    # def axis_on(self, axis):
    #     opcode = 0x013C 
    #     self.send_axis_command(axis, opcode)
    
    # def reset_axis(self, axis):
    #     opcode = 0x013E 
    #     print(f"Reset axis {axis}, answer: {self.send_axis_command(axis, opcode)}")

    # def axes_off(self):
    #     opcode = 0x013D
    #     self.axis_control(opcode, "Axis off")

    # def axes_on(self):
    #     opcode = 0x013C
    #     self.axis_control(opcode, "Axis on")
    
    # def reset_axes(self):
    #     opcode = 0x013E
    #     self.axis_control(opcode, "reset off")
    
    
    def close_connection(self):
        if hasattr(self, 'client_socket') and self.client_socket:
            try:
                self.client_socket.close()
                print("Connection closed.")
            except Exception as e:
                print("Error closing the connection:", e)
            finally:
                self.client_socket = None

# # # x = '4316'
# # # print(PacketConstructor.unpack_float32(x))
# captrack = CaptrackConnection("192.168.10.192", 4949)
# ans = captrack.speed_movement_both_axis(pan_speed=-2, acc_pan=120)
# time.sleep(5)
# ans = captrack.speed_movement_both_axis(pan_speed=0, acc_pan=120)
# captrack = CaptrackConnection("192.168.10.170", 4949)
# ans = captrack.set_lrf_manual_distance(200)
# # ans = captrack.is_active_swls(axis=1)
# # print('ans', ans)
# # ans = captrack.get_positive_SW(axis=1)
# # # ans = captrack.get_aim_degree(axis=1, camera_id=0)
# # print('ans', ans)
# # ans = captrack.get_negative_SW(axis=1)
# # # ans = captrack.get_aim_degree(axis=1, camera_id=0)
# # print('ans', ans)
# ans = captrack.sw_lsn_active(axis=1)
# print(ans)

# # # captrack.set_camera_center(1,0,0)
# # # print(captrack.get_camera_center_enabled(0))
# # # print(captrack.get_camera_center_enabled(1))
# # # captrack.close_connection()