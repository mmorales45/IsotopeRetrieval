# -*- coding: utf-8 -*-
"""
Script to control the Robotiq gripper using low level control.
"""
import socket
import time
import sys
import struct
import threading

#------------------------------------------------------------------------------
# RobotiqHand class for python 2.7
#------------------------------------------------------------------------------
class RobotiqHand():
    """
    Class to control robotiq gripper.
    """
    def __init__(self):
        self.so = None
        self._cont = False
        self._sem = None
        self._heartbeat_th = None
        self._max_position = 255
        self._min_position = 0

    def _heartbeat_worker(self):
        """
            Funtion:
                Loop and act as a heartbeat for the script.
        """
        while self._cont:
            self.status()
            time.sleep(0.5)

    def connect(self, ip, port):
        """
            Funtion:
                Connect to the robotiq gripper using socket and the IP address.
        """
        self.so = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.so.connect((ip, port))
        self._cont = True
        self._sem = threading.Semaphore(1)
        self._heartbeat_th = threading.Thread(target = self._heartbeat_worker)
        self._heartbeat_th.start()

    def disconnect(self):
        """
            Funtion:
                Disconnect from the robotiq gripper.
        """
        self._cont = False
        self._heartbeat_th.join()
        self._heartbeat_th = None
        self.so.close()
        self.so = None
        self._sem = None

    def _calc_crc(self, command):
        """
            Funtion:
                Do some bitwise operations to translate from bytes to a command value.
        """
        crc_registor = 0xFFFF
        for data_byte in command:
            tmp = crc_registor ^ data_byte
            for _ in range(8):
                if(tmp & 1 == 1):
                    tmp = tmp >> 1
                    tmp = 0xA001 ^ tmp
                else:
                    tmp = tmp >> 1
            crc_registor = tmp
        crc = bytearray(struct.pack('<H', crc_registor))
        return crc

    def send_command(self, command):
        """
            Funtion:
                Send the command after converting it to bytes.
        """
        with self._sem:
            crc = self._calc_crc(command)
            data = command + crc
            self.so.sendall(data)
            time.sleep(0.001)
            data = self.so.recv(1024)
        return bytearray(data)

    def status(self):
        """
            Funtion:
                Get the status of the gripper.
        """
        command = bytearray(b'\x09\x03\x07\xD0\x00\x03')
        return self.send_command(command)

    def reset(self):
        """
            Funtion:
                Reset the parameters of the command value.
        """
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def activate(self):
        """
            Funtion:
                Turn on the gripper.
        """
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def wait_activate_complete(self):
        """
            Funtion:
                Wait until the status is green and the gripper operational.
        """
        while True:
            data = self.status()
            if data[5] != 0x00:
                return data[3]
            if data[3] == 0x31 and data[7] < 4:
                return data[3]

    def adjust(self):
        """
            Funtion:
                Move the gripper to a closed then open state.
        """
        self.move(255, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._max_position = position
        self.move(0, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._min_position = position

    def get_position_mm(self, position):
        """
            Funtion:
                Get the current position of the gripper in mm.
        """
        if position > self._max_position:
            position = self._max_position
        elif position < self._min_position:
            position = self._min_position
        position_mm = 85.0 * (self._max_position - position) / (self._max_position - self._min_position)
        #print 'max=%d, min=%d, pos=%d pos_mm=%.1f' % (self._max_position, self._min_position, position, position_mm)
        return position_mm

    def get_force_mA(self, force):
        """
            Funtion:
                Convert the force to mA given the force value in bytes.
        """
        return 10.0 * force

    # position: 0x00...open, 0xff...close
    # speed: 0x00...minimum, 0xff...maximum
    # force: 0x00...minimum, 0xff...maximum
    def move(self, position, speed, force):
        """
            Funtion:
                Move the gripper to a goal position.
        """
        #print('move hand')
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\x00\x00')
        command[10] = position
        command[11] = speed
        command[12] = force
        return self.send_command(command)

    # result: (status, position, force)
    def wait_move_complete(self):
        """
            Funtion:
                Wait in a loop until the gripper has completed the motion.
        """
        while True:
            data = self.status()
            if data[5] != 0x00:
                return (-1, data[7], data[8])
            if data[3] == 0x79:
                return (2, data[7], data[8])
            if data[3] == 0xb9:
                return (1, data[7], data[8])
            if data[3] == 0xf9:
                return (0, data[7], data[8])
