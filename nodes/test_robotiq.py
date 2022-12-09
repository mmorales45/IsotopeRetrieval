#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Script used to initially control the Robotiq grippers using low level control.
"""

import time
import signal
from argonne_final_project.RobotiqHand import RobotiqHand


HOST = "192.168.12.248"
PORT = 54321

cont = True


def test_robotiq():
    """
        Function:
            Make the gripper open and close to test its operation
    """
    print 'test_robotiq start'
    hand = RobotiqHand()
    hand.connect(HOST, PORT)

    try:
        print 'activate: start'
        # hand.reset()
        hand.activate()
        result = hand.wait_activate_complete()
        print 'activate: result = 0x{:02x}'.format(result)
        if result != 0x31:
            hand.disconnect()
            return
        print 'close slow'
        hand.move(255, 100, 10)
        (status, position, force) = hand.wait_move_complete()
        position_mm = hand.get_position_mm(position)
        force_mA = hand.get_force_mA(force)

        if status == 0:
            print 'no object detected: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA)
        elif status == 1:
            print 'object detected closing: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA)
            print 'keeping'
            time.sleep(5)
        elif status == 2:
            print 'object detected opening: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA)
        else:
            print 'failed'

        hand.move(0, 0, 0)
        (status, position, force) = hand.wait_move_complete()
        position_mm = hand.get_position_mm(position)
        force_mA = hand.get_force_mA(force)
        print 'position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA)
    except:
        print 'Ctrl-c pressed'

    hand.disconnect()

if __name__ == '__main__':
    test_robotiq()
