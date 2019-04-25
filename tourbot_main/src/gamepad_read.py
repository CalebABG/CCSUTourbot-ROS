#!/usr/bin/python

import os.path
import re
import shlex
import signal
import struct
import sys
from time import sleep
import easyprocess
import serial

"""
Functions
"""


def try_connect_controller():
    global ser, in_file, infile_path, disable_xinput

    sleep_time = 2
    should_sleep = True

    print("Trying to Connect to Controller, Re-trying Every {} Seconds...".format(sleep_time))

    infile_path = "/dev/input/event" + str(get_gamepad_event_id())

    try:
        in_file = open(infile_path, "rb")

        if in_file is not None:
            should_sleep = False
            print("Controller Connected: Waiting to Write Serial Data")

            if disable_xinput:
                print("Disabling Controller Mouse Control")
                run_command('xinput disable ' + str(get_gamepad_xinput_id()), 1)

    except IOError:
        pass

    if should_sleep:
        sleep(sleep_time)


# stop ctrl+z from closing program
# if not shut down properly motors will keep sending
# the last command from the controller
def ctrl_z_handler(signum, frame):
    print("ctrl+z ignored")


# Handle ctrl+z
signal.signal(signal.SIGTSTP, ctrl_z_handler)


def run_command(cmd, cmd_timeout=5.0):
    cmd_output = easyprocess.Proc(shlex.split(cmd)).call(timeout=cmd_timeout)  # type: easyprocess.EasyProcess
    return cmd_output.stdout, cmd_output.stderr


def get_xinput_devices():
    regex = re.compile(r"(\s*(Gamepad|Wireless Controller\d*)\s*.*(id=(\d*)))")
    tuple_list = []

    cmd_output = run_command('xinput', cmd_timeout=.2)  # returns a tuple (stdout, stderr)

    # print(cmd_output)

    matches = re.finditer(regex, cmd_output[0])

    for match in matches:
        match_groups = match.groups()
        # print(match_groups)

        if not len(match_groups) < 4:
            tuple_list.append((str(match_groups[-3]), str(match_groups[-1])))

    return tuple_list


def get_controllers():
    regex = re.compile(r"(/event(\d+))\s*:\s*(.*)")
    tuple_list = []

    cmd_output = run_command('evtest', cmd_timeout=.2)  # returns a tuple (stdout, stderr)

    matches = re.finditer(regex, cmd_output[-1])

    for match in matches:
        match_groups = match.groups()

        if not len(match_groups) < 3:
            tuple_list.append((str(match_groups[-1]), str(match_groups[-2])))

    return tuple_list


def get_gamepad_event_id():
    # change both indexes if there are more than 1 controller connected
    controller_tuple_index = 0
    controller_id_index = -1
    controller_list = get_controllers()

    if controller_list:
        return controller_list[controller_tuple_index][controller_id_index]
    else:
        return None


def get_gamepad_xinput_id():
    # change both indexes if there are more than 1 controller connected
    device_tuple_index = 0
    device_id_index = -1
    xinput_list = get_xinput_devices()

    if xinput_list:
        return xinput_list[device_tuple_index][device_id_index]
    else:
        return None


def map_val(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def clamp_val(n, minn, maxn):
    return min(max(n, minn), maxn)


def m_drive(v):
    return (-.376 * v) + 48
    # return 1 * ((-.25 * v) + 32)


def m_angular_offset(v):
    return (v * .117) - 15
    # return 0


def controller_is_connected():
    return os.path.exists(infile_path)


def prog_shutdown_func(ex=None):
    global script_running, ser

    script_running = False
    print(ex)

    s_msg = "Sent Stop Command - Fully Stopped Motors "
    print("Stopping Motors...")

    if ser is not None:
        s_msg += u'\u2611'
        ser.write(struct.pack('B', 0x00))
        ser.close()
    else:
        s_msg += u'\u24CD'

    print(s_msg)

    sys.exit(-1)


def write_serial_data():
    (m1_command, m2_command) = read_data()

    if debugFlag:
        print("Debug: Serial Data(M1, M2): ({}, {})".format(m1_command, m2_command))
    else:
        print("Writing Serial Data(M1, M2): ({}, {})".format(m1_command, m2_command))
        ser.write(struct.pack('BB', m1_command, m2_command))
        print("Wrote Data Successfully \n")


def read_data():
    global last_ud, last_lr

    event = in_file.read(EVENT_SIZE)

    (tv_sec, tv_usec, event_type, event_code, event_value) = struct.unpack(FORMAT, event)

    # handle 'select' button press to stop motors and shutdown program
    if event_type == 1 and event_code == 314:
        if event_value == 1:
            prog_shutdown_func("\nSelect Pressed: Sending Shutdown Command")

    # left stick up and down
    if event_type == 3 and event_code == 1:
        last_ud = event_value

    # right stick left and right
    # if event_type == 3 and (event_code == 2 or event_code == 3):
    if event_type == 3 and event_code == 2:
        last_lr = event_value

    print("ABS_X: {} - ABS_A: {}".format(last_ud, last_lr))

    # m_ud1 = map_val(last_ud, 0, 255, 1, 128)
    # m_ud2 = map_val(last_ud, 0, 255, 128, 255)
    m_ud1 = round(m_drive(last_ud)) + m1_stop
    m_ud2 = round(m_drive(last_ud)) + m2_stop

    offset = round(m_angular_offset(last_lr))

    # print("OFFSET: {}".format(offset))

    # m_vel1 = int(clamp_val(m_ud1 - offset, 0, 255))
    # m_vel2 = int(clamp_val(m_ud2 + offset, 0, 255))

    m_vel1 = int(clamp_val(m_ud1 - offset, 0, 255))
    m_vel2 = int(clamp_val(m_ud2 + offset, 0, 255))

    # m_vel1 = m_ud1 - offset
    # m_vel2 = m_ud2 + offset

    return m_vel1, m_vel2


"""
Variables
"""

debugFlag = False
device = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"

ser = None if debugFlag else serial.Serial(port=device)

# struct destructuring format
FORMAT = "llHHI"

# constant event size to ensure restricted domain for event values: [0, 255]
EVENT_SIZE = 24

# File path to controller serial port
infile_path = "/dev/input/event" + str(get_gamepad_event_id())

print(infile_path)

disable_xinput = True

# open file in read + binary mode
if not os.path.exists(infile_path):
    in_file = None
    print("Controller not Connected")
else:
    print("Controller Connected")
    in_file = open(infile_path, "rb")

    if disable_xinput:
        print("Disabling Controller Mouse Control")
        run_command('xinput disable ' + str(get_gamepad_xinput_id()), cmd_timeout=1)

    print("Waiting to Write Serial Data...\n")

# up and down - left stick
last_ud = 128

# left and right - right stick
last_lr = 128

m1_stop = 64
m2_stop = 192

# boolean to keep program alive
script_running = True

"""
Main function
"""

if __name__ == '__main__':
    while script_running:
        try:
            # If the controller is connected when the program starts
            # then try to write serial data
            # if the controller dies or loses connection then go straight to
            # trying to connect to it
            if controller_is_connected():
                try:
                    write_serial_data()
                except IOError:
                    try_connect_controller()
            else:
                try_connect_controller()

        except KeyboardInterrupt as e:
            prog_shutdown_func(e)
