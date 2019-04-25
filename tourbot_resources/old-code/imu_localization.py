import serial
import struct
import sys
import time
import os


def prog_shutdown_callback(ex=None):
    global running, serial_device, output_csv_file
    running = False

    print(ex)

    if serial_device is not None:
        serial_device.close()

    if output_csv_file is not None:
        output_csv_file.close()

    sys.exit(-1)


def bit_is_set(num, nth_bit):
    return num & (1 << nth_bit)


def get_min_max_gyro():
    max_v = 0
    min_v = 0

    gyro_list = []

    with open("/home/caleb/ros_ws/output.csv", "r") as csv_data:

        for line in csv_data:
            line_split = line.split(',')
            # print(line_split)

            gyro_list.append(float(line_split[-1]))

    for val in gyro_list:
        if val > max_v:
            max_v = val

        if val < min_v:
            min_v = val

    print("Max: {}\nMin: {}".format(max_v, min_v))


def count_acc_gyro():
    acc = 0
    gyro = 0

    with open("/home/caleb/ros_ws/output.csv", "r") as csv_data:

        for line in csv_data:

            if line[0] == "a":
                acc += 1

            if line[0] == "g":
                gyro += 1

    if acc > gyro:
        print("Acc diff: {}".format(acc - gyro))

    else:
        print("Gyro diff: {}".format(gyro - acc))

    print("Acc and Gyro: {}".format((acc, gyro)))


def main():
    count_acc_gyro()


if __name__ == '__main__':
    main()
