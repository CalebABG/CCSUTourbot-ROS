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


def set_acc_and_gyro(num  # type: int
                     ):
    global serial_device

    if serial_device is not None:
        # turn on acceleration if bit is 0
        if bit_is_set(num, 0) == 0:
            print "Turning on Acc"
            serial_device.write("vara")
        else:
            print "Acc is on"

        # turn on gyro if bit is 0
        if bit_is_set(num, 1) == 0:
            print "Turning on Gyro"
            serial_device.write("varg")
        else:
            print "Gyro is on"

        # toggle bits we're not interested in collecting
        # turn off quat if bit is 1
        if bit_is_set(num, 2) == 0:
            print "Turning on Quats"
            serial_device.write("varq")
        else:
            print "Quats are on"

        # turn off mag if bit is 1
        if bit_is_set(num, 3) == 1:
            serial_device.write("varc")
        else:
            print "Turning off Mag"

        # turn off euler angles if bit is 1
        if bit_is_set(num, 4) == 0:
            serial_device.write("vare")
        else:
            print "Turning on Euler"


serial_device = None

try:
    # ser = serial.Serial('COM10')
    serial_device = serial.Serial(
        port='/dev/serial/by-id/usb-Variense_VMU931_9312095E77-if00',
        # baudrate=115200,
        timeout=0
        # writeTimeout=2
    )

except IOError:
    print("device not found")
    sys.exit(-1)

commandPrefix = "var"
commandTypes = {
    "Acc": commandPrefix + "a",
    "Gyro": commandPrefix + "g",
    "Mag": commandPrefix + "c",
    "Quat": commandPrefix + "q",
    "Euler": commandPrefix + "e",
    "Heading": commandPrefix + "h",
    "Self-Test": commandPrefix + "t",
    "Calibration": commandPrefix + "l",
    "Status": commandPrefix + "s"
}
msgTypes = {
    "Message-Header": 1,
    "Message-Calibration": 2,
    "Acc": 97,
    "Gyro": 103,
    "Mag": 99,
    "Quat": 113,
    "Euler": 101,
    "Heading": 104,
    "Sensor-Status": 115
}

try:
    os.remove("output.csv")
except OSError:
    print("Either no file or could not delete, check dir")
    pass


def setup_axes():
    global setup_count

    try:
        while setup_count == 0:

            # print("Sending Sensor Status Command")

            serial_device.write("vars")
            # time.sleep(0.01)

            message_ = serial_device.read(1)
            message_ = struct.unpack("B", message_)

            # print("Reading Bytes: {}".format(message_))

            if message_[0] == msgTypes["Message-Header"]:

                message_ = serial_device.read(2)
                message_ = struct.unpack("BB", message_)

                if message_[1] == msgTypes["Sensor-Status"]:
                    msg_format = '>BBBI'

                    msg_size = struct.calcsize(msg_format)
                    message_ = serial_device.read(msg_size)

                    (ss, sr, los, cds) = struct.unpack(msg_format, message_)

                    # message_ = struct.unpack(msg_format, message_)
                    # (ss, sr, los, cds) = message_

                    print("Data Streaming - Last Byte: " + format(cds, "08b") + "\n")
                    # print(format(message_[-1], "08b"))

                    print "Setting Axes, Toggling Bits\n"
                    set_acc_and_gyro(cds)
                    # set_acc_and_gyro(message_[-1])
                    print "\nSet Axes\n"

                    print "Setting Sensor Gyro Res: 250 dps"
                    serial_device.write("var0")
                    print "Set Sensor Gyro Resolution\n"

                    setup_count = 1

                    print "Pausing for 2 seconds...\n"
                    time.sleep(2)

                    return True

                    # prog_shutdown_callback("Test Setup")

    except (KeyboardInterrupt, serial.SerialException, Exception) as x:
        prog_shutdown_callback(x)


def calibrate_imu():
    done = False

    with serial.Serial('/dev/serial/by-id/usb-Variense_VMU931_9312095E77-if00', timeout=0, rtscts=1) as ser_dev:
        print "Initializing IMU Dev, Sleeping for 2 Secs...\n"
        time.sleep(2)

        if ser_dev.is_open:

            try:
                print "Flushing Port...\n"
                ser_dev.reset_input_buffer()
                ser_dev.reset_output_buffer()

                print "Preparing to write Calibration Command\n"

                # write data
                serial_device.write(b'vari')

                print("Writing Command: \'vari\' for Calibration\n")
                time.sleep(0.5)  # give the serial port sometime to receive the data

                print "Reading 1 Byte from Port\n"
                port_response = ser_dev.read(1)
                port_response = struct.unpack('B', port_response)

                print "Read 1 Byte - Response: {}\n".format(port_response)

                if port_response[0] == msgTypes["Message-Calibration"]:
                    print "Well"

            except (KeyboardInterrupt, serial.SerialException, Exception) as imu_except:
                prog_shutdown_callback(imu_except)

            finally:
                prog_shutdown_callback("Shutdown Issued at Term End")

        else:
            print("port is not open, please open imu port")

    # if serial_device.is_open:
    #
    #     try:
    #         serial_device.flushInput()  # flush input buffer, discarding all its contents
    #         serial_device.flushOutput()  # flush output buffer, aborting current output
    #
    #         print "Preparing to write Calibration Command"
    #
    #         # write data
    #         serial_device.write(b'vari')
    #
    #         print("Writing Command: vari For Calibration")
    #         time.sleep(0.5)  # give the serial port sometime to receive the data
    #
    #
    #
    #         # numOfLines = 0
    #         #
    #         # while True:
    #         #     response = serial_device.readline()
    #         #
    #         #     res = serial_device.read
    #         #     print("read data: " + response)
    #         #
    #         #     numOfLines += 1
    #         #
    #         #     if numOfLines >= 5:
    #         #         break
    #
    #     except (KeyboardInterrupt, serial.SerialException, Exception) as excep:
    #         prog_shutdown_callback(excep)
    #
    # else:
    #     print("IMU port is not open, please re-open port")


# def calibrate_imu():
#     done = False
#
#     try:
#         while not done:
#             # print("Sending Sensor Status Command")
#
#             serial_device.write("vari")
#
#             message_ = serial_device.read(1)
#             message_ = struct.unpack("B", message_)
#
#             # print("Calibrating: Reading Bytes: {}".format(message_,))
#
#             if message_[0] == msgTypes["Message-Calibration"]:
#
#                 message_ = serial_device.read(2)
#
#                 (message_data) = struct.unpack("BB", message_)
#
#                 print message_data
#
#                 msg_str = ""
#                 # msg_int = -1
#                 msg_int = ''
#
#                 while msg_int != 3:
#                     message_ = serial_device.read(1)
#
#                     (msg_int,) = struct.unpack("B", message_)
#
#                     # print msg_int
#                     msg_char = chr(msg_int)
#                     # msg_char = chr(msg_int).decode("utf-8")
#
#                     msg_str += msg_char
#
#                     # print(msg_char)
#
#                 print(msg_str)
#                 done = True
#
#     except (KeyboardInterrupt, serial.SerialException, Exception) as ex:
#         prog_shutdown_callback(ex)
#
#     finally:
#         prog_shutdown_callback("Stopped By Execution")

setup_count = 0

running = False

aGain = -9.81
gGain = (250.0 / 2000)

output_csv_file = None

try:
    output_csv_file = open("output.csv", "a")
except IOError:
    print("File either not found or not created, check dir")
    pass


def main(debug=False):
    global running

    acc_timestamp = None
    gyro_timestamp = None
    csv_acc_str = ""
    csv_gyro_str = ""

    while running:
        try:
            b = serial_device.read(1)
            b = struct.unpack("B", b)

            (message_start_type,) = b

            if message_start_type == msgTypes["Message-Header"]:

                b = serial_device.read(2)
                b = struct.unpack("BB", b)

                if b[1] == msgTypes["Acc"]:
                    b = serial_device.read(16)
                    b = struct.unpack(">Ifff", b)

                    (timestamp1, x1, y1, z1) = b

                    acc_timestamp = timestamp1

                    Ax = x1 * aGain
                    Ay = y1 * aGain
                    Az = z1 * aGain

                    if debug:
                        # pass
                        # csv_acc_str = "%c,%d,%f,%f,%f\n" % ('a', timestamp1, Ax, Ay, Az)
                        print "A({} - {:.3f}, {:.3f}, {:.3f})".format(timestamp1, Ax, Ay, Az)
                    else:
                        # pass
                        output_csv_file.write("%c,%d,%f,%f,%f\n" % ('a', timestamp1, Ax, Ay, Az))

                if b[1] == msgTypes["Gyro"]:
                    b = serial_device.read(16)
                    b = struct.unpack(">Ifff", b)

                    (timestamp2, x2, y2, z2) = b

                    gyro_timestamp = timestamp2

                    Gx = x2 * gGain
                    Gy = y2 * gGain
                    Gz = z2 * gGain

                    if debug:
                        # csv_gyro_str = "%c,%d,%f,%f,%f\n" % ('g', timestamp2, Gx, Gy, Gz)

                        # if Gz > max_Gyro_z:
                        #     max_Gyro_z = Gz
                        #
                        # if Gz < min_Gyro_z:
                        #     min_Gyro_z = Gz

                        print "G({} - {:.3f}, {:.3f}, {:.3f})".format(timestamp2, Gx, Gy, Gz)

                    else:
                        output_csv_file.write("%c,%d,%f,%f,%f\n" % ('g', timestamp2, Gx, Gy, Gz))

                # if acc_timestamp == gyro_timestamp:
                #     print(csv_acc_str, csv_gyro_str)
                #     # print "match"
                # else:
                #     print "skip"

                # Un-comment for Euler Angles
                # if b[1] == msgTypes["Euler"]:
                #     b = serial_device.read(16)
                #     b = struct.unpack(">Ifff", b)
                #
                #     (timestamp3, x3, y3, z3) = b
                #
                #     Ex = x3
                #     Ey = y3
                #     Ez = z3
                #
                #     if debug:
                #         pass
                #         print "E({} - {:.3f}, {:.3f}, {:.3f})\n".format(timestamp3, Ex, Ey, Ez)
                #     else:
                #         pass
                #         # output_csv_file.write("%c,%d,%f,%f,%f\n" % ('e', timestamp3, Ex, Ey, Ez))

            # print("\tMax: {} - Min: {}".format(max_Gyro_z, min_Gyro_z))

        except (KeyboardInterrupt, serial.SerialException, Exception) as e:
            prog_shutdown_callback(e)


if __name__ == '__main__':
    calibrate_imu()

    # running = setup_axes()
    # main(debug=False)
    # main(debug=True)
