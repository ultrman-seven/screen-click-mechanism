import math
import serial
import struct
import time

# COM_NAME = 'COM3'


class FoobarTest():
    def __init__(self, COM_NAME='COM3'):
        try:
            self.com = serial.Serial(COM_NAME, 115200, 5)

            # com.write('rcs\x01\x03\r\n'.encode())
        except Exception as e:
            print('error: ', e)
            exit(1)

    def send_serial_CMD(cmd_fun):
        def sendCMD(self,*args, **kwargs):
            self.com.write('rcs'.encode())
            cmd_fun(self,*args, **kwargs)
            self.com.write('\r\n'.encode())
            # time.sleep(0.1)
        return sendCMD

    @send_serial_CMD
    def click(self):
        self.com.write('\x01\x03'.encode())

    @send_serial_CMD
    def move(self, x, y):
        bytes_data_x = struct.pack('<f', x)
        bytes_data_y = struct.pack('<f', y)
        self.com.write('\x09\x02'.encode())
        self.com.write(bytes_data_x)
        self.com.write(bytes_data_y)

    @send_serial_CMD
    def setServo_X_Y_Position(self, id, x, y):
        bytes_data_x = struct.pack('<f', x)
        bytes_data_y = struct.pack('<f', y)
        if (id == 0):
            self.com.write('\x09\x05'.encode())
        else:
            self.com.write('\x09\x06'.encode())
        self.com.write(bytes_data_x)
        self.com.write(bytes_data_y)

    @send_serial_CMD
    def setLinkLength(self, id, l1, l2):
        bytes_data_x = struct.pack('<f', l1)
        bytes_data_y = struct.pack('<f', l2)
        if (id == 0):
            self.com.write('\x09\x07'.encode())
        else:
            self.com.write('\x09\x08'.encode())
        self.com.write(bytes_data_x)
        self.com.write(bytes_data_y)

    @send_serial_CMD
    def setClickPosition(self, off, on):
        bytes_data_x = struct.pack('<f', off)
        bytes_data_y = struct.pack('<f', on)
        self.com.write('\x09\x0a'.encode())
        self.com.write(bytes_data_x)
        self.com.write(bytes_data_y)

    @send_serial_CMD
    def setClickTime(self, t):
        bytes_data = struct.pack("<H", t)
        self.com.write('\x03\x09'.encode())
        self.com.write(bytes_data)

    @send_serial_CMD
    def setServoIncTime(self, t):
        bytes_data = struct.pack("<H", t)
        self.com.write('\x03\x0f'.encode())
        self.com.write(bytes_data)

    @send_serial_CMD
    def saveCfg(self):
        self.com.write('\x01\x0c'.encode())

    @send_serial_CMD
    def set_servo_position(self, id, p):
        self.com.write('\x03\x0d'.encode())
        bytes_data_1 = struct.pack("<B", id)
        bytes_data_2 = struct.pack("<B", p)
        self.com.write(bytes_data_1)
        self.com.write(bytes_data_2)

    @send_serial_CMD
    def move_slow(self, x,y):
        self.com.write('\x09\x10'.encode())
        bytes_data_1 = struct.pack("<f", x)
        bytes_data_2 = struct.pack("<f", y)
        self.com.write(bytes_data_1)
        self.com.write(bytes_data_2)

if __name__ == '__main__':

    nmmbd = FoobarTest('COM3')

    def main1():
        nmmbd.setLinkLength(0, 130.0, 120.0)
        nmmbd.setLinkLength(1, 130.0, 120.0)
        nmmbd.setServo_X_Y_Position(0, -140.0, 0.0)
        nmmbd.setServo_X_Y_Position(1, 140.0, 0.0)
        nmmbd.setClickTime(1000)
        nmmbd.setClickPosition(20, 43)
        nmmbd.saveCfg()
        # set_servo_position(0, 50)
        # set_servo_position(1, 50)
        # click()


    def main2(id=1):
        while True:
            p = input('posi: ')
            if not p.isdigit():
                return
            p = int(p)
            nmmbd.set_servo_position(id, p)
            print(f'set servo{id} to {p}')


    def main3():
        while True:
            x = input('x= ')
            y = input('y= ')
            x = float(x)
            y = float(y)
            nmmbd.move(x, y)
            # move(y, x)
            time.sleep(1)
            d = nmmbd.com.read_all()
            print(d.decode())
            # d = com.readline().decode()
            # d = com.readline().decode()
            # print(d)


    L1 = 130
    L2 = 120

    SL_X = -140
    SR_X = 140

    SL_Y = 0
    SR_Y = 0


    def __nm(x, y, sx, sy, l1, l2):
        x -= sx
        y -= sy

        d2 = x*x+y*y
        d = d2**0.5

        cos_theta1 = (d2 + l1*l1 - l2*l2) / (2*d*l1)
        theta1 = math.acos(cos_theta1)
        theta_aux = math.atan2(y, x)
        print('theta aux = ', theta_aux)
        if abs(theta_aux) < (math.pi/2):
            return theta1 + math.pi/2 + theta_aux
        while theta_aux < 0:
            theta_aux += (math.pi * 2)
        return theta_aux - (theta1 + math.pi/2)


    def move_py_fuckArm(x, y):
        left_angle = __nm(x, y, SL_X, SL_Y, L1, L2)
        right_angle = __nm(x, y, SR_X, SR_Y, L1, L2)

        left_angle *= (100/math.pi)
        right_angle *= (100/math.pi)

        print(f'left :  {left_angle*1.8}, ({int(left_angle)})')
        print(f'right :  {right_angle*1.8}, ({int(right_angle)})')

        nmmbd.set_servo_position(0, int(left_angle))
        nmmbd.set_servo_position(1, int(right_angle))


    def main4():
        while True:
            x = input('x= ')
            y = input('y= ')
            x = float(x)
            y = float(y)
            move_py_fuckArm(x, y)


    # X_MID = 15
    X_MID = 10
    Y_MID = -40
    GAP = 20

    POWER_BUTTON = [X_MID+GAP-5+79, Y_MID-1]

    points = [
        POWER_BUTTON,
        [X_MID, Y_MID + GAP],
        [X_MID - GAP, Y_MID],
        [X_MID, Y_MID - GAP],
        [X_MID + GAP-5, Y_MID]
    ]


    def main5():
        x, y = points[0]
        move_py_fuckArm(x, y)


    def main6():
        x, y = points[0]
        move_py_fuckArm(x, y)
        time.sleep(1)
        nmmbd.set_servo_position(2, 60)
        time.sleep(0.5)
        nmmbd.set_servo_position(2, 0)
        time.sleep(1)

        for _ in range(1):
            for x, y in points[1:]:
                # move_py_fuckArm(X_MID, Y_MID)
                # time.sleep(0.05)
                move_py_fuckArm(x, y)
                time.sleep(1)
                nmmbd.click()
                time.sleep(1.5)


    # main1()
    # main2(2)
    # main3()
    # main4()
    # main5()
    main6()
    # set_servo_position(0, 50)
    # set_servo_position(1, 50)
