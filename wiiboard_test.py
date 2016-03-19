#!/usr/bin/env python

#A majority of this code was taken from InitialState's beerfridge project.
#to find that go to https://github.com/InitialState/beerfridge/wiki

import collections
import time
import bluetooth
import sys
import subprocess
import RPi.GPIO as gpio

# --------- User Settings ---------
WEIGHT_SAMPLES = 10
# ---------------------------------

# Wiiboard Parameters
CONTINUOUS_REPORTING = "04"  # Easier as string with leading zero
COMMAND_LIGHT = 11
COMMAND_REPORTING = 12
COMMAND_REGISTER = 16
COMMAND_READ_REGISTER = 17
INPUT_STATUS = 20
INPUT_READ_DATA = 21
EXTENSION_8BYTES = 32
BUTTON_DOWN_MASK = 8
TOP_RIGHT = 0
BOTTOM_RIGHT = 1
TOP_LEFT = 2
BOTTOM_LEFT = 3
BLUETOOTH_NAME = "Nintendo RVL-WBC-01"

#pwm pins for bot
leftMotorPin = 18
rightMotorPin = 23

#current duty cycles for forward, backward, and off
forwardCycle = 80
backwardCycle = 30
neutralCycle = 0

#init gpio ad broadcom mode
gpio.setmode(gpio.BCM)
#setup pins
gpio.setup(rightMotorPin, gpio.OUT)
gpio.setup(leftMotorPin, gpio.OUT)
rightPwm = gpio.PWM(rightMotorPin, 400)
leftPwm = gpio.PWM(leftMotorPin, 400)
#init state
rightPwm.start(0)
leftPwm.start(0)

class Wiiboard:
    def __init__(self):
        # Sockets and status
        #<changed>
        self.TopLeft = 0
        self.TopRight = 0
        self.BottomLeft = 0
        self.BottomRight = 0
        #</changed>
        self.receivesocket = None
        self.controlsocket = None

        self.calibration = []
        self.calibrationRequested = False
        self.LED = False
        self.address = None
        self.buttonDown = False
        for i in xrange(3):
            self.calibration.append([])
            for j in xrange(4):
                self.calibration[i].append(10000)  # high dummy value so events with it don't register

        self.status = "Disconnected"
        try:
            self.receivesocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
            self.controlsocket = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        except ValueError:
            raise Exception("Error: Bluetooth not found")

    def isConnected(self):
        return self.status == "Connected"

    # Connect to the Wiiboard at bluetooth address <address>
    def connect(self, address):
        if address is None:
            print "Non existant address"
            return
        self.receivesocket.connect((address, 0x13))
        self.controlsocket.connect((address, 0x11))
        if self.receivesocket and self.controlsocket:
            print "Connected to Wiiboard at address " + address
            self.status = "Connected"
            self.address = address
            self.calibrate()
            useExt = ["00", COMMAND_REGISTER, "04", "A4", "00", "40", "00"]
            self.send(useExt)
            self.setReportingType()
            print "Wiiboard connected"
        else:
            print "Could not connect to Wiiboard at address " + address

    def receive(self):
        if self.status == "Connected":
            data = self.receivesocket.recv(25)
            intype = int(data.encode("hex")[2:4])
            if intype == INPUT_STATUS:
                # TODO: Status input received. It just tells us battery life really
                self.setReportingType()
            elif intype == INPUT_READ_DATA:
                if self.calibrationRequested:
                    packetLength = (int(str(data[4]).encode("hex"), 16) / 16 + 1)
                    self.parseCalibrationResponse(data[7:(7 + packetLength)])

                    if packetLength < 16:
                        self.calibrationRequested = False
            elif intype == EXTENSION_8BYTES:
                self.readBoardData(data[2:12])
            else:
                print "ACK to data write received"

    def disconnect(self):
        if self.status == "Connected":
            self.status = "Disconnecting"
            while self.status == "Disconnecting":
                self.wait(100)
        try:
            self.receivesocket.close()
        except:
            pass
        try:
            self.controlsocket.close()
        except:
            pass
        print "WiiBoard disconnected"

    # Try to discover a Wiiboard
    def discover(self):
        print "Press the red sync button on the board now"
        address = None
        bluetoothdevices = bluetooth.discover_devices(duration=6, lookup_names=True)
        for bluetoothdevice in bluetoothdevices:
            if bluetoothdevice[1] == BLUETOOTH_NAME:
                address = bluetoothdevice[0]
                print "Found Wiiboard at address " + address
        if address is None:
            print "No Wiiboards discovered."
        return address

    def readBoardData(self, bytes):
        buttonBytes = bytes[0:2]
        bytes = bytes[2:12]
        buttonPressed = False
        buttonReleased = False

        state = (int(buttonBytes[0].encode("hex"), 16) << 8) | int(buttonBytes[1].encode("hex"), 16)
        if state == BUTTON_DOWN_MASK:
            buttonPressed = True
            if not self.buttonDown:
                print "Button pressed"
                self.buttonDown = True

        #if not buttonPressed:

        rawTR = (int(bytes[0].encode("hex"), 16) << 8) + int(bytes[1].encode("hex"), 16)
        rawBR = (int(bytes[2].encode("hex"), 16) << 8) + int(bytes[3].encode("hex"), 16)
        rawTL = (int(bytes[4].encode("hex"), 16) << 8) + int(bytes[5].encode("hex"), 16)
        rawBL = (int(bytes[6].encode("hex"), 16) << 8) + int(bytes[7].encode("hex"), 16)

        self.TopLeft = self.calcMass(rawTL, TOP_LEFT)
        self.TopRight = self.calcMass(rawTR, TOP_RIGHT)
        self.BottomLeft = self.calcMass(rawBL, BOTTOM_LEFT)
        self.BottomRight = self.calcMass(rawBR, BOTTOM_RIGHT)

    def calcMass(self, raw, pos):
        val = 0.0
        #calibration[0] is calibration values for 0kg
        #calibration[1] is calibration values for 17kg
        #calibration[2] is calibration values for 34kg
        if raw < self.calibration[0][pos]:
            return val
        elif raw < self.calibration[1][pos]:
            val = 17 * ((raw - self.calibration[0][pos]) / float((self.calibration[1][pos] - self.calibration[0][pos])))
        elif raw > self.calibration[1][pos]:
            val = 17 + 17 * ((raw - self.calibration[1][pos]) / float((self.calibration[2][pos] - self.calibration[1][pos])))

        return val

    def getLED(self):
        return self.LED

    def parseCalibrationResponse(self, bytes):
        index = 0
        if len(bytes) == 16:
            for i in xrange(2):
                for j in xrange(4):
                    self.calibration[i][j] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                    index += 2
        elif len(bytes) < 16:
            for i in xrange(4):
                self.calibration[2][i] = (int(bytes[index].encode("hex"), 16) << 8) + int(bytes[index + 1].encode("hex"), 16)
                index += 2

    # Send <data> to the Wiiboard
    # <data> should be an array of strings, each string representing a single hex byte
    def send(self, data):
        if self.status != "Connected":
            return
        data[0] = "52"

        senddata = ""
        for byte in data:
            byte = str(byte)
            senddata += byte.decode("hex")

        self.controlsocket.send(senddata)

    #Turns the power button LED on if light is True, off if False
    #The board must be connected in order to set the light
    def setLight(self, light):
        if light:
            val = "10"
        else:
            val = "00"

        message = ["00", COMMAND_LIGHT, val]
        self.send(message)
        self.LED = light

    def calibrate(self):
        message = ["00", COMMAND_READ_REGISTER, "04", "A4", "00", "24", "00", "18"]
        self.send(message)
        self.calibrationRequested = True

    def setReportingType(self):
        bytearr = ["00", COMMAND_REPORTING, CONTINUOUS_REPORTING, EXTENSION_8BYTES]
        self.send(bytearr)

    def wait(self, millis):
        time.sleep(millis / 1000.0)

def getGraph(val, maxVal):
    toPrint = ""
    percent = val / maxVal * 100
    for i in range(0, int(percent)):
        toPrint += "."
    return toPrint

def getPower(top, bottom, deadzone):
    maxVal = 80.0
    diff = top - bottom
    if abs(diff) <= deadzone:
        return 0
    else:
        percent = abs(diff / maxVal)
        motorVal = 10.0 * percent
        if diff < 0:
            motorVal = motorVal * -1
    return motorVal

def drive(throttle, steer):
    if steer > 0:
        #turn right
        rightPwm.ChangeDutyCycle(backwardCycle);
        leftPwm.ChangeDutyCycle(forwardCycle);
    elif steer < 0:
        #turn left
        rightPwm.ChangeDutyCycle(forwardCycle);
        leftPwm.ChangeDutyCycle(backwardCycle);
    elif throttle > 0:
        #forward
        rightPwm.ChangeDutyCycle(forwardCycle);
        leftPwm.ChangeDutyCycle(forwardCycle);
    elif throttle < 0:
        #backward
        rightPwm.ChangeDutyCycle(backwardCycle);
        leftPwm.ChangeDutyCycle(backwardCycle);
    else:
        #no movement
        rightPwm.ChangeDutyCycle(neutralCycle);
        leftPwm.ChangeDutyCycle(neutralCycle);

def main():
    board = Wiiboard()
    if len(sys.argv) == 1:
        print "Discovering board..."
        address = board.discover()
    else:
        address = sys.argv[1]

    try:
        # Disconnect already-connected devices.
        # This is basically Linux black magic just to get the thing to work.
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
        subprocess.check_output(["bluez-test-input", "disconnect", address], stderr=subprocess.STDOUT)
    except:
        pass

    print "Trying to connect..."
    board.connect(address)  # The wii board must be in sync mode at this time
    board.wait(200)
    # Flash the LED so we know we can step on.
    board.setLight(False)
    board.wait(500)
    board.setLight(True)

    try:
        while 1:
            board.receive()
            sys.stderr.write("\x1b[2J\x1b[H")
            print "topLeft:     " + str(board.TopLeft)
            print "topRight:    " + str(board.TopRight)
            print "bottomLeft:  " + str(board.BottomLeft)
            print "bottomRight: " + str(board.BottomRight)
            top = (board.TopLeft + board.TopRight) / 2
            bottom = (board.BottomLeft + board.BottomRight) / 2
            right = (board.TopRight + board.BottomRight) / 2
            left = (board.TopLeft + board.BottomLeft) / 2
            throttle = getPower(top, bottom, 20)
            steer = getPower(left, right, 20)
            #steer is given priority for now because its harder to do without
            #messing with throttle
            if steer > 0:
                drive(0, 1)
            elif steer < 0:
                drive(0, -1)
            elif throttle > 0:
                drive(1, 0)
            elif throttle < 0:
                drive(-1, 0)
            else:
                drive(0, 0)
    except KeyboardInterrupt:
        #gpio cleanup here
        rightPwm.stop()
        leftPwm.stop()
        gpio.cleanup()
        print "finished"

if __name__ == "__main__":
    main()
