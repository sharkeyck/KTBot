"""
ROS Bindings can be found in the kt_node package.
"""
import thread, struct
import rospy

class SERVO_CONSTANTS:
  FRAME_HEADER = 0x55
  MOVE_TIME_WRITE = 1
  MOVE_TIME_READ = 2
  MOVE_TIME_WAIT_WRITE = 7
  MOVE_TIME_WAIT_READ = 8
  MOVE_START = 11
  MOVE_STOP = 12
  ID_WRITE = 13
  ID_READ = 14
  ANGLE_OFFSET_ADJUST = 17
  ANGLE_OFFSET_WRITE = 18
  ANGLE_OFFSET_READ = 19
  ANGLE_LIMIT_WRITE = 20
  ANGLE_LIMIT_READ = 21
  VIN_LIMIT_WRITE = 22
  VIN_LIMIT_READ = 23
  TEMP_MAX_LIMIT_WRITE = 24
  TEMP_MAX_LIMIT_READ = 25
  TEMP_READ = 26
  VIN_READ = 27
  POS_READ = 28
  SERVO_OR_MOTOR_MODE_WRITE = 29
  SERVO_OR_MOTOR_MODE_READ = 30
  LOAD_OR_UNLOAD_WRITE = 31
  LOAD_OR_UNLOAD_READ = 32
  LED_CTRL_WRITE = 33
  LED_CTRL_READ = 34
  LED_ERROR_WRITE = 35
  LED_ERROR_READ = 36

DATA_LENGTH_BYTE_IDX = 3
DATA_LENGTH_MAX = 7
DATA_LENGTH_MIN = 3
WHEEL_POS_RESOLUTION_DEG = 0.24
MAX_WHEEL_RPM = 62.5
WHEEL_CIRCUMFERENCE_METERS = 0.204204
BASE_WIDTH_METERS = 0.165
MAX_SPEED_MPS = (MAX_WHEEL_RPM / 60.0) * WHEEL_CIRCUMFERENCE_METERS

LEFT_WHEEL_ID = 0
RIGHT_WHEEL_ID = 1

class LX16AServo():

  def __init__(self, ser):
    self.ser = ser

  # ---------------- Low level servo stuff below --------------------

  def checksum(self, packet):
    temp = 0;
    i = 2
    while (i < packet[3] + 2):
      temp += packet[i];
      i += 1

    temp = ~temp;
    i = temp & 0xFF;
    return i;

  def write(self, id, buf):
    if (len(buf) > DATA_LENGTH_MAX):
      raise Exception("Invalid write buffer size")

    # Max packet size is 10
    packet = [
      SERVO_CONSTANTS.FRAME_HEADER,
      SERVO_CONSTANTS.FRAME_HEADER,
      id,
      len(buf)+2 # Data length includes length byte and checksum
    ]
    packet += buf
    packet.append(self.checksum(packet))

    # Clear the read buffer
    while self.ser.in_waiting:
      self.ser.read()

    # print("SEND: " + ", ".join([hex(p) for p in packet]))
    self.ser.write(packet)

  def read(self):
    frameStarted = False
    receiveFinished = False
    headerStartCount = 0
    dataCount = 0
    dataLength = 2
    recvBuf = [0] * 64
    i = 0

    # See https://www.dropbox.com/sh/b3v81sb9nwir16q/AACkK-tg0q39fKJZcSl-YrqOa/LX-16A%20Bus%20Servo?dl=0&preview=LewanSoul+Bus+Servo+Communication+Protocol.pdf
    # for communication protocol
    while True:
      c = self.ser.read()
      if not c:
        print "NO RESPONSE"
        return None
      c = ord(c)

      # Detect when the frame starts (two bytes of FRAME_HEADER)
      if not frameStarted:
        if c == SERVO_CONSTANTS.FRAME_HEADER:
          headerStartCount += 1
          if (headerStartCount == 2):
            headerStartCount = 0
            frameStarted = True
            dataCount = 1
        else:
          frameStarted = False
          dataCount = 0
          headerStartCount = 0

      # Read in data when frame has started
      if frameStarted:
        recvBuf[dataCount] = c

        # Parse length byte
        if dataCount == DATA_LENGTH_BYTE_IDX:
          dataLength = recvBuf[dataCount]

          # Start over if we've parsed a bad length byte
          if dataLength > DATA_LENGTH_MAX:
            dataCount = 0
            headerStartCount = 0
            frameStarted = False
            continue
          elif dataLength < DATA_LENGTH_MIN:
            dataLength = 2
            frameStarted = False
            continue

        dataCount += 1
        if dataCount == dataLength + 3:
          frameStarted = False
          # If checksum matches, we've found our packet
          if self.checksum(recvBuf) == recvBuf[dataCount - 1]:
            recvBuf = recvBuf[4:recvBuf[3]+2]
            # print "RECV: " + ", ".join([hex(b) for b in recvBuf])
            return recvBuf
          else:
            print "CHECKSUM FAIL"

    print "READ FAIL"
    return None

  def getPos(self, id):
    self.write(id, [SERVO_CONSTANTS.POS_READ])

    result = self.read()
    if not result:
      return None

    return ((result[2] << 8) + result[1]) * WHEEL_POS_RESOLUTION_DEG

  def getTemp(self, id):
    self.write(id, [SERVO_CONSTANTS.TEMP_READ])

    result = self.read()
    if not result:
      return None

    return result[1]

  def getVoltage(self, id):
    self.write(id, [SERVO_CONSTANTS.VIN_READ])

    result = self.read()
    if not result:
      return None

    return ((result[2] << 8) + result[1]) * WHEEL_POS_RESOLUTION_DEG

  def setWheelMode(self, id, speed):
    # Wheel speed is -1000 to 1000
    speed = round(max(min(1000, speed), -1000))
    self.write(id, [
      SERVO_CONSTANTS.SERVO_OR_MOTOR_MODE_WRITE,
      1,
      0
    ] + [ord(c) for c in struct.pack('<h', speed)])
