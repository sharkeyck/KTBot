"""
ROS Bindings can be found in the kt_node package.
"""
import thread, struct
import serial

SERVO_CONSTANTS = {
  FRAME_HEADER: 0x55,
  MOVE_TIME_WRITE: 1,
  MOVE_TIME_READ: 2,
  MOVE_TIME_WAIT_WRITE: 7,
  MOVE_TIME_WAIT_READ: 8,
  MOVE_START: 11,
  MOVE_STOP: 12,
  ID_WRITE: 13,
  ID_READ: 14,
  ANGLE_OFFSET_ADJUST: 17,
  ANGLE_OFFSET_WRITE: 18,
  ANGLE_OFFSET_READ: 19,
  ANGLE_LIMIT_WRITE: 20,
  ANGLE_LIMIT_READ: 21,
  VIN_LIMIT_WRITE: 22,
  VIN_LIMIT_READ: 23,
  TEMP_MAX_LIMIT_WRITE: 24,
  TEMP_MAX_LIMIT_READ: 25,
  TEMP_READ: 26,
  VIN_READ: 27,
  POS_READ: 28,
  SERVO_OR_MOTOR_MODE_WRITE: 29,
  SERVO_OR_MOTOR_MODE_READ: 30,
  LOAD_OR_UNLOAD_WRITE: 31,
  LOAD_OR_UNLOAD_READ: 32,
  LED_CTRL_WRITE: 33,
  LED_CTRL_READ: 34,
  LED_ERROR_WRITE: 35,
  LED_ERROR_READ: 36,
}

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

INFREQUENT_STATUS_PD = 5.0
POS_UPDATE_PD = 0.25

class WheelController():

  def __init__(self, port="/dev/ttyUSB0", positionCallback=None, infrequentCallback=None):
    self.ser = serial.Serial(port, 115200, timeout=0.5)
    self.positionCallback = positionCallback
    self.infrequentCallback = infrequentCallback
    th = thread.start_new_thread(self.read_status, ())

  def read_status(self):
    """ Read position, voltage, and temperature per wheel."""

    lastStatus = time.time()
    while True:
      # Update Vin & temp every ~5s
      # Update Pos every ~0.25s
      now = time.time()
      if now > lastStatus + INFREQUENT_STATUS_PD:
        temperatures = [self.getTemp(LEFT_WHEEL_ID), self.getTemp(RIGHT_WHEEL_ID)]
        voltages = [self.getVoltage(LEFT_WHEEL_ID), self.getVoltage(RIGHT_WHEEL_ID)]
        self.infrequentCallback(temperatures, voltages)

      self.positionCallback([self.getPos(LEFT_WHEEL_ID), self.getPos(RIGHT_WHEEL_ID)])
      time.sleep(POS_UPDATE_PD)

  def set_cmd_vel(self, cmd_vel):
    self.setWheelMode(LEFT_WHEEL_ID, cmd_vel[0] * 1000 / MAX_SPEED_MPS)
    self.setWheelMode(RIGHT_WHEEL_ID, cmd_vel[1] * 1000 / MAX_SPEED_MPS)

  # ---------------- Low level servo stuff below --------------------

  def checksum(packet):
    temp = 0;
    i = 2
    while (i < packet[3] + 2):
      temp += packet[i];
      i++

    temp = ~temp;
    i = (uint8_t)temp;
    return i;

  def write(self, id, buf):
    if (len > DATA_LENGTH_MAX) raise Exception("Invalid write buffer size");

    # Max packet size is 10
    packet = [
      SERVO_CONSTANTS.FRAME_HEADER,
      SERVO_CONSTANTS.FRAME_HEADER,
      id,
      len(buf)+2 # Data length includes length byte and checksum
    ]
    packet += buf
    packet.append(checksum(packet))

    print("Sending")

    # Clear the read buffer
    while self.ser.in_waiting:
      self.ser.read()

    self.ser.write(packet, packet[3] + 3)

  def read(self):
    frameStarted = False
    receiveFinished = False
    headerStartCount = 0
    dataCount = 0
    dataLength = 2
    rxBuf = ""
    recvBuf = [0] * 64
    i = 0

    # See https://www.dropbox.com/sh/b3v81sb9nwir16q/AACkK-tg0q39fKJZcSl-YrqOa/LX-16A%20Bus%20Servo?dl=0&preview=LewanSoul+Bus+Servo+Communication+Protocol.pdf
    # for communication protocol
    while rxBuf is not None:
      rxBuf = self.ser.read();

      # Detect when the frame starts (two bytes of FRAME_HEADER)
      if not frameStarted:
        if rxBuf == SERVO_CONSTANTS.FRAME_HEADER:
          headerStartCount++
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
        recvBuf[dataCount] = ord(rxBuf)

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
          frameStarted = False;
          print "RECV: " + ", ".join([hex(b) for b in recvBuf])
          # If checksum matches, we've found our packet
          if checksum(recvBuf) == recvBuf[dataCount - 1]:
            return recvBuf
          else:
            print "CHECKSUM FAIL"

    print "READ FAIL"
    return None

  def getPos(id):
    self.write(id, [SERVO_CONSTANTS.POS_READ])

    result = self.read()
    if not result:
      return None

    return struct.unpack('h', ret[2], ret[1]) * WHEEL_POS_RESOLUTION_DEG

  def getTemp(id):
    self.write(id, [SERVO_CONSTANTS.TEMP_READ])

    result = self.read()
    if not result:
      return None

    return ret[1]

  def getVoltage(id):
    self.write(id, [SERVO_CONSTANTS.VIN_READ])

    result = self.read()
    if not result:
      return None

    return struct.unpack('H', ret[2], ret[1]) * WHEEL_POS_RESOLUTION_DEG

  def setWheelMode(id, speed):
    # Wheel speed is -1000 to 1000
    speed = round(max(min(1000, speed), -1000))
    self.write(id, [
      SERVO_CONSTANTS.SERVO_OR_MOTOR_MODE_WRITE,
      1,
      0
    ] + struct.pack('h', speed))
