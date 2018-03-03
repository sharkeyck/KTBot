#!/usr/bin/env python
# Simple simulation of wheel controller code.
# No saving of registers is supported.

import driver
import roslib
roslib.load_manifest("kt_wheel_controller")
from node import WheelControllerNode

class FakeSerial():
  in_waiting = False

  def read(self):
    return None

  def write(self, val):
    return None

class SimSerial():
  def __init__(self, ser_ids):
    self.servos = [FakeLX16A(i) for i in ser_ids]
    self.in_waiting = False

  def calculate_in_waiting(self):
    self.in_waiting = sum([len(s.outbound) for s in self.servos]) > 0

  def read(self):
    if self.in_waiting:
      for s in self.servos:
        if len(s.outbound) > 0:
          c = chr(s.outbound.pop(0))
          self.calculate_in_waiting()
          return c
    else:
      return None

  def write(self, packet):
    for s in self.servos:
      s.receive_serial(packet)
    self.calculate_in_waiting()


class FakeLX16A():
  CMD_TO_STATE_KEY = {
    1: {"write": True, "register": "MOVE_TIME"},
    2: {"write": False, "register": "MOVE_TIME"},
    7: {"write": True, "register": "MOVE_TIME_WAIT"},
    8: {"write": False, "register": "MOVE_TIME_WAIT"},
    11: {"command_type": "START", "register": "MOVE"},
    12: {"command_type": "STOP", "register": "MOVE"},
    13: {"write": True, "register": "ID"},
    14: {"write": False, "register": "ID"},
    17: {"write": True, "register": "ANGLE_OFFSET"},
    18: {"write": True, "register": "ANGLE_OFFSET"},
    19: {"write": False, "register": "ANGLE_OFFSET"},
    20: {"write": True, "register": "ANGLE_LIMIT"},
    21: {"write": False, "register": "ANGLE_LIMIT"},
    22: {"write": True, "register": "VIN_LIMIT"},
    23: {"write": False, "register": "VIN_LIMIT"},
    24: {"write": True, "register": "TEMP_MAX_LIMIT"},
    25: {"write": False, "register": "TEMP_MAX_LIMIT"},
    26: {"write": False, "register": "TEMP"},
    27: {"write": False, "register": "VIN"},
    28: {"write": False, "register": "POS"},
    29: {"write": True, "register": "SERVO_OR_MOTOR_MODE"},
    30: {"write": False, "register": "SERVO_OR_MOTOR_MODE"},
    31: {"write": True, "register": "LOAD_OR_UNLOAD"},
    32: {"write": False, "register": "LOAD_OR_UNLOAD"},
    33: {"write": True, "register": "LED_CTRL"},
    34: {"write": False, "register": "LED_CTRL"},
    35: {"write": True, "register": "LED_ERROR"},
    36: {"write": False, "register": "LED_ERROR"},
  }

  def __init__(self, ser_id = 1):
    self.outbound = []
    self.registers = {
      "MOVE_TIME": [0, 0, 0, 0],
      "MOVE_TIME_WAIT": [0, 0, 0, 0],
      "ID": [1],
      "ANGLE_OFFSET": [0],
      "ANGLE_LIMIT": [0, 0, 0xE8, 0x03], # min 0, max 1k
      "VIN_LIMIT": [0], # 6500, 12000
      "TEMP_MAX_LIMIT": [0], # 85C
      "TEMP": [0],
      "VIN": [0, 0],
      "POS": [0, 0],
      "SERVO_OR_MOTOR_MODE": [0,0,0,0],
      "LOAD_OR_UNLOAD": [0],
      "LED_CTRL": [0],
      "LED_ERROR": [0x07],
    }
    self.state = {
      "id": ser_id,
      "pos": 0.0,
      "vin": 7.4,
      "temp_c": 20.0,
    }

  def compute_checksum(self, packet):
    temp = 0;
    i = 2
    while (i < packet[3] + 2):
      temp += packet[i];
      i += 1

    temp = ~temp;
    i = temp & 0xFF;
    return i;

  def receive_serial(self, packet):
    if (type(packet) is not list):
      raise Exception('only list type supported for servo sim')

    if (packet[0] != driver.SERVO_CONSTANTS.FRAME_HEADER or packet[1] != driver.SERVO_CONSTANTS.FRAME_HEADER):
      print self.state["id"], "Header mismatch"
      return

    id = packet[2]

    if self.state["id"] != id:
      return

    data_len = packet[3]
    cmd = packet[4]
    params = packet[5:(5+data_len-3)] # exclude checksum, length, and command bytes
    checksum = packet[2+data_len]
    computed_checksum = self.compute_checksum(packet)
    if checksum != computed_checksum:
      print self.state["id"], "Checksum mismatch (0x%02x vs computed 0x%02x)" % (checksum. computed_checksum)
      return

    resolved = self.CMD_TO_STATE_KEY[cmd]
    if not resolved:
      print self.state["id"], "Unknown command id 0x%02x" % cmd
      return

    if resolved["write"]:
      self.registers[resolved["register"]] = list(params)
      print self.state["id"], "Wrote", cmd
    else:
      self.outbound += self.make_response_packet(cmd, list(self.registers[resolved["register"]]))
      print self.state["id"], "Reading", cmd

  def make_response_packet(self, cmd, params):
    no_checksum = [
      driver.SERVO_CONSTANTS.FRAME_HEADER,
      driver.SERVO_CONSTANTS.FRAME_HEADER,
      self.state["id"],
      len(params) + 3, # include checksum, length, and command bytes
      cmd
    ] + params
    return no_checksum + [self.compute_checksum(no_checksum)]

  def update(self, pos=None, vin=None, temp=None):
    # Update state of the servo (using e.g. gazebo simulation)
    if pos is not None:
      self.state["pos"] = [ord(c) for c in struct.pack('h', pos)]

    if vin is not None:
      self.state["vin"] = vin
      self.registers["vin"] = [ord(c) for c in struct.pack('H', vin)]

    if temp is not None:
      self.state["temp"] = temp
      self.registers["temp"] = [temp]

if __name__ == "__main__":
  ser = SimSerial([0,1])
  robot = WheelControllerNode(ser)
  robot.loop()
