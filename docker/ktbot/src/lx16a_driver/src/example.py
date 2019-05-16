import serial
import sys
import time

from driver import LX16AServo

if __name__ == "__main__":
  if len(sys.argv) != 3:
    exit("Usage: %s <port> <id>" % sys.argv[0])
  
  ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)
  servo = LX16AServo(ser)
  print("Port is open.")

  while True:
    pos = servo.getPos(int(sys.argv[2]))
    if pos is not None:
      print("Current position is %f" % pos)
    time.sleep(1.0)
