import time
import datetime
import serial
import sys
from driver import LX16AServo
sampling_time = 5.0
sample_pd = 0.1
servo_id = 1

print "Connecting to servo"

ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)
servo = LX16AServo(ser)
print("Port is open.")

print "Beginning profile of wheels"

results = {}

vel = 0
prevPos = 0
prevTime = datetime.datetime.now()
def updateVelocity():
  global vel
  global prevPos
  global prevTime
  newPos = servo.getPos(servo_id)
  now = datetime.datetime.now()
  if newPos and prevPos is None:
    prevPos = newPos
    prevTime = now
  elif newPos:
    delta = newPos - prevPos;
    if delta < -1 and delta > -360:
      vel = vel * 0.9 + (newPos - prevPos) / (now - prevTime).total_seconds()
    else:
      vel = vel * 0.95
    prevPos = newPos
    prevTime = now
    print newPos, vel

for i in xrange(0, 1000, 50):
  print "Testing wheel speed " + str(i) + " for " + str(sampling_time) + "s"
  servo.setWheelMode(servo_id, -i)
  end_time = datetime.datetime.now() + datetime.timedelta(seconds=sampling_time)
  t = datetime.datetime.now()
  while t < end_time:
    time.sleep(sample_pd)
    updateVelocity()
    t = datetime.datetime.now()
  results[i] = vel
  print "Got angular speed " + str(results[i])

servo.setWheelMode(servo_id, 0)

print "RESULTS BITCHES:"
for i in xrange(0, 1000, 50):
  print str(i) + ', ' + str(results[i])

