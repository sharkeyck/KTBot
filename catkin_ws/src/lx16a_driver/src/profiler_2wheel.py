import time
import datetime
import serial
import sys
from driver import LX16AServo
sampling_time = 5.0
sample_pd = 0.1
left_servo_id = 1
right_servo_id = 0

print "Connecting to servo"

ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)
servo = LX16AServo(ser)
print("Port is open.")

print "Beginning profile of wheels"

results = {}

vel = [0, 0]
prevPos = None
prevTime = datetime.datetime.now()
def updateVelocity():
  global vel
  global prevPos
  global prevTime
  newPos = [servo.getPos(left_servo_id), servo.getPos(right_servo_id)]
  now = datetime.datetime.now()
  if newPos[0] and newPos[1] and prevPos is None:
    prevPos = newPos
    prevTime = now
  elif newPos[0] and newPos[1]:
    delta = [newPos[0] - prevPos[0], newPos[1] - prevPos[1]];
    if delta[0] > 1 and delta[1] < -1 and delta[0] < 3600 and delta[1] > -360:
      vel = [vel[0] * 0.9 + (newPos[0] - prevPos[0]) / (now - prevTime).total_seconds(), vel[1] * 0.9 + (newPos[1] - prevPos[1]) / (now - prevTime).total_seconds()]
    else:
      vel = [vel[0] * 0.95, vel[1] * 0.95]
    prevPos = newPos
    prevTime = now
    print newPos, vel

for i in xrange(0, 1000, 50):
  print "Testing wheel speed " + str(i) + " for " + str(sampling_time) + "s"
  servo.setWheelMode(left_servo_id, i)
  servo.setWheelMode(right_servo_id, -i)
  end_time = datetime.datetime.now() + datetime.timedelta(seconds=sampling_time)
  t = datetime.datetime.now()
  while t < end_time:
    time.sleep(sample_pd)
    updateVelocity()
    t = datetime.datetime.now()
  results[i] = vel
  print "Got angular speed " + str(results[i])

servo.setWheelMode(left_servo_id, 0)
servo.setWheelMode(right_servo_id, 0)

print "RESULTS BITCHES:"
for i in xrange(0, 1000, 50):
  print str(i) + ', ' + str(results[i][0]) + ', ' + str(results[i][1])

