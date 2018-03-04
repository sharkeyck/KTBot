#include "lx16a_driver/driver.h"

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

#define LOBOT_DEBUG 1  /*Debug ：print debug value*/

uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

uint16_t ConvertAngle(float degree) {
  return (degree > 240) ? 1000 : (degree < 0) ? 0 : uint16_t(degree / 0.24); 
}

LX16AServo::LX16AServo() {}

void LX16AServo::attach(int serial, uint8_t ID) {
  ser = serial;
  id = ID;
}

void LX16AServo::writePacket(uint8_t *buf, uint8_t len) {
  if (len > 7) return;

  // Max packet size is 10
  uint8_t packet[10];
  packet[0] = packet[1] = LOBOT_SERVO_FRAME_HEADER;
  packet[2] = id;
  packet[3] = len+2; // Data length includes length byte and checksum
  memcpy(packet + 4, buf, len);

  packet[packet[3]+2] = LobotCheckSum(packet);

  /*
  printf("SEND: ");
  for (int i = 0; i < packet[3] + 3; i++)
  {
    printf("%02x:", packet[i]);
  }
  printf("\n");
  */

  // Clear the buffer
  int bytes;
  ioctl(ser, FIONREAD, &bytes);
  char dump[bytes];
  read(ser, dump, bytes);

  write(ser, packet, packet[3] + 3);
}

bool LX16AServo::waitForAvailable() {
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(ser, &readset);
  char n = select(ser + 1, &readset, 0, 0, &timeout);
  
  // check if an error has occured
  if(n < 0) {
    perror("select failed\n");
  } else if (n == 0) {
    return false;   
  } else {
    return true;
  }
}

bool LX16AServo::readPacket(uint8_t *ret) {
  bool frameStarted = false;
  bool receiveFinished = false;
  uint8_t frameCount = 0;
  uint8_t dataCount = 0;
  uint8_t dataLength = 2;
  uint8_t rxBuf;
  uint8_t filbuf[128];
  uint8_t recvBuf[64];
  uint8_t i;

  if (!waitForAvailable()) {
    printf("NO DATA\n");
    return false;
  }

  uint8_t bytes_read;
  while ((bytes_read = read(ser, filbuf, 128)) > 0) {
    for (int i = 0; i < bytes_read; i++) {
      rxBuf = filbuf[i];
      if (!frameStarted) {
        if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
          frameCount++;
          if (frameCount == 2) {
            frameCount = 0;
            frameStarted = true;
            dataCount = 1;
          }
        }
        else {
          frameStarted = false;
          dataCount = 0;
          frameCount = 0;
        }
      }
      if (frameStarted) {
        recvBuf[dataCount] = (uint8_t)rxBuf;
        if (dataCount == 3) {
          dataLength = recvBuf[dataCount];
          if (dataLength > 7) {
            dataCount = 0;
            frameCount = 0;
            frameStarted = false;
          }
          if (dataLength < 3 || dataCount > 7) {
            dataLength = 2;
            frameStarted = false;
            continue;
          }
        }
        dataCount++;
        if (dataCount == dataLength + 3) {
          /*
          printf("RECV: ");
          for (i = 0; i < dataCount; i++) {
            printf("%02x:", recvBuf[i]);
          }
          printf("\n");
          */

          if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
            frameStarted = false;
            memcpy(ret, recvBuf + 4, dataLength);

            // Give the serial line time to clear (reduces error count)
            usleep(100);

            return true;
          }
          printf("CHECKSUM FAIL");
          return false;
        }
      }
    }

    if (!waitForAvailable()) {
      return false;
    }
  }
  printf("READ FAIL");
  return false;
}

void LX16AServo::move(float degree, uint16_t millis)
{
  uint16_t clampedPos = ConvertAngle(degree);
  uint16_t clampedMillis = (millis > 30000) ? 30000 : millis;

  uint8_t buf[] = {
    LOBOT_SERVO_MOVE_TIME_WRITE,
    GET_LOW_BYTE(clampedPos),
    GET_HIGH_BYTE(clampedPos),
    GET_LOW_BYTE(clampedMillis),
    GET_HIGH_BYTE(clampedMillis)
  };

  return LX16AServo::writePacket(buf, 5);
}

void LX16AServo::waitMove(float degree, uint16_t millis) {
  uint16_t clampedPos = ConvertAngle(degree);
  uint16_t clampedMillis = (millis > 30000) ? 30000 : millis;

  uint8_t buf[] = {
    LOBOT_SERVO_MOVE_TIME_WAIT_WRITE,
    GET_LOW_BYTE(clampedPos),
    GET_HIGH_BYTE(clampedPos),
    GET_LOW_BYTE(clampedMillis),
    GET_HIGH_BYTE(clampedMillis)
  };

  return LX16AServo::writePacket(buf, 5);
}

bool LX16AServo::getWaitMove(float *degree, uint16_t *time) {
  uint8_t cmd[] = {LOBOT_SERVO_MOVE_TIME_WAIT_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[7];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *degree = BYTE_TO_HW(ret[2], ret[1]) * 0.24;
  *time = BYTE_TO_HW(ret[4], ret[3]);
  return true;
}

bool LX16AServo::getPos(float *degree) {
  uint8_t cmd[] = {
    LOBOT_SERVO_POS_READ
  };
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[5];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *degree = int16_t(BYTE_TO_HW(ret[2], ret[1])) * 0.24;
  return true;
}

bool LX16AServo::getMode(bool *isWheel, int16_t *speed) {
  uint8_t cmd[] = {LOBOT_SERVO_OR_MOTOR_MODE_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[7];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *isWheel = bool(ret[1]);
  *speed = BYTE_TO_HW(ret[4], ret[3]);
  return true;
}

bool LX16AServo::getCompliance(bool *compliant) {
  uint8_t cmd[] = {LOBOT_SERVO_LOAD_OR_UNLOAD_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *compliant = !ret[1];
  return true;
}

bool LX16AServo::getMove(float *degree, uint16_t *time) {
  uint8_t cmd[] = {LOBOT_SERVO_MOVE_TIME_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[7];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *degree = BYTE_TO_HW(ret[2], ret[1]) * 0.24;
  *time = BYTE_TO_HW(ret[4], ret[3]);
  return true;
}

void LX16AServo::stopMove() {
  uint8_t cmd[] = {LOBOT_SERVO_MOVE_STOP};
  LX16AServo::writePacket(cmd, 1);
}

void LX16AServo::startMove() {
  uint8_t cmd[] = {LOBOT_SERVO_MOVE_START};
  LX16AServo::writePacket(cmd, 1);
}

void LX16AServo::setID(uint8_t id) {
  if (id > 253) {
    return;
  }
  uint8_t cmd[] = {LOBOT_SERVO_ID_WRITE, id};
  LX16AServo::writePacket(cmd, 2);
}

bool LX16AServo::getID(uint8_t *id) {
  uint8_t cmd[] = {LOBOT_SERVO_ID_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *id = ret[1];
  return true;
}

void LX16AServo::setAngleOffset(float offset, bool save) {
  int8_t offs = (offset > 30) ? 125 : (offset < -30) ? -125 : (int8_t(offset / 0.24));
  uint8_t cmd[] = {LOBOT_SERVO_ANGLE_OFFSET_ADJUST, offs};
  LX16AServo::writePacket(cmd, 2);

  if (save) {
    uint8_t cmd2[] = {LOBOT_SERVO_ANGLE_OFFSET_WRITE};
    LX16AServo::writePacket(cmd2, 1);
  }
}

bool LX16AServo::getAngleOffset(float *offset) {
  uint8_t cmd[] = {LOBOT_SERVO_ANGLE_OFFSET_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *offset = int8_t(ret[1]) * 0.24;
  return true;
}

void LX16AServo::setAngleLimit(float min, float max) {
  uint16_t minClamped = ConvertAngle(min);
  uint16_t maxClamped = ConvertAngle(max);
  if (minClamped > maxClamped) {
    return;
  }

  uint8_t cmd[] = {
    LOBOT_SERVO_ANGLE_LIMIT_WRITE,
    GET_LOW_BYTE(minClamped),
    GET_HIGH_BYTE(minClamped),
    GET_LOW_BYTE(minClamped),
    GET_HIGH_BYTE(minClamped),
  };
  LX16AServo::writePacket(cmd, 5);
}

bool LX16AServo::getAngleLimit(float *min, float *max) {
  uint8_t cmd[] = {LOBOT_SERVO_ANGLE_LIMIT_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[7];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *min = BYTE_TO_HW(ret[2], ret[1]) * 0.24;
  *max = BYTE_TO_HW(ret[4], ret[3]) * 0.24;
  return true;
}

void LX16AServo::setTempLimit(uint8_t max) {
  max = (max > 100) ? 100 : (max < 50) ? 50 : max;
  uint8_t cmd[] = {LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, max};
  LX16AServo::writePacket(cmd, 2);
}

bool LX16AServo::getTempLimit(uint8_t *max) {
  uint8_t cmd[] = {LOBOT_SERVO_TEMP_MAX_LIMIT_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *max = ret[1];
  return true;
}

bool LX16AServo::getTemp(uint8_t *temp) {
  uint8_t cmd[] = {LOBOT_SERVO_TEMP_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *temp = ret[1];
  return true;
}

void LX16AServo::setVoltageLimit(uint16_t min, uint16_t max) {
  uint16_t minClamped = (min > 12000) ? 12000 : (min < 4500) ? 4500 : min;
  uint16_t maxClamped = (max > 12000) ? 12000 : (max < 4500) ? 4500 : max;
  if (minClamped > maxClamped) {
    return;
  }

  uint8_t cmd[] = {
    LOBOT_SERVO_VIN_LIMIT_WRITE,
    GET_LOW_BYTE(minClamped),
    GET_HIGH_BYTE(minClamped),
    GET_LOW_BYTE(minClamped),
    GET_HIGH_BYTE(minClamped),
  };
  LX16AServo::writePacket(cmd, 5);
}

bool LX16AServo::getVoltageLimit(uint16_t *min, uint16_t *max) {
  uint8_t cmd[] = {LOBOT_SERVO_VIN_LIMIT_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[7];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *min = BYTE_TO_HW(ret[2], ret[1]);
  *max = BYTE_TO_HW(ret[4], ret[3]);
  return true;
}

bool LX16AServo::getVoltage(uint16_t *mv) {
  uint8_t cmd[] = {
    LOBOT_SERVO_VIN_READ
  };
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[5];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *mv = BYTE_TO_HW(ret[2], ret[1]);
  return true;
}

void LX16AServo::setServoMode() {
  uint8_t cmd[] = {
    LOBOT_SERVO_OR_MOTOR_MODE_WRITE,
    0,
    0,
    0,
    0 
  };
  LX16AServo::writePacket(cmd, 5);
}

void LX16AServo::setWheelMode(int16_t speed) {
  int16_t clampedSpeed = (speed < -1000) ? -1000 : (speed > 1000) ? 1000 : speed;
  uint8_t cmd[] = {
    LOBOT_SERVO_OR_MOTOR_MODE_WRITE,
    1,
    0,
    GET_LOW_BYTE(clampedSpeed),
    GET_HIGH_BYTE(clampedSpeed) 
  };
  LX16AServo::writePacket(cmd, 5);
}

void LX16AServo::setCompliance(bool compliant) {
  uint8_t cmd[] = {LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, !compliant};
  LX16AServo::writePacket(cmd, 2);
}

void LX16AServo::setLEDOn(bool on) {
  uint8_t cmd[] = {LOBOT_SERVO_LED_CTRL_WRITE, !on};
  LX16AServo::writePacket(cmd, 2);
}

bool LX16AServo::getLEDOn(bool *on) {
  uint8_t cmd[] = {LOBOT_SERVO_TEMP_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *on = !ret[1];
  return true;
}

void LX16AServo::setLEDError(bool whenOverTemp, bool whenOverVoltage, bool whenStalled) {
  uint8_t cmd[] = {
    LOBOT_SERVO_LED_ERROR_WRITE,
    ((!!whenOverTemp)
    | ((!!whenOverVoltage) << 1)
    | ((!!whenStalled) << 2))
  };
  LX16AServo::writePacket(cmd, 2);
}

bool LX16AServo::getLEDError(bool *whenOverTemp, bool *whenOverVoltage, bool *whenStalled) {
  uint8_t cmd[] = {LOBOT_SERVO_LED_ERROR_READ};
  LX16AServo::writePacket(cmd, 1);

  uint8_t ret[4];
  if (!LX16AServo::readPacket(ret)) {
    return false;
  }

  *whenOverTemp = !!(ret[1] & 0x01);
  *whenOverVoltage = !!(ret[1] & 0x02);
  *whenStalled = !!(ret[1] & 0x04);
  return true;
}
