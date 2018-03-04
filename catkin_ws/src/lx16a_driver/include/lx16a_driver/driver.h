#ifndef LX16A_LIB_H
#define LX16A_LIB_H

#include <stdint.h>

class LX16AServo {
  public:
  LX16AServo();
  void attach(int serial, uint8_t ID);

  // degree 0-240 with 0.24 degree resolution, time 0-30,000ms.
  // If wait == true, the move is queued and run on moveStart()
  void move(float degree, uint16_t millis);
  void waitMove(float degree, uint16_t millis);
  bool getMove(float *degree, uint16_t *time); // Returns 0 on success, populates pos & time.
  bool getWaitMove(float *degree, uint16_t *time);
  void stopMove();
  void startMove();
  bool getPos(float *degree);

  // 0-253. Defaults to 1.
  void setID(uint8_t id);
  bool getID(uint8_t *id);

  // -30 to 30, with 0.24 degree resolution
  void setAngleOffset(float offset, bool save);
  bool getAngleOffset(float *offset);

  // angle limits are 0~1000
  void setAngleLimit(float min, float max);
  bool getAngleLimit(float *min, float *max);

  // 50~100°C
  void setTempLimit(uint8_t max);
  bool getTempLimit(uint8_t *max);
  bool getTemp(uint8_t *temp);

  // Allowed range 4500~12000mv
  void setVoltageLimit(uint16_t min, uint16_t max);
  bool getVoltageLimit(uint16_t *min, uint16_t *max);
  bool getVoltage(uint16_t *mv);

  // allowed speed -1000~1000
  void setWheelMode(int16_t speed);
  void setServoMode();
  bool getMode(bool *isWheel, int16_t *speed);

  void setCompliance(bool compliant);
  bool getCompliance(bool *compliant);

  void setLEDOn(bool on);
  bool getLEDOn(bool *on);

  void setLEDError(bool whenOverTemp, bool whenOverVoltage, bool whenStalled);
  bool getLEDError(bool *whenOverTemp, bool *whenOverVoltage, bool *whenStalled);

  private:
    int ser;
    uint8_t id;

    // Data is command and parameters, len is number of bytes in data.
    void writePacket(uint8_t *data, uint8_t len);
    bool readPacket(uint8_t *buf);
    bool waitForAvailable();
};

#endif // LX16A_LIB_H