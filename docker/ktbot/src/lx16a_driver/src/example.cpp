#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h> 
#include "lx16a_driver/driver.h"
#include <string.h>

using std::string;

int open_port(const char * port) {
  int fd; // file description for the serial port
  
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  
  if(fd == -1) {
    printf("open_port: Unable to open port %s\n", port);
  } else {
    fcntl(fd, F_SETFL, 0);
    printf("Port is open.\n");
  }
  
  return(fd);
}

int set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    printf("Usage: %s <port> <id> <\"wheel\"|\"pos\">\n", argv[0]);
    return 1;
  }

  int ser = open_port(argv[1]);
  if (ser == -1) {
    return 2;
  }
  if (set_interface_attribs(ser, B115200, 0) == -1) {
    return 3;
  }
  set_blocking(ser, false);

  LX16AServo servo;
  int id = atoi(argv[2]);
  servo.attach(ser, id);
  servo.setCompliance(false);

  string mode = argv[3];
  if (mode == "wheel") {
    printf("Wheel mode test\n");
    int spd = 500;
    int i = 0;
    while(true) {
      float deg;
      uint16_t v;
      servo.getPos(&deg);
      servo.getVoltage(&v);
      printf("Current position is %f, voltage is %f\n", deg, v/1000.0);
      usleep(10000);
      if (i >= 160) {
        spd *= -1;
        servo.setWheelMode(spd);
        i = 0;
      }
      i++;
    }
  } else {
    printf("Position mode test\n");
    float pos = 40;
    int i = 0;
    servo.setServoMode();
    while(true) {
      float deg;
      uint16_t v;
      servo.getPos(&deg);
      servo.getVoltage(&v);
      printf("Current position is %f, voltage is %f\n", deg, v/1000.0);
      usleep(10000);
      if (i >= 300) {
        pos = 240 - pos;
        servo.move(pos, 100);
        i = 0;
      }
      i++;
    }
  } 
}