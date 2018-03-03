#include <stdio.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h> 
#include "driver.h"

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

int configure_port(int fd, speed_t baud) {
  struct termios port_settings;      // structure to store the port settings in

  cfsetispeed(&port_settings, baud);    // set baud rates
  cfsetospeed(&port_settings, baud);

  /*
  port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;
  
  
  tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
  */
  return(fd);
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    printf("Usage: %s <port> <id>\n", argv[0]);
    return 1;
  }

  int ser = open_port(argv[1]);
  if (ser == -1) {
    return 2;
  }
  configure_port(ser, B115200);

  LX16AServo servo;
  int id = atoi(argv[2]);
  servo.attach(ser, id);


  while(true) {
    float deg;
    servo.getPos(&deg);

    printf("Current position is %f\n", deg);
    sleep(1);
  }
  
}