#ifndef _SERVODEVICE_H_
#define _SERVODEVICE_H_

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <iostream>

class servoDevice {
public:
	servoDevice(void);
	~servoDevice(void);
	//setSpeed does the same both times
	//values between 0 and 255 are valid while 255 is the fastest.
	//0 does not constrain the speed
	void setSpeed(std::vector<int> speed);
	void setSpeed(int speedX, int speedY);
	//setPosition does the same both times
	//values between 3968 and 8000 are valid
	void setPosition(std::vector<int> speed);
	void setPosition(int speedX, int speedY);
	//for simplicity assume a center position at 6000
	void resetPosition(void);
	std::vector<int> getPosition();
private:
	int fd;	//file descriptor for the servo controller
	int maestroGetPosition(unsigned char channel);
	int maestroSetTarget(unsigned char channel, unsigned short target);
	int maestroSetSpeed(unsigned char channel, unsigned short speed);
};

#endif
//_SERVODEVICE_H_
