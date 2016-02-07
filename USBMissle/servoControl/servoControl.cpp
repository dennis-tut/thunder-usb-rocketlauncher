// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
// For further information you can look up the user manual for the Micro Maestro 6-channel at servocity.com

#include "servoControl.h"

servoDevice::servoDevice(void) {
	//Open the Maestro's virtual COM port.
	const char *device = "/dev/ttyACM0"; //Linux
	//if you are a Windows (wo)man you can use "\\\\.\\USBSER000". This solution is not checked though
	
	fd = open(device, O_RDWR | O_NOCTTY);
	if (fd == -1) {
		perror(device);
	}
}

servoDevice::~servoDevice(void) {
	resetPosition();
	close(fd);
}

//BOTH SPEED FUNCTIONS DO THE SAME
//set Speed from 0 to 255. 0 is no limitation
void servoDevice::setSpeed(std::vector<int> speed) {
	if (speed.size() != 2) {
		perror("error: speed vector has wrong length\n");
	} else {
		this->maestroSetSpeed(0, speed[0]);
		this->maestroSetSpeed(1, speed[1]);
	}
}

//set Speed from 0 to 255. 0 is no limitation
void servoDevice::setSpeed(int speedX, int speedY) {
	this->maestroSetSpeed(0, speedX);
	this->maestroSetSpeed(1, speedY);	
}

//values between 3968 and 8000 are valid
void servoDevice::setPosition(std::vector<int> pos) {
	if (pos.size() != 2) {
		perror("error: position vector has wrong length\n");
	} else {
		this->maestroSetTarget(0, pos[0]);
		this->maestroSetTarget(1, pos[1]);
	}
}

//values between 3968 and 8000 are valid
void servoDevice::setPosition(int posX, int posY) {
	this->maestroSetTarget(0, posX);
	this->maestroSetTarget(1, posY);	
}

// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int servoDevice::maestroGetPosition(unsigned char channel) {
	unsigned char command[] = {0x90, channel};
	if (write(fd, command, sizeof(command)) == -1) {
		perror("error writing at maestroGetPosition\n");
		return -1;
	}
	unsigned char response[2];
	if (read(fd, response, 2) != 2) {
		perror("error reading at maestroGetPosition\n");
		return -1;
	}
	
	return response[0] + 256 * response[1];
}

//Sets the target of a Maestro channel.
//See the "Serial Servo Commands" section of the user's guide
//The units of 'target' are quarter-microseconds.
int servoDevice::maestroSetTarget(unsigned char channel, unsigned short target) {
	//user manual pdf page 41 for extensive explanation
	unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1) {
		perror("error writing at maestroSetTarget\n");
		return -1;
	}
	return 0;
}

int servoDevice::maestroSetSpeed(unsigned char channel, unsigned short speed) {
	if (speed < 0 || speed > 255) {
		perror("error: set a correct speed\n");
		return -1;
	}
	unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1) {
		perror("error writing the speed\n");
		return -1;
	}
	return 0;
}

void servoDevice::resetPosition(void) {
	maestroSetTarget(0, 6000);
	maestroSetTarget(1, 6000);
}

int main() {
	
	
	// channel 0 is for vertical rotation, channel 1 for horizontal rotation
	// theoretically a 2-axis rotation at the same time should be possible, but currently we'd have to wait for the return of one function
	// should be implemented with pthreads or something similar
	servoDevice *device = new servoDevice();
	
	device->setSpeed(5,5);
	device->setPosition(4000, 4000);
	sleep(2);
	std::vector<int> *test = new std::vector<int>();
	test->push_back(40);
	test->push_back(40);
	device->setSpeed(*test);
	test->at(0) = 8000;
	test->at(1) = 8000;
	device->setPosition(*test);
	
	delete test;
	delete device;
	return 1;
}







