#include "missile.hpp"

int main(int argc, char **argv) {

	argv[argc] = nullptr;

	// change this value to the value of your camera if you have more than 1 camera (e.g. if you have an integrated laptop camera)
	int camera_number = 0;

//	std::cout << argv[0] << " version: " << VERSION << std::endl << "Author: " << AUTHOR << std::endl;

	missile_control *ctrl = new missile_control(camera_number);
	// main-loop a.k.a "defend()"
	ctrl->defend();
	delete ctrl;

	return(EXIT_SUCCESS);
}

