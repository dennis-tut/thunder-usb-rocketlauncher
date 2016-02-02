#include "missile.hpp"

int main(int argc, char **argv) {

	argv[argc] = nullptr;

	int camera_number = 0;

//	std::cout << argv[0] << " version: " << VERSION << std::endl << "Author: " << AUTHOR << std::endl;

	missile_control *ctrl = new missile_control(camera_number);
	// main-loop a.k.a "defend()"
	ctrl->defend();
	delete ctrl;

	return(EXIT_SUCCESS);
}

