#include "missile_control.hpp"
#include <opencv.hpp>
// This thread is responsible for controlling the servos and launching the missiles

// does what it says
void* missile_control::control_launcher_helper(void *context)
	{
	return((missile_control *)context)->control_launcher_method();
	}

// does what it says
void* missile_control::control_launcher_method()
	{
	int r;

	// set states for when to stop thread
	r = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_setcancelstate returned non zero." << std::endl;
		}

	r = pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_setcanceltype returned non zero." << std::endl;
		}
	while(true)
		{
		pthread_testcancel();

		r = pthread_mutex_lock(&mutex_face_detected);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		int hits_in_a_row_1 = hits_in_a_row;

        // copies things from the evaluation thread for faster performance
		cv::Point center_face_center_1;
		center_face_center_1.x = center_face_center.x;
		center_face_center_1.y = center_face_center.y;

        // safe former target size and coordinates
		cv::Rect target_1;
		target_1.x = target.x;
		target_1.y = target.y;
		target_1.width = target.width;
		target_1.height = target.height;

        // as well as camera specs
		cv::Point camera_center_1;
		camera_center_1.x = camera_center.x;
		camera_center_1.y = camera_center.y;


		int no_face_found_1 = no_face_found;

		r = pthread_mutex_unlock(&mutex_face_detected);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		std::vector<int> posXY (2,6000);


        // if the center of the face is NOT within the target area,
        // move the camera in the direction of the face.
        // this is NO prediction, thus the camera will always only track
        // movement. To predict movement, this part would have to be updated.
		if(!target_1.contains(cv::Point(center_face_center_1.x, center_face_center_1.y)))
			{
            // reset counter until shot is fired.
			hits_in_a_row_1 = 0;

            // save the distance from face center to camera center
			int x_correction, y_correction;
			// correction between face and camera center in x coordinates
			if(center_face_center_1.x < camera_center_1.x)
				{
				//face left from center
				x_correction = camera_center_1.x  - center_face_center_1.x;
				}
			else
				{
				//face right from center
				x_correction = center_face_center_1.x - camera_center_1.x ;
				}
			// correction between face and camera center in y coordinates
			if(center_face_center_1.y < camera_center_1.y)
				{
				//face below center
				y_correction = camera_center_1.y   - center_face_center_1.y;
				}
			else
				{
				//face above center
				y_correction = center_face_center_1.y - camera_center_1.y  ;
				}



            // go to the max right/left point of the servo if face is right/left from the face
			if( center_face_center_1.x < camera_center_1.x )
				{

				posXY[0] = 3968;
				if(r != EXIT_SUCCESS)
					{
					std::cout << "missile_launcher::send_command failed." << std::endl;
					}
				}
			else
				{

				posXY[0] = 8000;
				if(r != EXIT_SUCCESS)
					{
					std::cout << "missile_launcher::send_command failed." << std::endl;
					}
				}

			// go to the max upmost/downmost position of the servo if face is over/under the face
			if(center_face_center_1.y < camera_center_1.y )
				{

				posXY[1] = 8000;
				if(r != EXIT_SUCCESS)
					{
					std::cout << "missile_launcher::send_command failed." << std::endl;
					}
				}
			else
				{

				posXY[1] = 3968;
				if(r != EXIT_SUCCESS)
					{
					std::cout << "missile_launcher::send_command failed." << std::endl;
					}
				}

            // set intervals for x and y speed.
            // Notice that the y speed should always be set to slower values than the
            // x speed to avoid "wobbling" of the launcher.
            // Suitable values for the speed are from 10-100, but can be different
            // for different kind of environments (e.g. larger average distance to faces, ...)
			int ySpeed = min(40,max(10,std::abs(y_correction)/7));
			int xSpeed = min(100,max(20,std::abs(x_correction)/10*4));
			// give servo the correct commands
			servo->setSpeed(xSpeed,ySpeed);
			servo->setPosition(posXY);
			// stop after 10 ms at current position
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			servo->setPosition(servo->getPosition());
			// stop for 50 ms so the camera can take a good picture
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		// if the center of the face is in the target area
		else
			{
			// shoot if the face is in the target area for more than 15 frames and if in the last frame the face was NOT only found by the Kalman filter
			if(hits_in_a_row_1 > 15 && no_face_found_1 == 0)
				{
				std::cout << "*** FIRE ***" << std::endl;
				// send stand command to the rocket launcher (not needed if you use the servos to move)
				r = send_command('s');
				cv::displayOverlay("Capture" ,"******************FIRE******************" ,3000 );
				// send fire command to the rocket launcher
				r = send_command('f');
				if(r != EXIT_SUCCESS)
					{
					std::cout << "missile_launcher::send_command failed." << std::endl;
					}
				r = pthread_mutex_lock(&mutex_face_detected);
				if(r != 0)
					{
					std::cout << "pthread_mutex_lock returned non zero." << std::endl;
					}
				// reset found faces in consecutive frames
				hits_in_a_row = hits_in_a_row_1 = 0;
				r = pthread_mutex_unlock(&mutex_face_detected);
				if(r != 0)
					{
					std::cout << "pthread_mutex_lock returned non zero." << std::endl;
					}

				}
			}

		if(r != EXIT_SUCCESS)
			{
			std::cout << "missile_launcher::send_command failed." << std::endl;
			}
		}
	return(EXIT_SUCCESS);

	}



int missile_control::send_command(char command)
	{
	if(command != last_command || command == 's')
		{
		last_command = command;
		unsigned char data[8];
		data[0] = 0x02;
		data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = 0x00;
		switch(command)
			{
		case 'a':
			//led off
			data[0] = 0x03;
			data[1] = 0x00;
			break;
		case 'A':
			//led on
			data[0] = 0x03;
			data[1] = 0x01;
			break;
		case 's':
			//stop
			data[1] = 0x20;
			break;
		case 'f':
			//fire
			data[1] = 0x10;
			break;
		case 'd':
			//down
			data[1] = 0x01;
			break;
		case 'u':
			//up
			data[1] = 0x02;
			break;
		case 'l':
			//left
			data[1] = 0x04;
			break;
		case 'r':
			//right
			data[1] = 0x08;
			break;
		default:
			//unknown command
			std::cout << "Unknown command: " << command << "." << std::endl;
			return(EXIT_FAILURE);
			}
		//	std::cout << "Command send: " << command << std::endl;
		libusb_control_transfer(dev_handle, 0x21, 0x09, 0, 0, data, sizeof(data), 1000);
		if(command == 'f')
			{
			//	std::this_thread::sleep_for(std::chrono::seconds(3));
			}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	return(EXIT_SUCCESS);
	}

int missile_control::move_launcher(char where, int time)
	{
	int r;
	switch(where)
		{
	case 'u':
	case 'd':
		if(time < 0)
			{
			time = 0;
			}
		else if(time > 750)
			{
			time = 750;
			}
		break;
	case 'l':
	case 'r':
		if(time < 0)
			{
			time = 0;
			}
		else if(time > 6500)
			{
			time = 6500;
			}
		break;
	default:
		std::cout << "Trying to move launcher but received unknown command." << std::endl;
		return(EXIT_FAILURE);
		}
	r = send_command(where);
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_launcher::send_command failed." << std::endl;
		}
	std::this_thread::sleep_for(std::chrono::milliseconds(time));
	r = send_command('s');
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_launcher::send_command failed." << std::endl;
		}
	return(EXIT_SUCCESS);
	}

int missile_control::park_launcher()
	{
	//int r;
	std::cout << "Parking launcher." << std::endl;
	servo->resetPosition();
	/*
	// THIS IS IF YOU USE THE SERVO OF THE LAUNCHER ITSELF
	//park at bottom left position
	//r = move_launcher('l', 1000000);
	if(r != EXIT_SUCCESS) {
		std::cout << "missile_control::move_launcher returned EXIT_FAILURE." << std::endl;
	}
	//r = move_launcher('d', 1000000);
	if(r != EXIT_SUCCESS) {
		std::cout << "missile_control::move_launcher returned EXIT_FAILURE." << std::endl;
	}

	//move to middle position
	//r = move_launcher('r', 3120);
	if(r != EXIT_SUCCESS) {
		std::cout << "missile_control::move_launcher returned EXIT_FAILURE." << std::endl;
	}
	//r = move_launcher('u', 250);
	if(r != EXIT_SUCCESS) {
		std::cout << "missile_control::move_launcher returned EXIT_FAILURE." << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	//set LED off
	r = send_command('a');

	if(r != EXIT_SUCCESS) {
		std::cout << "missile_control::send_command returned EXIT_FAILURE." << std::endl;
	} */

	std::cout << "Launcher in parking position." << std::endl;



	return(EXIT_SUCCESS);
	}
