#include "missile_control.hpp"

//init Kalman
cv::KalmanFilter KF;
cv::Mat_<float> measurement(2,1);

missile_control::missile_control(int c)
	{
	int r;
	std::cout << "Constructor called." << std::endl;
	camera_number = c;

	r = pthread_mutex_init(&mutex_mat_frame, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_mutex_init returned non zero." << std::endl;
		}

	r= pthread_mutex_init(&mutex_vector_rectangles_to_draw, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_mutex_init returned non zero." << std::endl;
		}

	r= pthread_mutex_init(&mutex_face_detected, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_mutex_init returned non zero." << std::endl;
		}

	r = initialize_camera();
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_control::initialize_camera failed." << std::endl;
		}

	r = initialize_detector();
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_control::initialize_detector failed." << std::endl;
		}

	//initialize various counters
	frame_counter = frame_analyzed_counter = frame_analyzed = 0;

	r = initialize_launcher();
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_control::initialize_launcher failed." << std::endl;
		}

	servo = new servoDevice();
	if(servo == NULL)
		{
		std::cout << "missile_control::NewServoDevice failed." << std::endl;
		}

	r = park_launcher();
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_control::park_launcher failed." << std::endl;
		}



	//make to rectangles fit into array (target rectangle and detected face rectangle)
	rectangles_to_draw.resize(2);
	no_face_found = 0;
	misses_in_a_row = 0;
	hits_in_a_row = 0;
	led_is_on = false;
	}

missile_control::~missile_control()
	{
	int r;
	std::cout << "Destructor called." << std::endl;

	delete servo;
	//turn led off
	r = send_command('a');
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_launcher::send_command failed." << std::endl;
		}

	std::cout << "frame_counter: " << frame_counter << " frame_analyzed_counter: " << frame_analyzed_counter << std::endl;

	r = pthread_mutex_destroy(&mutex_mat_frame);
	if(r != 0)
		{
		std::cout << "pthread_mutex_destroy returned non zero." << std::endl;
		}

	r = pthread_mutex_destroy(&mutex_vector_rectangles_to_draw);
	if(r != 0)
		{
		std::cout << "pthread_mutex_destroy returned non zero." << std::endl;
		}

	r = pthread_mutex_destroy(&mutex_face_detected);
	if(r != 0)
		{
		std::cout << "pthread_mutex_destroy returned non zero." << std::endl;
		}
	r = destroy_launcher();
	if(r != EXIT_SUCCESS)
		{
		std::cout << "missile_control::destroy_launcher failed." << std::endl;
		}



	}

int missile_control::initialize_camera()
	{
	int r;
	r = cap.open(camera_number);
	if(r != true)
		{
		std::cout << "cv::VideoCapture::open failed." << std::endl;
		return(EXIT_FAILURE);
		}

	if(!cap.isOpened())
		{
		std::cout << "cv::VideoCapture::isOpened returned non zero." << std::endl;
		return(EXIT_FAILURE);
		}
	else
		{
		camera_width = (int) cap.get(CV_CAP_PROP_FRAME_WIDTH);
		camera_height = (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		camera_center = cv::Point(camera_width / 2, camera_height / 2);
		return(EXIT_SUCCESS);
		}
	}

int missile_control::defend()
	{
	//This function starts two threads.
	//Thread one will continuously  grab frames from the camera and will display live video + detected rectangles
	//Thread two will analyze captured frames and control the missile launcher

	int r;

	r = pthread_create(&camera_thread, nullptr, &grab_and_display_video_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	r = pthread_create(&detect_and_control_thread, nullptr, &detect_faces_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	r = pthread_create(&control_launcher_thread, nullptr, &control_launcher_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	//wait for both threads to finish
	r = pthread_join(camera_thread, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_join returned non zero." << std::endl;
		}

	r = pthread_join(detect_and_control_thread, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_join returned non zero." << std::endl;
		}
	return(EXIT_SUCCESS);
	}

int missile_control::initialize_detector()
	{
	if(!det.load(frontal_face_xml))
		{
		std::cout << "cv::CascadeClassifier::load returned false." << std::endl;
		return(EXIT_FAILURE);
		}
	if(det.empty())
		{
		std::cout << "cv::CascadeClassifier::empty returned true." << std::endl;
		return(EXIT_FAILURE);
		}
	return(EXIT_SUCCESS);
	}

int missile_control::initialize_launcher()
	{
    // libusb stuff. thanks to the errors self-explanatory.
    // the "magic" constants didn't need to be changed, so it should be fine.
	int r;
	r = libusb_init(&ctx);
	if(r != 0)
		{
		std::cout << "libusb_init returned non zero." << std::endl;
		}

	dev_handle = libusb_open_device_with_vid_pid(ctx, 0x2123, 0x1010);

	if( dev_handle == NULL )
		{
		std::cout << "libusb_open_device_with_vid_pid returned NULL dev handle!" << std::endl;
		}

	r = libusb_kernel_driver_active(dev_handle, 0);
	if(r == 0)
		{
		std::cout << "No active kernel driver detected." << std::endl;
		}
	else if(r == 1)
		{
		std::cout << "Kernel driver attached." << std::endl;
		kernel_driver_active = true;
		libusb_detach_kernel_driver(dev_handle, 0);
		}
	else
		{
		std::cout << "Kernel-driver-active-check failed." << std::endl;
		}

	r = libusb_claim_interface(dev_handle, 0x00);
	if(r == 0)
		{
		std::cout << "Successfully claimed usb interface." << std::endl;
		}
	else
		{
		std::cout << "Claiming usb interface failed." << std::endl;
		}
	return(EXIT_SUCCESS);
	}



int missile_control::destroy_launcher()
	{
	int r;
	r = libusb_release_interface(dev_handle, 0);
	if(r == 0)
		{
		std::cout << "Released usb interface." << std::endl;
		}
	else
		{
		std::cout << "Could not release usb interface." << std::endl;
		}
	if(kernel_driver_active)
		{
		r = libusb_attach_kernel_driver(dev_handle, 0);
		if(r == 0)
			{
			std::cout << "Reattached kernel driver." << std::endl;
			}
		else
			{
			std::cout << "Could not reattach the previously attached kernel driver." << std::endl;
			}
		}
	libusb_close(dev_handle);
	libusb_exit(ctx);
	return(EXIT_SUCCESS);
	}

void* missile_control::grab_and_display_video_helper(void *context)
	{
	return((missile_control *)context)->grab_and_display_video();
	}

void* missile_control::grab_and_display_video()
	{
	int r;
	cv::namedWindow("Capture", cv::WINDOW_NORMAL);
	while(true)
		{

		r = pthread_mutex_lock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}
        // this actually provides the next frame
		cap >> frame;
		frame_counter++;

		pthread_mutex_unlock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
			}

		//draw rectangles if mutex is free && detected rectangles are more or less up to date
		// unklar
		r = pthread_mutex_trylock(&mutex_vector_rectangles_to_draw);
		if(r == 0)
			{
			if(frame_counter - frame_analyzed < 3  && misses_in_a_row == 0)
				{
				cv::rectangle(frame, rectangles_to_draw[0], cv::Scalar(0, 0, 255), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[1], cv::Scalar(255, 0, 0), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[2], cv::Scalar(0, 255, 0), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[3], cv::Scalar(255, 255, 0), 1, 8, 0);
				//		cv::putText(frame,"This is a test",cv::Point(100,100), 2,1,cv::Scalar(200,200,100));
				//cv::displayOverlay("Capture" ,"FIRE" ,0 );
				cv::displayStatusBar("Capture" ,"Toolbar" ,0);
				}
			r = pthread_mutex_unlock(&mutex_vector_rectangles_to_draw);
			if(r != 0)
				{
				std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
				}
			}

        // pictre the captured frame
		cv::imshow("Capture", frame);

        // unklar. sorgt das für Beendigung?
		if(cv::waitKey(30) == 27)
			{
			std::cout << "ESC key pressed." << std::endl;

			r = pthread_cancel(detect_and_control_thread);
			if(r != 0)
				{
				std::cout << "pthread_cancle returned non zero." << std::endl;
				}
			break;
			}
		}
	return(nullptr);
	}
void* missile_control::control_launcher_helper(void *context)
	{
	return((missile_control *)context)->control_launcher_method();
	}

void* missile_control::control_launcher_method()
	{
	int r;

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

        // assign face center to unklar
		cv::Point center_face_center_1;
		center_face_center_1.x = center_face_center.x;
		center_face_center_1.y = center_face_center.y;

        // safe former target size and coordinates ...
		cv::Rect target_1;
		target_1.x = target.x;
		target_1.y = target.y;
		target_1.width = target.width;
		target_1.height = target.height;

        // as well as camera specs
		cv::Point camera_center_1;
		camera_center_1.x = camera_center.x;
		camera_center_1.y = camera_center.y;

		int camera_height_1 = camera_height;

		int camera_width_1 = camera_width;

		int face_distance_from_camera_center_1 = face_distance_from_camera_center;
		int no_face_found_1 = no_face_found;

		r = pthread_mutex_unlock(&mutex_face_detected);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		std::vector<int> posXY (2,6000);


        // unklar
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



            // unklar vermutlich Fehlermeldungen wenn außer range?
			if( center_face_center_1.x < camera_center_1.x )  //- target_1.size().width / 2 ) {
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
			// print value for debugging purposes. if the speed is set to one of the
            // boundaries most of the time, you can probably try and adapt those values.
			std::cout<< xSpeed << std::endl;
			//std::cout<< ySpeed << std::endl;
			servo->setSpeed(xSpeed,ySpeed);
			servo->setPosition(posXY);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			servo->setPosition(servo->getPosition());
			std::this_thread::sleep_for(std::chrono::milliseconds(50));

			//}
			}
		else
			{
			if(hits_in_a_row_1 > 15 && no_face_found_1 == 0)
				{
				std::cout << "*** FIRE ***" << std::endl;
				r = send_command('s');
				cv::displayOverlay("Capture" ,"******************FIRE******************" ,3000 );
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

void* missile_control::detect_faces_helper(void *context)
	{
	return((missile_control *)context)->detect_faces();
	}

void* missile_control::detect_faces()
	{
	int r;

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
	// Init Kalman with Camera_center
	kalmanInit(250, 200);
	width_last_frame = 0;
	center_face_center = cv::Point(0,0);
	center_last_frame = cv::Point(0,0);
	width_last_frame = 0;
	face_distance_from_camera_center = std::numeric_limits<int>::max();

	while(true)
		{

		//thread can be canceled at this point
		pthread_testcancel();

		r = pthread_mutex_lock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		frame.copyTo(frame_analyze);
		frame_analyzed = frame_counter;

		r = pthread_mutex_unlock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
			}

		++frame_analyzed_counter;

		if(frame_analyze.empty())
			{
			std::cout << "cv::Mat::empty returned true. Skipping frame." << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
			}

		//equalize histogramm
		cv::Mat img_hist_equalized;
		cv::Mat cropped_image;
		cvtColor(frame_analyze, frame_analyze, CV_BGR2GRAY);

		// If face was detected last frame, only search for a Face nearby
		if(width_last_frame != 0)
			{
			int corner_x = max(center_last_frame.x - width_last_frame/2 , 0);
			int corner_y = max(center_last_frame.y - width_last_frame/2 , 0);
			int width = min(center_last_frame.x + width_last_frame * 3/2, camera_width);
			int height = min(center_last_frame.y + width_last_frame * 3/2 , camera_height);


			cropped_image = frame_analyze(cv::Rect(corner_x , corner_y , width , height));
			corner_cropped_image = cv::Point(corner_x , corner_y);
			}
		else
			{
			cropped_image = frame_analyze;
			corner_cropped_image = cv::Point(0,0);
			}

		cv::equalizeHist(cropped_image, img_hist_equalized);
		det.detectMultiScale(
		    img_hist_equalized,
		    faces,
		    1.04,
		    15,
		    0,
		    cv::Size(camera_width / 12, camera_height / 12),
		    cv::Size(camera_width , camera_height )
		);

		r = control_launcher();
		if(r != EXIT_SUCCESS)
			{
			std::cout << "missile_launcher::control_launcher failed." << std::endl;
			}
		}
	}
void missile_control::kalmanInit(float x, float y)
	{
	//  measurement = Mat_<float>::zeros(2,1);
	target.x = camera_center.x - 100;
	target.y = camera_center.y - 100;
	target.width = 100;
	target.height = 75*2;

	KF.init(4, 2, 0);
	measurement.at<float>(0, 0) = x;
	measurement.at<float>(0, 0) = y;

	KF.statePre.setTo(0);
	KF.statePre.at<float>(0, 0) = x;
	KF.statePre.at<float>(1, 0) = y;

	KF.statePost.setTo(0);
	KF.statePost.at<float>(0, 0) = x;
	KF.statePost.at<float>(1, 0) = y;

	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,2,0,   0,1,0,0.5,  0,0,1,0,  0,0,0,1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(.005)); //adjust this for faster convergence - but higher noise
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	}

Point missile_control::kalmanPredict()
	{
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

	KF.statePre.copyTo(KF.statePost);
	KF.errorCovPre.copyTo(KF.errorCovPost);

	return predictPt;
	}
Point missile_control::kalmanCorrect(float x , float y)
	{
	measurement(0) = x;
	measurement(1) = y;
	Mat estimated = KF.correct(measurement);
	Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	return statePt;
	}

int missile_control::control_launcher()
	{
	int r;
	// process detected faces

	r = pthread_mutex_lock(&mutex_face_detected);
	if(r != 0)
		{
		std::cout << "pthread_mutex_lock returned non zero." << std::endl;
		}

	if(!faces.empty())
		{

		misses_in_a_row = 0;

		std::cout << "Face found" << std::endl;

		// Step 1: find face with minimal distance to camera_center
		size_t center_face = 0;
		int face_distance = std::numeric_limits<int>::max();
		int face_size = 0;
		for(size_t i = 0; i < faces.size(); ++i)
			{
			faces[i].x = faces[i].x + corner_cropped_image.x;
			faces[i].y = faces[i].y + corner_cropped_image.y;
			int x = faces[i].x + faces[i].size().width;
			int y = faces[i].y + faces[i].size().height;
			int temp = (camera_center.x - x) * (camera_center.x - x) + (camera_center.y - y) * (camera_center.y - y);
			int size_rect = faces[i].size().width;


			if(no_face_found < 0)
				{
				int width_diff = abs(faces[i].size().width - width_last_frame);
				int center_distance = abs (faces[i].x - center_last_frame.x)  + abs (faces[i].y - center_last_frame.y);
				int face_distance_last_frame = width_diff + center_distance;
				if(face_distance_last_frame < face_distance)
					{
					face_distance = face_distance_last_frame;
					center_face = i;
					face_distance_from_camera_center = temp;
					}
				}
			else
				{
				if (size_rect > face_size)
					{
					face_size = size_rect;
					center_face = i;
					face_distance_from_camera_center = temp;

					}
				}

			}
		// target rect (aim needs to be more accurate on distant targets)
		center_face_center.x = faces[center_face].x + faces[center_face].size().width / 2;
		center_face_center.y = faces[center_face].y + faces[center_face].size().height / 2;
		target.x = camera_center.x ;
		target.y = camera_center.y ;
		target.width = faces[center_face].size().width/2;
		target.height= faces[center_face].size().height/2 ;

		center_last_frame.x = faces[center_face].x;
		center_last_frame.y = faces[center_face].y;
		width_last_frame = faces[center_face].size().width;

		kalmanCorrect(center_face_center.x , center_face_center.y);

		r = pthread_mutex_lock(&mutex_vector_rectangles_to_draw);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		rectangles_to_draw[0] = faces[center_face]; //detected face
		rectangles_to_draw[1] = target; //target area
		rectangles_to_draw[3] = cv::Rect(faces[center_face].x  +faces[center_face].size().width / 2, faces[center_face].y +faces[center_face].size().width / 2, 10 ,10);


		r = pthread_mutex_unlock(&mutex_vector_rectangles_to_draw);
		if(r != 0)
			{
			std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
			}

		if(target.contains(cv::Point(center_face_center.x, center_face_center.y)))
			{


			++hits_in_a_row;
			}
		no_face_found = 0;
		}
	else
		{
		no_face_found++;
		if (no_face_found > 40)
			{
			kalmanInit(camera_center.x  , camera_center.y );

			servo->setSpeed(50,50);
			servo->resetPosition();
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
		center_face_center = cv::Point(camera_center.x , camera_center.y);
		face_distance_from_camera_center = std::numeric_limits<int>::max();






		}

	center_face_center= kalmanPredict();
	cv::Rect kalmanRect = cv::Rect(center_face_center.x   , center_face_center.y  ,
	                               10, 10);
	center_last_frame = cv::Point(0,0);
	width_last_frame = 0;
	r = pthread_mutex_lock(&mutex_vector_rectangles_to_draw);
	if(r != 0)
		{
		std::cout << "pthread_mutex_lock returned non zero." << std::endl;
		}
	rectangles_to_draw[2] = kalmanRect;

	r = pthread_mutex_unlock(&mutex_vector_rectangles_to_draw);
	if(r != 0)
		{
		std::cout << "pthread_mutex_lock returned non zero." << std::endl;
		}
	r = pthread_mutex_unlock(&mutex_face_detected);
	if(r != 0)
		{
		std::cout << "pthread_mutex_lock returned non zero." << std::endl;
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
	int r;
	std::cout << "Parking launcher." << std::endl;
	servo->resetPosition();
	/*
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
	}
	led_is_on = false;*/

	std::cout << "Launcher in parking position." << std::endl;



	return(EXIT_SUCCESS);
	}
