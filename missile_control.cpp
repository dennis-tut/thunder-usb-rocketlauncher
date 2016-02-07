#include "missile_control.hpp"


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


    measurement = cv::Mat_<float> (2,1);
	// make two rectangles fit into array (target rectangle and detected face rectangle)
	rectangles_to_draw.resize(2);
	// number of frames no face was found
	no_face_found = 0;
	misses_in_a_row = 0;
	// number of consecutive frames a face was detected
	hits_in_a_row = 0;
	//set constants for face detection
	scaleFactorFaceDetection = 1.04;
    minimumNeighboursFaceDetection = 15;
	}

missile_control::~missile_control() {
	int r;
	std::cout << "Destructor called." << std::endl;

	delete servo;
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

int missile_control::defend()
	{
	//This function starts three threads.
	//Thread two will analyze captured frames and control the missile launcher

	int r;

	// grabs frames from camera and will display live video + detected rectangles
	r = pthread_create(&camera_thread, nullptr, &grab_and_display_video_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	// takes pictures, evaluates them and searches faces
	r = pthread_create(&detect_thread, nullptr, &detect_faces_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	// is responsible for controlling the servos and launching the missiles
	r = pthread_create(&control_launcher_thread, nullptr, &control_launcher_helper, this);
	if(r != 0)
		{
		std::cout << "pthread_create returned non zero." << std::endl;
		}

	//wait for all threads to finish
	r = pthread_join(camera_thread, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_join returned non zero." << std::endl;
		}

	r = pthread_join(detect_thread, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_join returned non zero." << std::endl;
		}

	r = pthread_join(control_launcher_thread, nullptr);
	if(r != 0)
		{
		std::cout << "pthread_join returned non zero." << std::endl;
		}
	return(EXIT_SUCCESS);

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

// initializes face detector
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

// initializes launcher
// important thing: If you change to another rocket launcher model change the ID here
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

	// change rocket launcher ID here
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


// clean up duty for the launcher
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



// many good examples can be found online
// initialize Kalman filter with predefined values
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

	// this changes the acceleration in x and y value
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,2,0,   0,1,0,0.5,  0,0,1,0,  0,0,0,1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(.005)); //adjust this for faster convergence - but higher noise
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	}

// predict next postion of the spectated object and return the prediction point
Point missile_control::kalmanPredict()
	{
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

	KF.statePre.copyTo(KF.statePost);
	KF.errorCovPre.copyTo(KF.errorCovPost);

	return predictPt;
	}

// input correct measured object point
Point missile_control::kalmanCorrect(float x , float y)
	{
	measurement(0) = x;
	measurement(1) = y;
	Mat estimated = KF.correct(measurement);
	Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	return statePt;
	}


