#ifndef _MISSILE_CONTROL_HPP_
#define _MISSILE_CONTROL_HPP_

#include <thread>
#include <libusb-1.0/libusb.h>
#include <opencv.hpp>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>




//Kalman Filter
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"


// servo device which is in a subfolder for further reference
#include "servoDevice.hpp"


using namespace cv;
using namespace std;

class missile_control {
	public:

		//constructor(camera_number)
		missile_control(int);

		//destructor
		~missile_control();

		int defend();
	private:
		int frame_counter, frame_analyzed_counter, frame_analyzed, camera_number, camera_width, camera_height;
		cv::Point camera_center;
		char last_command = 'x';
		unsigned short misses_in_a_row, hits_in_a_row;

		int initialize_camera();
		int initialize_detector();

		static void* grab_and_display_video_helper(void *);
		void* grab_and_display_video();

		static void* detect_faces_helper(void *);
		void* detect_faces();

		static void* control_launcher_helper(void *);
		void* control_launcher_method();

		//OpenCV stuff
		cv::VideoCapture cap;
		cv::Mat frame;
		cv::Mat frame_analyze;
		std::vector<cv::Rect> faces;
		std::vector<cv::Rect> rectangles_to_draw;
		cv::CascadeClassifier det;

		//face detection parameters
		int minimumNeighboursFaceDetection;
		float scaleFactorFaceDetection;

        //openCV Path goes here
		std::string frontal_face_xml = "/home/dennis/opencv-2.4.9/data/haarcascades/haarcascade_frontalface_default.xml";

		//pthread
		pthread_t camera_thread, detect_thread, control_launcher_thread;
		pthread_mutex_t mutex_mat_frame, mutex_vector_rectangles_to_draw, mutex_face_detected;

		//Launcher control
		int find_and_predict_best_face();
		int move_launcher(char, int);
		int park_launcher();
		int send_command(char);
		int initialize_launcher();
		int destroy_launcher();

		//libusb
		libusb_context *ctx = NULL;
		libusb_device_handle *dev_handle;
		bool kernel_driver_active = false;

		//init Kalman
		cv::KalmanFilter KF;
		cv::Mat_<float> measurement;

		//kalmanfilter and further control mechanisms
		void kalmanInit(float x, float y);
		Point kalmanPredict();
		Point kalmanCorrect(float x, float y);
		cv::Rect target;
		cv::Point center_face_center;
		int no_face_found;
		int width_last_frame;
		cv::Point center_last_frame;
		int face_distance_from_camera_center;
		cv::Point corner_cropped_image;

		//servo
		servoDevice *servo;
		void moveServo(int faceX, int faceY);

};

#endif	//_MISSILE_CONTROL_HPP_
