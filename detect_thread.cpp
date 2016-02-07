#include "missile_control.hpp"
#include <opencv.hpp>
// This thread takes pictures from the camera_thread, evaluates them and searches faces

// does what it says
void* missile_control::detect_faces_helper(void *context)
	{
	return((missile_control *)context)->detect_faces();
	}

// detects faces thread main method
void* missile_control::detect_faces()
	{
	int r;
	// set states for which thread should terminate
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

	// main loop
	while(true)
		{
		//thread can be canceled at this point
		pthread_testcancel();

		r = pthread_mutex_lock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		// copy frame from the camera thread
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

		// If face was detected last frame, only search for a face nearby
		if(width_last_frame != 0)
			{
			// specify the area in which the algorithm searches for faces --> only faces near the last found face are searched for
			// face detection finds same face more stable
			// bottom left corner
			int corner_x = max(center_last_frame.x - width_last_frame/2 , 0);
			int corner_y = max(center_last_frame.y - width_last_frame/2 , 0);
			// dont search outside of the frame
			int width = min(center_last_frame.x + width_last_frame * 3/2, camera_width);
			int height = min(center_last_frame.y + width_last_frame * 3/2 , camera_height);

			// search for face only in the computated rectangle
			cropped_image = frame_analyze(cv::Rect(corner_x , corner_y , width , height));
			corner_cropped_image = cv::Point(corner_x , corner_y);
			}
		// analyze the whole picture if no face was found in the last frame
		else
			{
			cropped_image = frame_analyze;
			corner_cropped_image = cv::Point(0,0);
			}

		// equalizeHist adds more contrast, look for an example online which should make it pretty clear
		cv::equalizeHist(cropped_image, img_hist_equalized);
		// this is the face detection
		det.detectMultiScale(
		    img_hist_equalized,
		    faces,
		    scaleFactorFaceDetection,
		    minimumNeighboursFaceDetection,
		    0,
		    cv::Size(camera_width / 12, camera_height / 12),
		    cv::Size(camera_width , camera_height )
		);

		r = find_and_predict_best_face();
		if(r != EXIT_SUCCESS)
			{
			std::cout << "missile_launcher::control_launcher failed." << std::endl;
			}
		}
	}

	// this controls the movement of the servos
int missile_control::find_and_predict_best_face()
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

			// evtl raus
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

		// save values of this frame for use in the next frame
		center_last_frame.x = faces[center_face].x;
		center_last_frame.y = faces[center_face].y;
		width_last_frame = faces[center_face].size().width;

		// give the real position to Kalman filter
		kalmanCorrect(center_face_center.x , center_face_center.y);

		r = pthread_mutex_lock(&mutex_vector_rectangles_to_draw);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}

		// store rectangles for camera thread
		//rectangles_to_draw.clear();
		//rectangles_to_draw.resize(3+ faces.size());
		rectangles_to_draw[0] = faces[center_face]; //detected face
		rectangles_to_draw[1] = target; //target area
		rectangles_to_draw[3] = cv::Rect(faces[center_face].x  +faces[center_face].size().width / 2, faces[center_face].y +faces[center_face].size().width / 2, 10 ,10);
        // possibly label all other "faces"
    /*
        for(int i = 0; i < faces.size(); i++){
            if(i != center_face) {
                rectangles_to_draw[i+3] = faces[i];
            }
        }
    */

		r = pthread_mutex_unlock(&mutex_vector_rectangles_to_draw);
		if(r != 0)
			{
			std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
			}

		// increment hits_in_a_row if the face center is in the target area
		if(target.contains(cv::Point(center_face_center.x, center_face_center.y)))
			{
			++hits_in_a_row;
			}
		// face found
		no_face_found = 0;
		}
	else
		{
		no_face_found++;
		// reset Kalman and reset position of the servos if no face was found for over 40 frames
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

	// predict center of face with Kalman
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
	// save area of Kalman rectangle for camera thread
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
