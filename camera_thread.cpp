#include "missile_control.hpp"
#include <opencv.hpp>
// This thread grabs frames from camera and will display live video + detected rectangles

// does what it says
void* missile_control::grab_and_display_video_helper(void *context)
	{
	return((missile_control *)context)->grab_and_display_video();
	}

// does what it says
void* missile_control::grab_and_display_video()
	{
	int r;
	cv::namedWindow("Capture", cv::WINDOW_NORMAL);
	while(true)
		{
		//lock mutex while grabbing the next frame
		r = pthread_mutex_lock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_lock returned non zero." << std::endl;
			}
        // this provides the next frame
		cap >> frame;

		frame_counter++;

		pthread_mutex_unlock(&mutex_mat_frame);
		if(r != 0)
			{
			std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
			}

		// draw rectangles if mutex is free && detected rectangles are more or less up to date
		r = pthread_mutex_trylock(&mutex_vector_rectangles_to_draw);
		if(r == 0)
			{
			if(frame_counter - frame_analyzed < 3  && misses_in_a_row == 0)
				{
				cv::rectangle(frame, rectangles_to_draw[0], cv::Scalar(0, 0, 255), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[1], cv::Scalar(255, 0, 0), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[2], cv::Scalar(0, 255, 0), 1, 8, 0);
				cv::rectangle(frame, rectangles_to_draw[3], cv::Scalar(255, 255, 0), 1, 8, 0);

				// enable for all "faces"
                /*
				for(int i = 4; i<rectangles_to_draw.size(); i++) {
                    cv::rectangle(frame, rectangles_to_draw[i], cv::Scalar(0, 0, 255), 1, 8, 0);
                }
                */
				cv::displayStatusBar("Capture" ,"Toolbar" ,0);
				}
			r = pthread_mutex_unlock(&mutex_vector_rectangles_to_draw);
			if(r != 0)
				{
				std::cout << "pthread_mutex_unlock returned non zero." << std::endl;
				}
			}

        // show the captured frame
		cv::imshow("Capture", frame);
        // terminates program
		if(cv::waitKey(30) == 27)
			{
			std::cout << "ESC key pressed." << std::endl;

			r = pthread_cancel(detect_thread);
			if(r != 0)
				{
				std::cout << "pthread_cancle returned non zero." << std::endl;
				}
			break;
			}
		}
	return(nullptr);
	}
