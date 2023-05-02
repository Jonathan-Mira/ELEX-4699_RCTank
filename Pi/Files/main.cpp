/////////////////////////////////////////////////
//Server for network controlled tank
//
/////////////////////////////////////////////////

#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include <string>
#include <iostream>
#include "server.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "pigpio.h"
#include <chrono>

#define camPort 4620	
#define commPort 4618
#define frameSkip 0
#define FPS 30

#define Wheel_Stop 1460

#define R_Wheel 3
#define R_forward 1370
#define R_reverse 1550
#define R_left 	  1370
#define R_right   1460

#define L_Wheel 4
#define L_forward 1550
#define L_reverse 1370
#define L_left 	  1460
#define L_right   1550

#define Turrent_servo 21
#define T_p1 1500
#define T_p2 800

#define Turret_POS_servo 20
#define Tpos_Straight 900
#define Tpos_Left 1300
#define Tpos_Right 500
#define Tpos_Step 50
#define Tpos_Sleep 50

#define Text_Color 0xffffff

std::mutex Cam_Mutex;
bool Quit;
std::string gstreamer_pipeline(int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " libcamerasrc ! video/x-raw, "
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! appsink";}

void Th_Camera(CServer* server)
{
	server->start(camPort);
}

void Th_CamFeed(CServer* server, char* cmd)
{
	bool target_seen[5] = {false, false, false, false, false};
	char Cbuff;
		cv::Mat _camera;
		cv::Mat _cambuff;
			 //pipeline parameters
    int capture_width = 640; //1280 ;
    int capture_height = 480; //720 ;
    int framerate = 15 ;
    int display_width = 640; //1280 ;
    int display_height = 480; //720 ;
    
	float w = 0.066; // Width of QR code in meters
	float f = 22; // Focal length of camera in pixels
	float d; // Distance from camera to QR code
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	
	
    //reset frame average
    std::string pipeline = gstreamer_pipeline(capture_width, capture_height, framerate, display_width, display_height);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n\n\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
	}
    cvui::init("Camera");
    cv::waitKey(10);
	std::vector<int> ids;
	int index;
	while(cap.isOpened() && !Quit)
		{
			index = 0;
			while ( index < (frameSkip +1))
			{
				if (!cap.read(_camera)) 
					{
					std::cout<<"Capture read error"<<std::endl;
					break;
					}
	    
				index++;
			}
			///////////////////////////////////////

   
  
     
     
         if (_camera.empty() == false)
         {
            
            std::vector<std::vector<cv::Point2f> > corners;
            cv::aruco::detectMarkers(_camera, dictionary, corners, ids);

            for (size_t i = 0; i < corners.size(); i++)
            {
               std::vector<cv::Point2f> marker_corners = corners[i];
               if (marker_corners.size() == 4) // Only consider markers with 4 corners
               {
                  // Compute area using the Shoelace Formula
                  float area = 0.5 * std::abs((marker_corners[1].x - marker_corners[0].x) * (marker_corners[2].y - marker_corners[0].y)
                     - (marker_corners[2].x - marker_corners[0].x) * (marker_corners[1].y - marker_corners[0].y));
                  d = (w * f) / (2 * area);
                  std::cout << "Marker " << i << " area: " << area << " distance: "<< d<< std::endl;
               }
            }
            if (ids.size() > 0)
            {
               cv::aruco::drawDetectedMarkers(_camera, corners, ids);
               
               	///////////////////////////////////////
				std::cout << ids[0]<< std::endl;
				////////
              
               
            }
         }
		cv::transpose(_camera,_cambuff);
		cv::transpose(_cambuff,_camera);		
		//cv::transpose(_camera,_camera);
		//cv::transpose(_camera,_camera);
		
		//cv::flip(_camera,_camera,1);
		
		if (ids.size() > 0)
		{
			index = 0;
			while (index < ids.size())
			{
				cvui::text(_camera, 10, 10, std::to_string(ids[0]), 1, Text_Color);
				switch(ids[0])
				{
					case 14: target_seen[4] = true; break;
					case 21: target_seen[0] = true; break;
					case 22: target_seen[1] = true; break;
					case 27: target_seen[2] = true; break;
					case 23: target_seen[3] = true; break;
					default: break;
				}
				index++;
			}
		}
		/*
		if (target_seen[4])	
			cvui::text(_camera, 10, 35,"target: Test; ID 14", 1, Text_Color);
	
		if (target_seen[0] )
		{
			cvui::text(_camera, 10, 60,"target: 1; ID 21", 1, Text_Color);
			
		}

		if (target_seen[1])
		{
			cvui::text(_camera, 10, 85,"target: 2; ID 22", 1, Text_Color);
			
		}
		if (target_seen[2])
		{
			cvui::text(_camera, 10, 110,"target: 3; ID 27", 1, Text_Color);
			
		}
		
		if (target_seen[3] )
		{
			cvui::text(_camera, 10, 135,"target: 4; ID 23", 1, Text_Color);
			
		}
		Cbuff = *cmd;
		if (Cbuff == 'E')
			std::fill(target_seen, target_seen +5, false);
		*/	
		
		server->set_txim(_camera);
		
		cv::imshow("Camera", _camera);
			cv::waitKey(1);
	}

}

void Th_Command(CServer* server)
{
	server->start(commPort);
}

void Th_Command_feed(CServer* server)
{
	auto endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(30);
	
		std::this_thread::sleep_until(endtime);
}

int main(void)
{
	Quit = false;

	if  (gpioInitialise() <0){}
	
	gpioSetMode(R_Wheel, PI_OUTPUT);
	gpioSetMode(L_Wheel, PI_OUTPUT);
	gpioSetMode(Turrent_servo, PI_OUTPUT);
	gpioSetMode(Turret_POS_servo, PI_OUTPUT);
	char Auto_cmd;
	 
	CServer S_Camera; 
	std::thread t(&Th_Camera, &S_Camera);
	std::thread tc(&Th_CamFeed, &S_Camera, &Auto_cmd);
	t.detach();		
	tc.detach();
	
	CServer S_Commands;
	std::thread q(&Th_Command, &S_Commands);
	q.detach();

	//std::thread qc(&Th_Command_feed, &S_Commands);
	//qc.detach();
	double _freq = cv::getTickFrequency();
	double elapsed_time = cv::getTickCount() / _freq;
	
	bool reset_Turret = false;
	
	int index;
	std::vector<std::string> cmds;

	int T_Position = Tpos_Straight;
	
	// Controls
	while (!Quit)
	{
		//endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(30);
		S_Commands.get_cmd(cmds);
		
		
		if(cmds.size() > 0)
		{
				std::cout<< cmds[0];
				if (cmds[0] == "W")
				{
					
					gpioServo(R_Wheel, R_forward);
					gpioServo(L_Wheel, L_forward);	
				}
				if (cmds[0] == "B")
				{
					gpioServo(R_Wheel, R_reverse);
					gpioServo(L_Wheel, L_reverse);	
				}
				if (cmds[0] == "L")
				{
					gpioServo(R_Wheel, R_left);
					gpioServo(L_Wheel, L_left);	
				}
				if (cmds[0] == "R")
				{
					gpioServo(R_Wheel, R_right);
					gpioServo(L_Wheel, L_right);	
				}
				if (cmds[0] == "Q")
				{
					gpioServo(Turrent_servo, T_p1);
					Auto_cmd = 'E';
				}else
					Auto_cmd = 'Q';
				if (cmds[0] == "E")
				{
					gpioServo(Turrent_servo, T_p2);
					elapsed_time = cv::getTickCount() / _freq;
					reset_Turret = true;
				}
				
				if(cmds[0] == " ")
				{
					gpioServo(R_Wheel, Wheel_Stop);
					gpioServo(L_Wheel, Wheel_Stop);
				}
				if(cmds[0] == "I")
					T_Position = Tpos_Straight;
				if(cmds[0] == "U")
					T_Position = Tpos_Left;
				if(cmds[0] == "O")
					T_Position = Tpos_Right;
				if(cmds[0] == "J")
				{
					if (T_Position >= Tpos_Left)
						T_Position = Tpos_Left;
					else
						T_Position = T_Position + Tpos_Step;
				}
					
				if(cmds[0] == "V")
				{
					if (T_Position <= Tpos_Right)
						T_Position = Tpos_Right;
					else
						T_Position = T_Position - Tpos_Step;
				}
		}
		
		if (reset_Turret && (cv::getTickCount() / _freq) - elapsed_time > 0.5)
		{
			gpioServo(Turrent_servo, T_p1);
			reset_Turret = false;
		}
		
		gpioServo(Turret_POS_servo, T_Position);
	}		
			
}
	

