////////////////////////////////////////////////////////////////
// ELEX 4618 Client Template project for BCIT
// Created Oct 5, 2016 by Craig Hennessey
// Last updated April 2022
////////////////////////////////////////////////////////////////
#include "stdafx.h"

#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include <string>
#include <iostream>
#include <regex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include "Client.h"
#include <vector>


#include "Servo_Values.h"
#include "Auto_Control.h"

std::string server_ip = "192.168.0.106";
#define Cam_Port  4620
#define Comm_Port 4618

std::string server_ip_craig = "192.168.0.100";
#define Cam_Port_craig  4013

float timeout_start;
bool Quit;

std::string Track_Server = "192.168.0.100";
#define Track_Port 5013

#define Text_Color 0xffffff


void print_menu()
{
	std::cout << "\n***********************************";
	std::cout << "\n* ELEX4618 Client Project";
	std::cout << "\n***********************************";
	std::cout << "\n(1) Send image command";
	std::cout << "\n(2) Send other command";
	std::cout << "\n(0) Exit";
	std::cout << "\nCMD> ";
}

void craig_Cam(CClient* client)
{
	auto endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(100 / 30);;
	cv::Mat im;
	while (!Quit)
	{
		endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(33);
		client->tx_str("G 1");
		Sleep(100);
		if (client->rx_im(im) == true)
		{
			
			timeout_start = cv::getTickCount();
			if (im.empty() == false)
			{
				//std::cout << "\nClient Rx: Image received";

				cv::imshow("Criag", im);
				cv::waitKey(5);
			}
		}
		std::this_thread::sleep_until(endtime);
	}
}

void th_CamFeed(CClient* CamStream, char* _Auto_cmd)
{	
	auto endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(100 / 30);;
	cv::Mat im;

	//QR detection
	float w = 0.066; // Width of QR code in meters
	float f = 22; // Focal length of camera in pixels
	float d; // Distance from camera to QR code
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	std::vector<int> ids;

	//Auto control
	int QR_ID[7] = { Target_1_ID, Target_2_ID, Target_3_ID, Target_4_ID, QR_1_ID, QR_2_ID, QR_3_ID };
	int QR_Distance[7] = { -1, -1 ,-1, -1, -1, -1, -1 };
	bool QR_Seen[7] = { false, false, false, false, false, false, false };
	bool QR_Hit[4] = { false, false, false, false };
	cv::Point QR_Position[7] = { cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0) };
	cv::Point Aim = cv::Point(213, 240);
	int Aim_Rad = 50;
	int state = 0;


	//Communicate with areana
	std::string sbuff;
	std::string Arean_Targets = "";
	CClient Target_Server;
	Target_Server.connect_socket(Track_Server, Track_Port);
	//Process areana message
	std::regex target1_not_hit_search("D1=\"0\"");
	std::regex target2_not_hit_search("D2=\"0\"");
	std::regex target3_not_hit_search("D3=\"0\"");
	std::regex target4_not_hit_search("D4=\"0\"");
	std::smatch target_not_hit;


	int index;

	double _freq = cv::getTickFrequency();
	double elapse_time = cv::getTickCount() / _freq;

	while (!Quit)
	{
		endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(33);
		CamStream->tx_str("im");
		//Sleep(200);
		if (CamStream->rx_im(im) == true)
		{
			timeout_start = cv::getTickCount();
			if (im.empty() == false)
			{


				
					//set up for QR detection 
					std::vector<std::vector<cv::Point2f> > corners;
					cv::aruco::detectMarkers(im, dictionary, corners, ids);

					//reset QR arrays to default values			
					std::fill(QR_Distance, QR_Distance + 7, 100);
					//QR_Distance[7] = {-1, -1 ,-1, -1, -1, -1, -1};
					std::fill(QR_Seen, QR_Seen + 7, false);
					//QR_Seen [7] = {false, false, false, false, false, false, false};
					std::fill(QR_Hit, QR_Hit + 4, false);
					//QR_Hit[4]	= {false, false, false, false};
					std::fill(QR_Position, QR_Position + 7, cv::Point(600, 0));
					//QR_Position [7] = { cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0)};

					for (size_t i = 0; i < corners.size(); i++)
					{
						std::vector<cv::Point2f> marker_corners = corners[i];
						if (marker_corners.size() == 4) // Only consider markers with 4 corners
						{
							// Compute area using the Shoelace Formula
							float area = 0.5 * std::abs((marker_corners[1].x - marker_corners[0].x) * (marker_corners[2].y - marker_corners[0].y) - (marker_corners[2].x - marker_corners[0].x) * (marker_corners[1].y - marker_corners[0].y));
							d = ((w * f) / (2 * area)) * 100000;
							std::cout << "Marker " << i << " area: " << area << " distance: " << d << std::endl;
						}

						//detecting specific QR codes and storing the values
						switch (ids[i])
						{
						case Target_1_ID:
							QR_Seen[Target_1] = true;
							QR_Distance[Target_1] = d;
							QR_Position[Target_1] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[Target_1], 5, cv::Scalar(255, 0, 255));
							break;

						case Target_2_ID:
							QR_Seen[Target_2] = true;
							QR_Distance[Target_2] = d;
							QR_Position[Target_2] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[Target_2], 5, cv::Scalar(255, 0, 255));
							break;

						case Target_3_ID:
							QR_Seen[Target_3] = true;
							QR_Distance[Target_3] = d;
							QR_Position[Target_3] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[Target_3], 5, cv::Scalar(255, 0, 255));
							break;

						case Target_4_ID:
							QR_Seen[Target_4] = true;
							QR_Distance[Target_4] = d;
							QR_Position[Target_4] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[Target_4], 5, cv::Scalar(255, 0, 255));
							break;

						case QR_1_ID:

							QR_Seen[QR_1] = true;
							QR_Distance[QR_1] = d;
							QR_Position[QR_1] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[QR_1], 5, cv::Scalar(255, 255, 255));
							break;


						case QR_2_ID:
							QR_Seen[QR_2] = true;
							QR_Distance[QR_2] = d;
							QR_Position[QR_2] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[QR_2], 5, cv::Scalar(255, 255, 255));
							break;

						case QR_3_ID:
							QR_Seen[QR_3] = true;
							QR_Distance[QR_3] = d;
							QR_Position[QR_3] = cv::Point((marker_corners[0].x + marker_corners[1].x + marker_corners[2].x + marker_corners[3].x) / 4, (marker_corners[0].y + marker_corners[1].y + marker_corners[2].y + marker_corners[3].y) / 4);
							cv::circle(im, QR_Position[Target_3], 5, cv::Scalar(255, 255, 255));
							break;

						default: break;
						}

					}

				



				//"Cross hair"
				cv::circle(im, Aim, Aim_Rad, cv::Scalar(255, 0, 0));

				//Get string from arena server
				Target_Server.tx_str("G 0");
				cv::waitKey(50);
				Target_Server.rx_str(Arean_Targets);
				//cvui::text(_camera, 10, 35, Arean_Targets, 0.7, Text_Color);

				//process arena string //////////////////////////////////////////////////////////////////////////////
				std::regex_search(Arean_Targets, target_not_hit, target1_not_hit_search);
				if (target_not_hit.size() == 0)
				{
					QR_Hit[Target_1] = true;
					cvui::text(im, 10, 10, "Target 1: Hit", 0.5, Text_Color);
				}
				else
					QR_Hit[Target_1] = false;

				std::regex_search(Arean_Targets, target_not_hit, target2_not_hit_search);
				if (target_not_hit.size() == 0)
				{
					QR_Hit[Target_2] = true;
					cvui::text(im, 10, 30, "Target 2: Hit", 0.5, Text_Color);
				}
				else
					QR_Hit[Target_2] = false;

				std::regex_search(Arean_Targets, target_not_hit, target3_not_hit_search);
				if (target_not_hit.size() == 0)
				{
					QR_Hit[Target_3] = true;
					cvui::text(im, 10, 50, "Target 3: Hit", 0.5, Text_Color);
				}
				else
					QR_Hit[Target_3] = false;

				std::regex_search(Arean_Targets, target_not_hit, target4_not_hit_search);
				if (target_not_hit.size() == 0)
				{
					QR_Hit[Target_4] = true;
					cvui::text(im, 10, 70, "Target 4: Hit", 0.5, Text_Color);
				}
				else
					QR_Hit[Target_4] = false;
				//////////////////////////////////////////////////////////////////////////////////////////////////

				//Auto State machine//////////////////////////////////////////////////////////////////////////////
				//Enable for toggling between manual and auto

				if (*_Auto_cmd == 'X' && state == 0)
				{
					state = 1;
					elapse_time = cv::getTickCount() / _freq;
				}
				else if (*_Auto_cmd == 'Z')
					state = 0;

				switch (state)
				{
					//Keep shooting untill target 1 is hit
				case 1:
					if (!QR_Hit[Target_1])
					{
						*_Auto_cmd = 'e';
					}
					else
					{
						*_Auto_cmd = 'g';
						Sleep(servo_move_wait);
						state++;
					}
					break;

					//Turn, aim, and shoot target 4 until hit, then straighten camera
				case 2:
					 if (!QR_Hit[Target_4])
					{
						*_Auto_cmd = 'e';
					} 

					else
					{
						*_Auto_cmd = 'i';
						cv::waitKey(Servor_move_wait);

						state = 21;
						elapse_time = cv::getTickCount() / _freq;
					}
					break;

					//Turn left for a delay
				case 21:
					if ((cv::getTickCount() / _freq) - elapse_time < Left_turn_time)
						*_Auto_cmd = 'a';
					else
					{
						state++;
						elapse_time = cv::getTickCount() / _freq;
					}
					break;

					//Turn left for a delay
				case 22:
					if ((cv::getTickCount() / _freq) - elapse_time < Straight_1_time)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 32;
						state++;
						//state = 3;
						elapse_time = cv::getTickCount() / _freq;
					}
					break;
					/*
					//turn left and allign QR_1 with center, then drive
				case 3:
					if (QR_Position[QR_1].x > midupbound)
						*_Auto_cmd = 'd';
					else if (QR_Position[QR_1].x < midlowbound)
						*_Auto_cmd = 'a';
					else
					{
						*_Auto_cmd = 'w';
						state++;
					}
					break;

					//Drive forward untill QR_1 is a set distance away
				case 4:
					if (QR_Distance[QR_1] > QR1_Distance_Turn)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 32;
						state++;
					}
					break;

					//Turn untill QR_2 is left allign	
				case 5:
					if (QR_Position[QR_2].x > upperbound)
						*_Auto_cmd = 'd';
					else if (QR_Position[QR_2].x < lowerbound)
						*_Auto_cmd = 'a';
					else
					{
						*_Auto_cmd = 'w';
						state++;
					}
					break;

					//Drive untill QR_2 is a set distance away, turn camera to the 90 left 
				case 6:
					if (QR_Distance[QR_2] > QR2_Distance_fst)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 'u';
						state++;
				
					}
					break;
*/

				//Turn Right for a delay
				case 23:
					if ((cv::getTickCount() / _freq) - elapse_time < Right_Turn1)
						*_Auto_cmd = 'd';
					else
					{
						state++;
						elapse_time = cv::getTickCount() / _freq;
					}
					break;

					//Drive straight to target 2
				case 24:
					if ((cv::getTickCount() / _freq) - elapse_time < Straight_2_time)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 32;
						Sleep(servo_move_wait);
						//*_Auto_cmd = 'u';
						//Sleep(servo_move_wait);
						state++;
					}
					break;

				case 25:
					*_Auto_cmd = 'u';
					state = 7;
					break;

					//Aim and shoot Target_2, then straigten camera
				case 7:
					if (!QR_Hit[Target_2])
					{
						

						if (QR_Position[Target_2].x > upperbound)
							*_Auto_cmd = 'l';
						else if (QR_Position[Target_2].x < lowerbound)
							*_Auto_cmd = 'j';
						else
							*_Auto_cmd = 'e';
					}
					else
					{

						*_Auto_cmd = 'i';
						cv::waitKey(Servor_move_wait);
						elapse_time = cv::getTickCount() / _freq;
						state++;
					}
					break;
/*
					//Drive untill QR_2 is a set distance away,
				case 8:
					if (QR_Distance[QR_2] > QR2_Distance_snd)
						*_Auto_cmd = 'w';
					else
						state++;
					break;
*/
				case 8:
					if ((cv::getTickCount() / _freq) - elapse_time < Straight_3_time)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 32;
						Sleep(servo_move_wait);
						//*_Auto_cmd = 'u';
						//Sleep(servo_move_wait);
						//state++;
						//state++;
						state++;
					}
					break;

					//Turn right untill target_3 is center allign
				case 9:
					*_Auto_cmd = 'u';
					state = 11;
					break;
					/*
					//Drive untill target_3 is a set distance away
				case 10:
					if (QR_Distance[Target_3] > Target_3_Distace_tShot)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 'e';
						state++;
					}
					break;
					*/
					//Aim and shoot target_3
				case 11:
					if (!QR_Hit[Target_3])
					{
						//cvui::text(_camera, 270, 10,std::to_string(QR_Position[Target_4].x), 1, Text_Color);

						if (QR_Position[Target_3].x > upperbound)
							*_Auto_cmd = 'l';
						else if (QR_Position[Target_3].x < lowerbound)
							*_Auto_cmd = 'j';
						else
							*_Auto_cmd = 'e';
					}
					else
					{
						*_Auto_cmd = 'i';
						cv::waitKey(Servor_move_wait);
						state++;
					}
					break;
					
				case 12:
					if ((cv::getTickCount() / _freq) - elapse_time < Straight_4_time)
						*_Auto_cmd = 'w';
					else
					{
						*_Auto_cmd = 32;
						Sleep(servo_move_wait);
						//*_Auto_cmd = 'u';
						//Sleep(servo_move_wait);
						state++;
					}
					break;

				case 13:
					if ((cv::getTickCount() / _freq) - elapse_time < Turn_3_time)
						*_Auto_cmd = 'd';
					else
					{
						*_Auto_cmd = 'w';
						//*_Auto_cmd = 'u';
						//Sleep(servo_move_wait);
						state++;
					}
					break;

				


				default: break;
				}
				//cvui::text(im, 270, 10, std::to_string(QR_Position[Target_4].x), 1, Text_Color);
				cvui::text(im, 580, 10, std::to_string(state), 1, Text_Color);
				////////////////////////////////////////////////////////////////////////////////////////////////////////////


				cv::imshow("rx", im);
				cv::waitKey(1);
			}
		}
		std::this_thread::sleep_until(endtime);
	}
}

int main(int argc, char* argv[])
{
	char _Auto_cmd;
	CClient Camclient;
	int cmd = -1;
	timeout_start = cv::getTickCount();
	Camclient.connect_socket(server_ip, Cam_Port);
	std::thread t(&th_CamFeed, &Camclient, &_Auto_cmd);
	t.detach();

	CClient Commclient;
	timeout_start = cv::getTickCount();
	Commclient.connect_socket(server_ip, Comm_Port);

	//std::thread t(&th_CamFeed, &Commclient);
	//t.detach();

	CClient Craig_Cam;
	timeout_start = cv::getTickCount();
	Craig_Cam.connect_socket(server_ip_craig, Cam_Port_craig);
	std::thread g(&craig_Cam, &Craig_Cam);
	g.detach();


	cvui::init("Controls");
	cv::Mat Control_Window;
	Control_Window.create(200, 200, CV_8UC3);
	
	bool Auto = false;
	std::string sbuff;
	char cbuff;

	do
	{
		Sleep(50);

		cbuff = cv::waitKey(5);
		switch (cbuff)
		{
		case 'x':
			Auto = true; _Auto_cmd = 'X'; break;
		case 'z':
			Auto = false; _Auto_cmd = 'Z';  cbuff = 32; break;
		default:
			break;
		}

		if (Auto)
			cbuff = _Auto_cmd;

		switch (cbuff)
		{
		case 'w':
			Commclient.tx_str("W"); break;
		case 'a':
			Commclient.tx_str("A"); break;
		case 'd':
			Commclient.tx_str("D"); break;
		case 's':
			Commclient.tx_str("S"); break;
		case 'q':
			Commclient.tx_str("Q"); break;
		case 'e':
			Commclient.tx_str("E"); break;
		case ' ':
            Commclient.tx_str(" "); break;
		case 'u':
			Commclient.tx_str("U"); break;
		case 'i':
			Commclient.tx_str("I"); break;
		case 'o':
			Commclient.tx_str("O"); break;
		case 'j':
			Commclient.tx_str("J"); break;
		case 'l':
			Commclient.tx_str("L"); break;
		case 'k':
			Commclient.tx_str("E"); break;
		case 'y':
			Commclient.tx_str("Y"); break;
		case 'h':
			Commclient.tx_str("H"); break;
		case '/':
			Commclient.tx_str("/"); break;
		case 'm':
			Commclient.tx_str("M"); break;
		case 'n':
			Commclient.tx_str("N"); break;
		case 'g':
			Commclient.tx_str("G"); break;
		default: break;
		}
		//Commclient.tx_str("im");
		//Commclient.rx_str(sbuff);
		//std::cout << sbuff << std::endl;

		cv::imshow("Controls", Control_Window);
		cv::waitKey(1);
		
	} while (!Quit);

}
