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
#include <thread>

#include "Client.h"

std::string server_ip = "192.168.137.133";
#define Cam_Port  4620
#define Comm_Port 4618

float timeout_start;
bool Quit;

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

void send_command(CClient &client, std::string cmd)
{
	std::string str;

	client.tx_str(cmd);
	std::cout << "\nClient Tx: " << cmd;

	if (cmd == "im")
	{

	}
	else
	{
		if (client.rx_str(str) == true)
		{
			timeout_start = cv::getTickCount();
			std::cout << "\nClient Rx: " << str;
		}
		else
		{
			if ((cv::getTickCount() - timeout_start) / cv::getTickFrequency() > 1000)
			{
				// No response, disconnect and reconnect
				timeout_start = cv::getTickCount();
				client.close_socket();
				client.connect_socket(server_ip, Cam_Port);
			}
		}
	}
}

void th_CamFeed(CClient* CamStream)
{	
	auto endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(100 / 30);;
	cv::Mat im;
	while (!Quit)
	{
		endtime = std::chrono::system_clock::now() + std::chrono::milliseconds(33);
		CamStream->tx_str("im");
		if (CamStream->rx_im(im) == true)
		{
			timeout_start = cv::getTickCount();
			if (im.empty() == false)
			{
				//std::cout << "\nClient Rx: Image received";

				cv::imshow("rx", im);
				cv::waitKey(1);
			}
		}
		std::this_thread::sleep_until(endtime);
	}
}

int main(int argc, char* argv[])
{
	CClient Camclient;
	int cmd = -1;
	timeout_start = cv::getTickCount();
	Camclient.connect_socket(server_ip, Cam_Port);
	std::thread t(&th_CamFeed, &Camclient);
	t.detach();

	CClient Commclient;
	timeout_start = cv::getTickCount();
	Commclient.connect_socket(server_ip, Comm_Port);

	//std::thread t(&th_CamFeed, &Commclient);
	//t.detach();

	cvui::init("Controls");
	cv::Mat Control_Window;
	Control_Window.create(400, 500, CV_8UC3);
	
	bool menubuff;
	std::string sbuff;
	char cbuff;

	do
	{
		Sleep(50);
		//cv::rectangle(Control_Window, cv::Point(0, 0), cv::Point(500, 400), cv::Scalar(0, 0, 0), CVUI_FILLED, cv::LINE_AA, 0);	
		menubuff = cvui::button(Control_Window, 150, 10, 120, 50, "Forward");
		if (menubuff)
		{
			Commclient.tx_str("W");
			std::cout << "W" << std::endl;
		}


		menubuff = cvui::button(Control_Window, 80, 10, 50, 120, "<-");
		if (menubuff)
		{
			Commclient.tx_str("L");
			std::cout << "L" << std::endl;
		}

		menubuff = cvui::button(Control_Window, 290, 10, 50, 120, "->");
		if (menubuff)
		{
			Commclient.tx_str("R");
			std::cout << "R" << std::endl;
		}

		cbuff = cv::waitKey(1);
		switch (cbuff)
		{
		case 'w':
			Commclient.tx_str("W"); break;
		case 'a':
			Commclient.tx_str("L"); break;
		case 'd':
			Commclient.tx_str("R"); break;
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
			Commclient.tx_str("V"); break;
		case 'k':
			Commclient.tx_str("E"); break;
		case 's':
			Commclient.tx_str("B"); break;

		default:
			break;
		}
		//Commclient.tx_str("im");
		//Commclient.rx_str(sbuff);
		//std::cout << sbuff << std::endl;

		cv::imshow("Controls", Control_Window);
		cv::waitKey(1);
		
	} while (!Quit);

}
