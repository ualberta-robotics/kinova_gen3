/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

/* 
* Author: Laura Petrich
* Date: July 19, 2019
*/

#include "spnavkinova.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <ctime>
#include <cstddef>         // std::size_t
#include <thread>         // std::thread

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>

using namespace std;

namespace k_api = Kinova::Api;


#define IP_ADDRESS "192.168.1.12"
#define PORT 10000
#define MODE_TRANS 0 // Translation Mode
#define MODE_WRIST 1 // Wrist Orientation Mode
#define MODE_FINGER 2 // Finger Mode

array<double, 6> old_motion;
double old_grasp;
int mode;
int switch_count;
bool quit;
ofstream outfile;

void sig(int s)
{
	spnav_close();
	exit(0);
}

void setup_file(char *task, char* participant)
{
	string fname = "../sessions/";
	fname.append(task);
	fname.append("3D");
	fname.append(participant);
    std::time_t t = std::time(nullptr);
	char mbstr[100];
    if (std::strftime(mbstr, sizeof(mbstr), "%y%m%e%H%M", std::localtime(&t))) {
	    fname.append(mbstr);
    }
	outfile.open(fname);
	std::size_t found = fname.find_last_of("/\\");
	char timestr[100];
    outfile << "*************************************** INFO ***************************************\n";
    outfile << "Filename: " << fname.substr(found + 1) << "\n";
    if (std::strftime(timestr, sizeof(timestr), "%c", std::localtime(&t))) {
    outfile << "Date: " << timestr << "\n";
    }
    outfile << "Task: " << task << "\n";
    outfile << "Participant: " << participant << "\n";
    outfile << "************************************** FORMAT **************************************\n";
    outfile << "mode\n";
    outfile << "motion force command [linear_x linear_y linear_z angular_x angular_y angular_z]\n";
    outfile << "gripper force command [open/close]\n";
    outfile << "current pose [x y z theta_x theta_y theta_z]\n";
    outfile << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
    outfile << "************************************************************************************\n";
    cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
}

void write_to_file(k_api::Base::BaseClient* pBase) 
{
	// pBase->GetMeasuredJointSpeeds(); does not work
	k_api::Base::Pose pose;
	k_api::Base::Twist twist;
	k_api::Base::JointAngles ja;
	while (!quit) {
		// write mode
		outfile << mode << "\n";
		// write motion force command
		for (int i = 0; i < old_motion.size(); ++i) {
			outfile << old_motion.at(i) << " ";
		}
		outfile << "\n";
		// write gripper force command
		outfile << old_grasp << "\n";
		// grab and write current pose 
		pose = pBase->GetMeasuredCartesianPose();
		outfile << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
		// grab and write current joint angles
		ja = pBase->GetMeasuredJointAngles();
		for (int i = 0; i < ja.joint_angles_size(); ++i) {
			outfile << ja.joint_angles(i).value() << " ";
		}
		outfile << "\n";
		// grab and write current joint speeds
		twist = pBase->GetMeasuredTwist();
		outfile << twist.linear_x() << " " << twist.linear_y() << " " << twist.linear_z() << " " << twist.angular_x() << " " << twist.angular_y() << " " << twist.angular_z() << "\n"; 

		// cout << "\n";
		// cout << ja.joint_angles_size() << endl;
		// separate by a new line
		outfile << "\n";
	}
}
void write_to_screen(k_api::Base::BaseClient* pBase)
{
	while (!quit) {
		k_api::Base::Pose pose = pBase->GetMeasuredCartesianPose();
		cout << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
	}
}

void print_mode() 
{
    switch(mode) {
        case MODE_TRANS:
            std::cout << "\nTRANSLATION MODE" << std::endl;
            break;
        case MODE_WRIST:
            std::cout << "\nWRIST ORIENTATION MODE" << std::endl;
            break;
        case MODE_FINGER:
            std::cout << "\nFINGER MODE" << std::endl;
            break;
        default:
            break;
    }
}

void send_twist_command(k_api::Base::BaseClient* pBase, const array<double, 6> &v)
{
	auto command = k_api::Base::TwistCommand();
	command.set_mode(k_api::Base::UNSPECIFIED_TWIST_MODE);
	command.set_duration(1);  // set to 0 for unlimited time to execute
	auto twist = command.mutable_twist();
    twist->set_linear_x(v.at(0));
    twist->set_linear_y(v.at(1));
    twist->set_linear_z(v.at(2));
    twist->set_angular_x(v.at(3));
    twist->set_angular_y(v.at(4));
    twist->set_angular_z(v.at(5));
	// pBase->SendTwistCommand(command);
}

void send_gripper_command(k_api::Base::BaseClient* pBase, double val)
{
	k_api::Base::GripperCommand output;
	output.set_mode(k_api::Base::GRIPPER_FORCE);
	auto gripper = output.mutable_gripper();
	gripper->clear_finger();
	auto finger = gripper->add_finger();
	finger->set_finger_identifier(1);
	finger->set_value(val);
	output.set_duration(0);    
	pBase->SendGripperCommand(output);
	cout << "gripper command: " << val << endl;
}

double map_x(double x)
{
	double sgn;
	double ret;
	double a = 0.0;
	double b = 250.0;
	double c = 0.0;
	double d = 0.6;
	// save sign for later use
	if (x > 0)  sgn = 1.0;
	else sgn = -1.0;
	// find dead zone
	if (abs(x) < 100) return 0.0;
	else x -= (sgn * 100);
	// calculate mapped return value [a, b] -> [c, d]
	ret = (abs(x) - a) * ((d - c) / (b - a)) + c;
	// another dead zone
	if (ret < 0.05) return 0.0;
	// round to 2 decimal digits with correct sign
	return floor((sgn * ret * 10) + .5) / 10;
}

bool handle_motion(k_api::Base::BaseClient* pBase, spnav_event_motion motion, array<double, 6> &v)
{   
	/*******************************************************************************
	* values are mapped wrong in spnav library
	* linear x = motion.z
	* linear y = -motion.x
	* linear z = motion.y
	* angular x = motion.rz
	* angular y = -motion.rx
	* angular z = motion.ry 
	*******************************************************************************/
	double x;
	switch (mode) {
		case MODE_TRANS:
			v = {map_x((double)motion.z), map_x((double)-motion.x), map_x((double)-motion.ry), 0, 0, 0};
			break;
		case MODE_WRIST:
			v = {0, 0, 0, map_x((double)motion.z), map_x((double)motion.x), map_x((double)-motion.ry)};
			break;
		case MODE_FINGER:
			v = {0, 0, 0, 0, 0, 0};
			x = map_x((double)motion.z);
			if (x != old_grasp) {
				send_gripper_command(pBase, x);
				old_grasp = x;
			}
			break;
		default:
			break;
	}
	if (v == old_motion) return false;
	return true;


}


void handle_button(int button) 
{
	if (button == 1) {
		quit = true;
	} else {
        mode = (mode + 1) % 3;
        print_mode();
        switch_count += 1;
	}
}

void loop(k_api::Base::BaseClient* pBase)
// void loop()
{
	// Setup SpaceMouse
	signal(SIGINT, sig);
	if (spnav_open() == -1) {
		fprintf(stderr, "failed to connect to the space navigator daemon\n");
		exit(0);
	}
	quit = false;
	// grip = 1.0;
	switch_count = 0;
	mode = 0;
	old_grasp = 0.0;
	print_mode();
	// setup_file();
	while (!quit) {
		usleep(10000);
		spnav_event sev;
		if (spnav_poll_event(&sev)) {
			if(sev.type == SPNAV_EVENT_MOTION) {
				array<double, 6> new_motion;
				if (handle_motion(pBase, sev.motion, new_motion)) {
					old_motion = new_motion;
					// send_twist_command(pBase, new_motion);
					// cout << "motion command: ";
					// for (int i = 0; i < new_motion.size(); ++i) {
					// 	cout << new_motion.at(i) << " ";
					// }
					// cout << endl;
				} 
			} else if (sev.button.press) {    /* SPNAV_EVENT_BUTTON */
				handle_button(sev.button.bnum);
			}
		}
		// write_to_file();
		// write_to_screen(pBase);
	}
	spnav_close();
	cout << "TOTAL MODE SWITCHES: " << switch_count << endl;
	outfile << "TOTAL MODE SWITCHES: " << switch_count << endl;
}

int main(int argc, char **argv)
{
	// Setup API
	auto errorCallback =  [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
	auto pTransport = new k_api::TransportClientUdp();
	auto pRouter = new k_api::RouterClient(pTransport, errorCallback);
	pTransport->connect(IP_ADDRESS, PORT);

	// Create session
	auto createSessionInfo = k_api::Session::CreateSessionInfo();
	createSessionInfo.set_username("iris");
	createSessionInfo.set_password("IRIS");
	createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
	createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

	cout << "\nCreating session for communication" << endl;
	auto pSessionMng = new k_api::SessionManager(pRouter);
	pSessionMng->CreateSession(createSessionInfo);
	cout << "Session created" << endl;

	// Create required services
	auto pBase = new k_api::Base::BaseClient(pRouter);

	// Move arm to ready position
	// cout << "Moving the arm to a safe position before executing example\n" << endl;
	// auto action_type = k_api::Base::RequestedActionType();
	// action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
	// auto action_list = pBase->ReadAllActions(action_type);
	// auto action_handle = k_api::Base::ActionHandle();
	// action_handle.set_identifier(0); 
	// for( auto action : action_list.action_list()) {
	//     if(action.name() == "Home") {
	//         action_handle = action.handle();
	//     }
	// }

	// if(action_handle.identifier() == 0) {
	//     cout << "\nCan't reach safe position. Exiting" << endl;       
	// } else {
	//     pBase->ExecuteActionFromReference(action_handle);
	//     this_thread::sleep_for(chrono::seconds(5)); // Leave time to action to finish
	// }
	if (argc <= 2) {
		cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
		cout << "Running without saving data" << endl;
		loop(pBase);
		// thread loop_thread(loop, pBase);
		// loop_thread.detach();
		// loop(pBase);
	} else {
		setup_file(argv[1], argv[2]);
		thread file_thread(write_to_file, pBase);
		if (outfile.is_open()) {
			thread loop_thread(loop, pBase);
			loop_thread.detach();
			file_thread.join();
			// loop(pBase);
		}
		outfile.close();
	}
	// Close API session
	pSessionMng->CloseSession();

	// Deactivate the router and cleanly disconnect from the transport object
	pRouter->SetActivationStatus(false);
	pTransport->disconnect();

	// Destroy the API
	delete pBase;
	delete pSessionMng;
	delete pRouter;
	delete pTransport;
};