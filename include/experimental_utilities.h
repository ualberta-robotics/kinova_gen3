/* 
* Author: Laura Petrich
* Date: July 25, 2019
*/


#ifndef EXPERIMENTAL_UTILITIES_H
#define EXPERIMENTAL_UTILITIES_H

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
#include <cstddef>
#include <thread>
#include <bits/stdc++.h> 


#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

using namespace std;

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.12"
#define PORT 10000

class ExperimentalUtilities {
	protected:
	bool quit;
	int axis;
	int mode;
	double gripper_value;
	array<double, 6> motion_value;
	string task;
	string participant;
	ofstream outfile;
	time_t start_time;
	time_t end_time; 

	public:
	ExperimentalUtilities(int axis_, string task_, string participant_) :
		axis(axis_),
		task(task_),
		participant(participant_),
		quit(false),
		mode(0),
		gripper_value(0.0),
		motion_value({0, 0, 0, 0, 0, 0})
	{
	 	cout << "Running " << axis << " Dimensional Controller" << endl;
	}
	~ExperimentalUtilities() 
	{
		outfile.close();
	}

	void set_mode(int i)
	{
		mode = i;
	}

	void set_quit(bool b)
	{
		quit = b;
	}

	void set_gripper_value(double d)
	{
		gripper_value = d;
	}

	void set_motion_value(array<double, 6> &a)
	{
		motion_value = a;
	}

	void write_mode_count(int count)
	{	
		cout << "TOTAL MODE SWITCHES: " << count << endl;
		outfile << "TOTAL MODE SWITCHES: " << count << endl;
	}	

	bool setup_file()
	{
		string fname = "../sessions/";
		fname.append(task);
		fname.append(participant);
		fname.append(to_string(axis) + "D_");
	    std::time_t t = std::time(nullptr);
		char mbstr[100];
	    if (std::strftime(mbstr, sizeof(mbstr), "%y%m%d%H%M", std::localtime(&t))) {
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
	    
	    cout << "*************************************** INFO ***************************************\n";
	    cout << "Filename: " << fname.substr(found + 1) << "\n";
    	cout << "Date: " << timestr << "\n";
	    cout << "Task: " << task << "\n";
	    cout << "Participant: " << participant << "\n";
	    cout << "************************************************************************************\n";
	    if (outfile.is_open()) return true;
	    return false;
	}

	void write_to_file(k_api::Base::BaseClient* pBase, bool experimental) 
	{
		// pBase->GetMeasuredJointSpeeds(); does not work
		k_api::Base::Pose pose;
		// k_api::Base::Twist twist;
		k_api::Base::JointAngles ja;
		while (!quit) {
			if (experimental) {
				// write mode
				outfile << mode << "\n";
				// write motion force command
				for (int i = 0; i < (int)motion_value.size(); ++i) {
					outfile << motion_value.at(i) << " ";
				}
				outfile << "\n";
				// write gripper force command
				outfile << gripper_value << "\n";
			}
			// grab and write current pose 
			pose = pBase->GetMeasuredCartesianPose();
			outfile << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
			// grab and write current joint angles
			ja = pBase->GetMeasuredJointAngles();
			for (int i = 0; i < ja.joint_angles_size(); ++i) {
				outfile << ja.joint_angles(i).value() << " ";
			}
			outfile << "\n\n";
		}
	}

	void start_timer()
	{
		time(&start_time);
	    ios_base::sync_with_stdio(false); 
	    cout << "\nStarting timer..." << endl;
	}

	void stop_timer()
	{
		time(&end_time); 
	    double sec_taken = double(end_time - start_time);
	    int min_taken = (int)(sec_taken / 60);
		cout << "TOTAL EXECUTION TIME: " << min_taken << " MINUTES " << sec_taken - (min_taken * 60) << " SECONDS" << endl;
		outfile << "TOTAL EXECUTION TIME: " << min_taken << " SECONDS" << endl;
	}

	private:
		// SDL_Surface* window;
};    



// void setup_file(char *task, bool debug=false)
// {
// 	debug_ = debug;
// 	string fname = "../sessions/";
// 	fname.append(task);
// 	fname.append("Ideal_");
//     std::time_t t = std::time(nullptr);
// 	char mbstr[100];
//     if (std::strftime(mbstr, sizeof(mbstr), "%y%m%e%H%M", std::localtime(&t))) {
// 	    fname.append(mbstr);
//     }
// 	outfile.open(fname);
// 	std::size_t found = fname.find_last_of("/\\");
// 	char timestr[100];
//     outfile << "*************************************** INFO ***************************************\n";
//     outfile << "Filename: " << fname.substr(found + 1) << "\n";
//     if (std::strftime(timestr, sizeof(timestr), "%c", std::localtime(&t))) {
//     outfile << "Date: " << timestr << "\n";
//     }
//     outfile << "Task: " << task << "\n";
//     outfile << "Participant: Ideal Trajectory" << "\n";
//     outfile << "************************************** FORMAT **************************************\n";
//     outfile << "current pose [x y z theta_x theta_y theta_z]\n";
//     outfile << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
//     outfile << "************************************************************************************\n";
//     // cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
//     if (debug_) {
// 	    cout << "*************************************** INFO ***************************************\n";
// 	    cout << "Filename: " << fname.substr(found + 1) << "\n";
//     	cout << "Date: " << timestr << "\n";
// 	    cout << "Task: " << task << "\n";
// 	    cout << "Participant: Ideal Trajectory" << "\n";
// 	    cout << "************************************** FORMAT **************************************\n";
// 	    cout << "current pose [x y z theta_x theta_y theta_z]\n";
// 	    cout << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
// 	    cout << "************************************************************************************\n";    	
//     } else {
// 	    cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
//     }
// }

#endif  /* EXPERIMENTAL_UTILITIES_H */
