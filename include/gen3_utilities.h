/* 
* Author: Laura Petrich
* Date: July 25, 2019
*/


#ifndef GEN3_UTILITIES_H
#define GEN3_UTILITIES_H

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

#include <BaseClientRpc.h>

using namespace std;

namespace k_api = Kinova::Api;

array<double, 6> old_motion;
double old_grasp;
ofstream outfile;
bool quit;
bool debug_;
int mode;
int switch_count;
// time_t start_time;
// time_t end_time; 

void setup_file(char *task, char* participant, string dim, bool debug=false)
{
	debug_ = debug;
	string fname = "../sessions/";
	fname.append(task);
	fname.append(participant);
	fname.append(dim);
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
    // cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
    if (debug_) {
	    cout << "*************************************** INFO ***************************************\n";
	    cout << "Filename: " << fname.substr(found + 1) << "\n";
    	cout << "Date: " << timestr << "\n";
	    cout << "Task: " << task << "\n";
	    cout << "Participant: " << participant << "\n";
	    cout << "************************************** FORMAT **************************************\n";
	    cout << "mode\n";
	    cout << "motion force command [linear_x linear_y linear_z angular_x angular_y angular_z]\n";
	    cout << "gripper force command [open/close]\n";
	    cout << "current pose [x y z theta_x theta_y theta_z]\n";
	    cout << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
	    cout << "************************************************************************************\n";    	
    } else {
	    cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
    }
}

void setup_file(char *task, bool debug=false)
{
	debug_ = debug;
	string fname = "../sessions/";
	fname.append(task);
	fname.append("Ideal_");
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
    outfile << "Participant: Ideal Trajectory" << "\n";
    outfile << "************************************** FORMAT **************************************\n";
    outfile << "current pose [x y z theta_x theta_y theta_z]\n";
    outfile << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
    outfile << "************************************************************************************\n";
    // cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
    if (debug_) {
	    cout << "*************************************** INFO ***************************************\n";
	    cout << "Filename: " << fname.substr(found + 1) << "\n";
    	cout << "Date: " << timestr << "\n";
	    cout << "Task: " << task << "\n";
	    cout << "Participant: Ideal Trajectory" << "\n";
	    cout << "************************************** FORMAT **************************************\n";
	    cout << "current pose [x y z theta_x theta_y theta_z]\n";
	    cout << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
	    cout << "************************************************************************************\n";    	
    } else {
	    cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
    }
}

void write_to_file(k_api::Base::BaseClient* pBase, bool experimental) 
{
	// pBase->GetMeasuredJointSpeeds(); does not work
	k_api::Base::Pose pose;
	k_api::Base::Twist twist;
	k_api::Base::JointAngles ja;
	while (!quit) {
		if (experimental) {
			// write mode
			outfile << mode << "\n";
			if (debug_) cout << mode << "\n";
			// write motion force command
			for (int i = 0; i < (int)old_motion.size(); ++i) {
				outfile << old_motion.at(i) << " ";
				if (debug_) cout << old_motion.at(i) << " ";
			}
			outfile << "\n";
			// write gripper force command
			outfile << old_grasp << "\n";
			if (debug_) cout << "\n" << old_grasp << "\n";
		}
		// grab and write current pose 
		pose = pBase->GetMeasuredCartesianPose();
		outfile << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
		if (debug_) cout << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
		// grab and write current joint angles
		ja = pBase->GetMeasuredJointAngles();
		for (int i = 0; i < ja.joint_angles_size(); ++i) {
			outfile << ja.joint_angles(i).value() << " ";
			if (debug_) cout << ja.joint_angles(i).value() << " ";
		}
		outfile << "\n";
		// grab and write current joint speeds
		// twist = pBase->GetMeasuredTwist();
		// outfile << twist.linear_x() << " " << twist.linear_y() << " " << twist.linear_z() << " " << twist.angular_x() << " " << twist.angular_y() << " " << twist.angular_z() << "\n"; 

		// cout << "\n";
		// cout << ja.joint_angles_size() << endl;
		// separate by a new line
		outfile << "\n";
		if (debug_) cout << "\n\n";
	}
}



#endif  /* GEN3_UTILITIES_H */
