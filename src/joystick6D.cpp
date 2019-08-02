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
#include "experimental_utilities.h"

#include "gen3_utilities.h"
#include "gen3_commands.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <signal.h>
// #include <array>
// #include <cmath>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>

// using namespace std;

// namespace k_api = Kinova::Api;

// #define IP_ADDRESS "192.168.1.12"
// #define PORT 10000
#define MODE_MOTION 0 // Motion Mode
#define MODE_FINGER 1 // Finger Mode

// int switch_count;

void sig(int s)
{
	spnav_close();
	exit(0);
}

void print_mode() 
{
    switch(mode) {
        case MODE_MOTION:
            std::cout << "\nMotion Mode" << std::endl;
            break;
        case MODE_FINGER:
            std::cout << "\nFinger Mode" << std::endl;
            break;
        default:
            break;
    }
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
	double x;
	switch (mode) {
		case MODE_MOTION:
			v = {(double)motion.z, (double)-motion.x, (double)motion.y, (double)-motion.rx, (double)motion.rz, (double)-motion.ry};
			for (int i = 0; i < (int)v.size(); ++i) {
				v.at(i) = map_x((double)v.at(i));
			}
			break;
		case MODE_FINGER:
			v = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
        mode = (mode + 1) % 2;
        print_mode();
        switch_count += 1;
	}
}

void loop(k_api::Base::BaseClient* pBase)
{
	// Setup SpaceMouse
	signal(SIGINT, sig);
	if (spnav_open() == -1) {
		fprintf(stderr, "failed to connect to the space navigator daemon\n");
		exit(0);
	}
	quit = false;
	switch_count = 0;
	mode = 0;
	old_grasp = 0.0;
	print_mode();
	while (!quit) {
		usleep(1000);
		spnav_event sev;
		if (spnav_wait_event(&sev)) {
			if(sev.type == SPNAV_EVENT_MOTION) {
				array<double, 6> new_motion;
				if (handle_motion(pBase, sev.motion, new_motion)) {
					old_motion = new_motion;
					send_twist_command(pBase, new_motion, 1);
					// cout << "SENDING NEW COMMAND: ";
					// for (int i = 0; i < new_motion.size(); ++i) {
					// 	cout << new_motion.at(i) << " ";
					// }
					// cout << endl;
				} 
			} else if (sev.button.press) {    /* SPNAV_EVENT_BUTTON */
				handle_button(sev.button.bnum);
			}
		} 
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
	cout << "Session created\n" << endl;
	// Create required services
	auto pBase = new k_api::Base::BaseClient(pRouter);
    // if (!send_safe_position_command(pBase)) return 0;
	send_joint_angle_command(pBase, start_position_angles, 7000);
	time_t start, end; 

    // Setup outfile if required and run program
	if (argc <= 2) {
		cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
		cout << "Running without saving data" << endl;
		loop(pBase);
	} else {
		if (argc == 4) setup_file(argv[1], argv[2], "6D_", true);
		else setup_file(argv[1], argv[2], "6D_");
		thread file_thread(write_to_file, pBase, true);
		if (outfile.is_open()) {
			time(&start);
		    ios_base::sync_with_stdio(false); 
			thread loop_thread(loop, pBase);
			loop_thread.detach();
			file_thread.join();
			time(&end); 
		    double time_taken = double(end - start); 
			cout << "TOTAL EXECUTION TIME: " << time_taken << " SECONDS" << endl;
			outfile << "TOTAL EXECUTION TIME: " << time_taken << " SECONDS" << endl;
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