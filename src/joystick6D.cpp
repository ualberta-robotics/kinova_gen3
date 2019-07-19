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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <spnav.h>
// #include <iostream>
// #include <unistd.h>
#include <array>
#include <cmath>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>

using namespace std;

namespace k_api = Kinova::Api;


#define IP_ADDRESS "192.168.1.12"
#define PORT 10000

double grip;
bool quit;
array<double, 6> old_motion;

void sig(int s)
{
	spnav_close();
	exit(0);
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
	return floor((sgn * ret * 100) + .5) / 100;
}

bool handle_motion(spnav_event_motion motion, array<double, 6> &v)
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
	// cout << endl;
	array<int, 6> tempv = {motion.z, -motion.x, motion.y, motion.rz, -motion.rx, motion.ry};
	for (int i = 0; i < tempv.size(); ++i) {
		v.at(i) = map_x((double)tempv.at(i));
		// cout << tempv.at(i) << " -> " << v.at(i) << endl;
	}
	if (v == old_motion) return false;
	// cout << endl;
	return true;


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
	pBase->SendTwistCommand(command);
}

void send_gripper_command(k_api::Base::BaseClient* pBase)
{
	// cout << "gripper command: " << cmd << endl;
	k_api::Base::GripperCommand output;
	output.set_mode(k_api::Base::GRIPPER_FORCE);
	auto gripper = output.mutable_gripper();
	gripper->clear_finger();
	auto finger = gripper->add_finger();
	finger->set_finger_identifier(1);
	finger->set_value(grip * 0.5);
	output.set_duration(0);    
	pBase->SendGripperCommand(output);
	cout << "sending grasp command " << grip << endl;
	grip *= -1.0;
}


void handle_button(k_api::Base::BaseClient* pBase, int button) 
{
	if (button == 1) {
		quit = true;
	} else {
		send_gripper_command(pBase);
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
	grip = 1.0;
	while (!quit) {
		usleep(1000);
		spnav_event sev;
		if (spnav_wait_event(&sev)) {
			if(sev.type == SPNAV_EVENT_MOTION) {
				// printf("SAMPLE got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
				// printf("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);
				array<double, 6> new_motion;
				if (handle_motion(sev.motion, new_motion)) {
					old_motion = new_motion;
					send_twist_command(pBase, new_motion);
					cout << "SENDING NEW COMMAND: ";
					for (int i = 0; i < new_motion.size(); ++i) {
						cout << new_motion.at(i) << " ";
					}
					cout << endl;
				} 
				// else {
				// 	cout << "SAME" << endl;
				// }
				// send_twist_command(pBase, sev.motion);
			} else if (sev.button.press) {    /* SPNAV_EVENT_BUTTON */
				// printf("SAMPLE got button %s event b(%d)\n", sev.button.press ? "press" : "release", sev.button.bnum);
				handle_button(pBase, sev.button.bnum);
			}
		} 
	}
	spnav_close();
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

	// // Move arm to ready position
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
	
	loop(pBase);
	// loop();
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