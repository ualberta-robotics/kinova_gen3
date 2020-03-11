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
#include "gen3_commands.h"

#define MODE_TRANS 0 // Translation Mode
#define MODE_ORTN 1 // Wrist Orientation Mode
#define MODE_GRIPPER 2 // Finger Mode

class Joystick3D {
protected:
	ExperimentalUtilities * util_ptr;
	bool quit;
	int mode;
	int count;
	double gripper;
	array<double, 6> motion;

public:
	Joystick3D (ExperimentalUtilities * util_ptr_) : 
		util_ptr(util_ptr_),
		quit(false),
		mode(0),++
		gripper(0.0),
		motion({0,0,0,0,0,0}),
		count(0)
	{
		if (spnav_open() == -1) {
			fprintf(stderr, "Failed to connect to the space navigator daemon\n");
			util_ptr->set_quit(true);
			exit(0);
		}
		std::cout << "Joystick3D object created" << std::endl;
	}

	~Joystick3D() 
	{
		util_ptr->set_quit(true);
		spnav_close();
	}

	void print_mode() 
	{
	    switch (mode) {
	        case MODE_TRANS:
	            std::cout << "\nTranslation Mode" << std::endl;
	            break;
	        case MODE_ORTN:
	            std::cout << "\nWrist Orientation Mode" << std::endl;
	            break;
	        case MODE_GRIPPER:
	            std::cout << "\nGripper Mode" << std::endl;
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
		double d = 0.5;
		// save sign for later use
		if (x > 0)  sgn = 1.0;
		else sgn = -1.0;
		// find dead zone
		if (abs(x) < 100) return 0.0;
		else x -= (sgn * 100);
		// calculate mapped return value [a, b] -> [c, d]
		ret = (abs(x) - a) * ((d - c) / (b - a)) + c;
		// another dead zone
		if (ret < 0.1) return 0.0;
		// round to 2 decimal digits with correct sign
		return floor((sgn * ret * 10) + .5) / 10;
	}

	bool handle_motion(k_api::Base::BaseClient* pBase, spnav_event_motion e, array<double, 6> &v)
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
				v = {map_x((double)e.z), map_x((double)-e.x), map_x((double)-e.ry), 0, 0, 0};
				break;
			case MODE_ORTN:
				v = {0, 0, 0, map_x((double)(e.z - e.rx)*0.8), map_x((double)(e.x + e.rz)*0.8), map_x((double)-e.ry)};
				break;
			case MODE_GRIPPER:
				v = {0, 0, 0, 0, 0, 0};
				x = map_x((double)e.z);
				if (x != gripper) {
					util_ptr->set_gripper_value(x);
					send_gripper_command(pBase, x);
					gripper = x;
				}
				break;
			default:
				break;
		}
		if (v == motion) return false;
		return true;
	}

	void handle_button(int button) 
	{
		if (button == 1) {
			quit = true;
			util_ptr->set_quit(quit);
		} else {
	        mode = (mode + 1) % 3;
			util_ptr->set_mode(mode);
	        print_mode();
	        count += 1;
		}
	}

	void loop(k_api::Base::BaseClient* pBase)
	{
		util_ptr->start_timer();
		print_mode();
		while (!quit) {
			usleep(1000);
			spnav_event sev;
			if (spnav_poll_event(&sev)) {
				if (sev.type == SPNAV_EVENT_MOTION) {
					array<double, 6> new_motion;
					// cout << sev.motion.x << " " << sev.motion.y << " " << sev.motion.z << " " << sev.motion.rx << " " << sev.motion.ry << " " << sev.motion.rz << endl;
					if (handle_motion(pBase, sev.motion, new_motion)) {
						motion = new_motion;
						util_ptr->set_motion_value(motion);
						send_twist_command(pBase, motion, 0);
					} 
				} else if (sev.button.press) {    /* SPNAV_EVENT_BUTTON */
					handle_button(sev.button.bnum);
				}
			}
		}
		util_ptr->stop_timer();
		util_ptr->write_mode_count(count);
	}

private:
};   

int main(int argc, char **argv)
{
    // Setup API
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("iris");
    create_session_info.set_password("IRIS");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    try {
        session_manager->CreateSession(create_session_info);
    } catch (...) {
        cout << "Session could not be created. Check if robot is connected and try again.\n" << endl;
        exit(0);
    }
    std::cout << "Session created" << std::endl;

    // Create DeviceConfigClient and BaseClient
    auto device_config = new k_api::DeviceConfig::DeviceConfigClient(router);
    auto base = new k_api::Base::BaseClient(router);

    // send_joint_angle_command(base, start_position_angles, 7000);

    // Setup outfile if required and run program
	if (argc <= 2) {
		ExperimentalUtilities * eu_ptr = new ExperimentalUtilities(3, "", "");
		cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
		cout << "Running without saving data" << endl;
		Joystick3D * joystick_ptr = new Joystick3D(eu_ptr);
		joystick_ptr->loop(base);
	} else {
		ExperimentalUtilities * eu_ptr = new ExperimentalUtilities(3, string(argv[1]), string(argv[2]));
		Joystick3D * joystick_ptr = new Joystick3D(eu_ptr);
		if (eu_ptr->setup_file()) {
			thread file_thread(&ExperimentalUtilities::write_to_file, eu_ptr, base, true);
			thread loop_thread(&Joystick3D::loop, joystick_ptr, base);
			loop_thread.detach();
			file_thread.join();
		}
	}
    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete device_config;
    delete session_manager;
    delete router;
    delete transport;

    return 0;
};