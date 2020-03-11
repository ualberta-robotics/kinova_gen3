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
* Date: March 11, 2020
*/

#include "spnavkinova.h"
#include "experimental_utilities.h"
// #include "gen3_commands.h"


class SpaceMouseExample {
protected:
	ExperimentalUtilities * util_ptr;
	bool quit;

public:
	SpaceMouseExample (ExperimentalUtilities * util_ptr_) : 
		util_ptr(util_ptr_),
		quit(false),
	{
		if (spnav_open() == -1) {
			fprintf(stderr, "Failed to connect to the space navigator daemon\n");
			util_ptr->set_quit(true);
			exit(0);
		}
        std::cout << "SpaceMouseExample object created" << std::endl;
	}

	~SpaceMouseExample() 
	{
		util_ptr->set_quit(true);
		spnav_close();
	}

	void handle_button(int button) 
	{
		if (button == 1) {
			quit = true;
			util_ptr->set_quit(quit);
		} 
	}

	void loop()
	{
		util_ptr->start_timer();
		while (!quit) {
			usleep(1000);
			spnav_event sev;
			if (spnav_poll_event(&sev)) {
				if (sev.type == SPNAV_EVENT_MOTION) {
					cout << sev.motion.x << " " << sev.motion.y << " " << sev.motion.z << " " << sev.motion.rx << " " << sev.motion.ry << " " << sev.motion.rz << endl;
				} else if (sev.button.press) {    /* SPNAV_EVENT_BUTTON */
					handle_button(sev.button.bnum);
				}
			}
		}
		util_ptr->stop_timer();
	}

private:
};   

int main(int argc, char **argv)
{
    // Setup outfile if required and run program
	if (argc <= 2) {
		ExperimentalUtilities * eu_ptr = new ExperimentalUtilities(6, "", "");
		SpaceMouseExample * joystick_ptr = new SpaceMouseExample(eu_ptr);
		joystick_ptr->loop();
	} 

    return 0;
};