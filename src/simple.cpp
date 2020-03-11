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
// #include "experimental_utilities.h"
// #include "gen3_commands.h"


class SpaceMouseExample {
protected:
	bool quit;

public:
	SpaceMouseExample() : 
		quit(false)
	{
		if (spnav_open() == -1) {
			fprintf(stderr, "Failed to connect to the space navigator daemon\n");
			quit = true;
			exit(0);
		}
        std::cout << "SpaceMouseExample object created" << std::endl;
	}

	~SpaceMouseExample() 
	{
		quit = true;
		spnav_close();
	}

	void handle_button(int button) 
	{
		if (button == 1) {
			quit = true;
		} 
	}

	void loop()
	{
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
	}

private:
};   

int main(int argc, char **argv)
{
	if (argc <= 2) {
		SpaceMouseExample * joystick_ptr = new SpaceMouseExample();
		joystick_ptr->loop();
	} 

    return 0;
};