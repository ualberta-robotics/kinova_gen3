#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <X11/Xlib.h>
#include <spnav.h>

void sig(int s)
{
	spnav_close();
	exit(0);
}

class SpaceMouse { 
	private: 

	public: 
		spnav_event sev;
		int number;
		float value;
		SpaceMouse();
		~SpaceMouse();
		// bool isAxis();
		// bool isButton(); 
	 	bool sample();
	 	// bool sampleButton();

}; 

SpaceMouse::SpaceMouse()
{
	signal(SIGINT, sig);
	if (spnav_open() == -1) {
		fprintf(stderr, "failed to connect to the space navigator daemon\n");
		exit(0);
	}
}


SpaceMouse::~SpaceMouse()
{
	spnav_close();
	exit(0);
}

bool SpaceMouse::sample()
{
	spnav_wait_event(&sev);
	if(sev.type == SPNAV_EVENT_MOTION) {
		printf("SAMPLE got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
		printf("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);
	} else {    /* SPNAV_EVENT_BUTTON */
		printf("SAMPLE got button %s event b(%d)\n", sev.button.press ? "press" : "release", sev.button.bnum);
	}
	return true;
}

int main(void)
{
	// // spnav_event sev;
	// // signal(SIGINT, sig);
	// // if(spnav_open()==-1) {
	// // 	fprintf(stderr, "failed to connect to the space navigator daemon\n");
	// // 	return 1;
	// // }
	// while (spnav_wait_event(&sev)) {
	// 	if(sev.type == SPNAV_EVENT_MOTION) {
	// 		printf("got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
	// 		printf("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);
	// 	} else {    /* SPNAV_EVENT_BUTTON */
	// 		printf("got button %s event b(%d)\n", sev.button.press ? "press" : "release", sev.button.bnum);
	// 	}
	// }

	// spnav_close();
	SpaceMouse mouse;
	while (true) {
		mouse.sample();
	}
	return 0;
}
