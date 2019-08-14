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

#include "joystickBox.hh"
#include "experimental_utilities.h"
#include "gen3_commands.h"

#define AXISVALUE -32767
#define MODE_TRANSX_TRANSY 0 // Translation-X and Translation-Y
#define MODE_TRANSZ_ORNT 1 // Translation-Z and Wrist Orientation
#define MODE_ORTN 2 // Wrist Orientation
#define MODE_GRIPPER 3 // Finger Mode
#define BUTTON_BLUE 0
#define BUTTON_RED 2

const float GAINS = 0.5;

class Joystick2D {
protected:
    ExperimentalUtilities * util_ptr;
    bool quit;
    int mode;
    int count;
    double gripper;
    array<double, 6> motion;

public:
Joystick2D (ExperimentalUtilities * util_ptr_) : 
        util_ptr(util_ptr_),
        quit(false),
        mode(0),
        gripper(0.0),
        motion({0,0,0,0,0,0}),
        count(0)
    {}

    ~Joystick2D() 
    {
        util_ptr->set_quit(true);
    }

    void print_mode() 
    {
        switch (mode) {
            case MODE_TRANSX_TRANSY:
                std::cout << "\nTranslation-X and Translation-Y Mode" << std::endl;
                break;
            case MODE_TRANSZ_ORNT:
                std::cout << "\nTranslation-Z and Wrist Rotation Mode" << std::endl;
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

    bool handle_motion(k_api::Base::BaseClient* pBase, int axis, double direction, array<double, 6> &v)
    {   
        bool ret_val = true;
        switch(mode) {
            case MODE_TRANSX_TRANSY:
                if (axis == 1) {
                    v = {GAINS * direction, 0.0, 0.0, 0.0, 0.0, 0.0};
                } else if (axis == 0) {
                    v = {0.0, GAINS * direction, 0.0, 0.0, 0.0, 0.0};                
                }
                break;
            case MODE_TRANSZ_ORNT:
                if (axis == 1) {
                    v = {0.0, 0.0, GAINS * direction, 0.0, 0.0, 0.0};
                } else if (axis == 0) {
                    v = {0.0, 0.0, 0.0, 0.0, 0.0, -GAINS * direction};                
                }            
                break;
            case MODE_ORTN:
                if (axis == 1) {
                    v = {0.0, 0.0, 0.0, GAINS * direction, 0.0, 0.0};
                } else if (axis == 0) {
                    v = {0.0, 0.0, 0.0, 0.0, GAINS * direction, 0.0};                
                } 
                break;
            case MODE_GRIPPER:
                if (axis == 1 && direction != gripper) {
                    util_ptr->set_gripper_value(direction);
                    send_gripper_command(pBase, direction);
                    gripper = direction;
                }
                ret_val = false;
                break;
            default:
                ret_val = false;
                break;
        }
        return ret_val;
    }

    void handle_button(int button) 
    {
        switch (button) {
            case BUTTON_BLUE:
                mode = (mode + 1) % 4;
                util_ptr->set_mode(mode);
                print_mode();
                count += 1;
                break;
            case BUTTON_RED:
                quit = true;
                util_ptr->set_quit(quit);
                break;
            default:
                break;
        }
    }

    void loop(k_api::Base::BaseClient* pBase)
    {
        util_ptr->start_timer();
        JoystickBox joystick("/dev/input/js0");
        if (!joystick.isFound()) {
            printf("open failed.\n");
            exit(1);
        }
        print_mode();
        int joystick_initialized = 0;
        while (!quit) {
            usleep(1000);
            JoystickBoxEvent event;
            if (joystick.sample(&event) && joystick_initialized++ > 18){
                if (event.isAxis()) {
                    array<double, 6> new_motion;
                    if (handle_motion(pBase, event.number, event.value / AXISVALUE, new_motion)) {
                        util_ptr->set_motion_value(new_motion);
                        send_twist_command(pBase, new_motion, 0);
                        motion = new_motion;
                    }
                } else if (event.isButton() && event.value == 1) {
                    handle_button(event.number);
                }
            }
        }
        util_ptr->stop_timer();
        util_ptr->write_mode_count(count);
    }
};

int main(int argc, char **argv)
{
    // Setup API
    auto errorCallback =  [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    auto pTransport = new k_api::TransportClientUdp();
    auto pRouter = new k_api::RouterClient(pTransport, errorCallback);
    pTransport->connect(IP_ADDRESS, PORT);
    // Create session
    auto createSessionInfo = k_api::Session::CreateSessionInfo();
    createSessionInfo.set_username("iris");
    createSessionInfo.set_password("IRIS");
    createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
    createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)
    std::cout << "\nCreating session for communication" << std::endl;
    auto pSessionMng = new k_api::SessionManager(pRouter);
    pSessionMng->CreateSession(createSessionInfo);
    std::cout << "Session created\n" << std::endl;
    // Create required services
    auto pBase = new k_api::Base::BaseClient(pRouter);
    send_joint_angle_command(pBase, start_position_angles, 7000);
    // Setup outfile if required and run program
    if (argc <= 2) {
        ExperimentalUtilities * eu_ptr = new ExperimentalUtilities(2, "", "");
        cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
        cout << "Running without saving data" << endl;
        Joystick2D * joystick_ptr = new Joystick2D(eu_ptr);
        joystick_ptr->loop(pBase);
    } else {
        ExperimentalUtilities * eu_ptr = new ExperimentalUtilities(2, string(argv[1]), string(argv[2]));
        Joystick2D * joystick_ptr = new Joystick2D(eu_ptr);
        if (eu_ptr->setup_file()) {
            thread file_thread(&ExperimentalUtilities::write_to_file, eu_ptr, pBase, true);
            thread loop_thread(&Joystick2D::loop, joystick_ptr, pBase);
            loop_thread.detach();
            file_thread.join();
        }
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
    return 0;
};