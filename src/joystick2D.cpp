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

#include "joystick2d.hh"
#include "experimental_utilities.h"

#include "gen3_utilities.h"
#include "gen3_commands.h"

#include <unistd.h>
#include <array>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>

// namespace k_api = Kinova::Api;

// #define IP_ADDRESS "192.168.1.12"
// #define PORT 10000
#define AXISVALUE -32767
#define MODE_XY 0 // Translation-X and Translation-Y
#define MODE_ZR 1 // Translation-Z and Wrist Rotation
#define MODE_WO 2 // Wrist Orientation
#define MODE_FM 3 // Finger Mode
#define BUTTON_BLUE 0
#define BUTTON_RED 2

const float GAINS = 0.5;
// int switch_count;

void print_mode() 
{
    switch (mode) {
        case MODE_XY:
            std::cout << "\nTranslation-X and Translation-Y Mode" << std::endl;
            break;
        case MODE_ZR:
            std::cout << "\nTranslation-Z and Wrist Rotation Mode" << std::endl;
            break;
        case MODE_WO:
            std::cout << "\nWrist Orientation Mode" << std::endl;
            break;
        case MODE_FM:
            std::cout << "\nFinger Mode" << std::endl;
            break;
        default:
            break;
    }
}

void move_axis(k_api::Base::BaseClient* pBase, int axis, double direction)
{
    std::array<double, 6> command = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    switch(mode) {
        case MODE_XY:
            if (axis == 1) {
                command = {GAINS * direction, 0.0, 0.0, 0.0, 0.0, 0.0};
            } else if (axis == 0) {
                command = {0.0, GAINS * direction, 0.0, 0.0, 0.0, 0.0};                
            }
            break;
        case MODE_ZR:
            if (axis == 1) {
                command = {0.0, 0.0, GAINS * direction, 0.0, 0.0, 0.0};
            } else if (axis == 0) {
                command = {0.0, 0.0, 0.0, 0.0, 0.0, -GAINS * direction};                
            }            
            break;
        case MODE_WO:
            if (axis == 1) {
                command = {0.0, 0.0, 0.0, GAINS * direction, 0.0, 0.0};
            } else if (axis == 0) {
                command = {0.0, 0.0, 0.0, 0.0, GAINS * direction, 0.0};                
            } 
            break;
        case MODE_FM:
            if (axis == 1) {
                send_gripper_command(pBase, direction);
            }
            return;
        default:
            break;
    }
    send_twist_command(pBase, command, 0);
}

void handle_button(int button) 
{
    switch (button) {
        case BUTTON_BLUE:
            mode = (mode + 1) % 4;
            print_mode();
            switch_count += 1;
            break;
        case BUTTON_RED:
            quit = true;
            break;
        default:
            break;
    }
}

void loop(k_api::Base::BaseClient* pBase)
{
    // Setup Joystick
    Joystick2D joystick("/dev/input/js0");
    if (!joystick.isFound()) {
        printf("open failed.\n");
        exit(1);
    }
    quit = false;
    mode = 0;
    switch_count = 0;
    print_mode();
    int joystick_initialized = 0;
    while (!quit) {
        usleep(1000);
        Joystick2DEvent event;
        if (joystick.sample(&event) && joystick_initialized++ > 18){
            if (event.isButton() && event.value == 1) {
                handle_button(event.number);
            } else if (event.isAxis()) {
                move_axis(pBase, event.number, event.value / AXISVALUE);
            }
        }
    }
    std::cout << "TOTAL MODE SWITCHES: " << switch_count << std::endl;
    outfile << "TOTAL MODE SWITCHES: " << switch_count << endl;
}

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
    time_t start, end; 
    // if (!send_safe_position_command(pBase)) return 0;
    // Setup outfile if required and run program
    if (argc <= 2) {
        cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
        cout << "Running without saving data" << endl;
        loop(pBase);
    } else {
        if (argc == 4) setup_file(argv[1], argv[2], "2D_", true); // debug - also prints to screen
        else setup_file(argv[1], argv[2], "2D_");
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
    return 0;
};