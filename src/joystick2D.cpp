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

#include "joystick2d.hh"
#include <unistd.h>
#include <array>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.12"
#define PORT 10000
#define AXISVALUE -32767
#define MODE_XY 0 // Translation-X and Translation-Y
#define MODE_ZR 1 // Translation-Z and Wrist Rotation
#define MODE_WO 2 // Wrist Orientation
#define MODE_FM 3 // Finger Mode
#define BUTTON_BLUE 0
#define BUTTON_RED 1

const float GAINS = 0.5;
int mode;
int switch_count;
bool quit;

void print_mode() 
{
    switch(mode) {
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

void send_twist_command(k_api::Base::BaseClient* pBase, const std::array<float, 6> &cmd)
{
    auto command = k_api::Base::TwistCommand();
    command.set_mode(k_api::Base::UNSPECIFIED_TWIST_MODE);
    command.set_duration(0);  // Unlimited time to execute

    auto twist = command.mutable_twist();
    // std::cout << "command: ";
    // for (int i = 0; i < 6; ++i) {
    //     std::cout << cmd.at(i) << " "; 
    // }
    // std::cout << std::endl;
    twist->set_linear_x(cmd.at(0));
    twist->set_linear_y(cmd.at(1));
    twist->set_linear_z(cmd.at(2));
    twist->set_angular_x(cmd.at(3));
    twist->set_angular_y(cmd.at(4));
    twist->set_angular_z(cmd.at(5));
    pBase->SendTwistCommand(command);
    std::cout << "SENDING NEW COMMAND: ";
    for (int i = 0; i < cmd.size(); ++i) {
        std::cout << cmd.at(i) << " ";
    }
    std::cout << std::endl;
}

void send_gripper_command(k_api::Base::BaseClient* pBase, float cmd)
{
    // std::cout << "gripper command: " << cmd << std::endl;
    k_api::Base::GripperCommand output;
    output.set_mode(k_api::Base::GRIPPER_FORCE);
    auto gripper = output.mutable_gripper();
    gripper->clear_finger();
    auto finger = gripper->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(cmd * 0.5);
    output.set_duration(0);    
    pBase->SendGripperCommand(output);
}

void move_axis(k_api::Base::BaseClient* pBase, int axis, float direction)
{
    // std::cout << axis << " " << direction << std::endl;
    std::array<float, 6> command;
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
            // break;
        default:
            break;
    }
    send_twist_command(pBase, command);
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
// void loop()
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
                // printf("Button %u is %s\n", event.number, event.value == 0 ? "up" : "down");
            } else if (event.isAxis()) {
                move_axis(pBase, event.number, event.value / AXISVALUE);
                // printf("Axis %u is at position %d\n", event.number, event.value);
            }
        }
    }
    std::cout << "TOTAL MODE SWITCHES: " << switch_count << std::endl;
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
    std::cout << "Session created" << std::endl;

    // Create required services
    auto pBase = new k_api::Base::BaseClient(pRouter);

    // // Move arm to ready position
    std::cout << "Moving the arm to a safe position before executing example\n" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = pBase->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0); 
    for( auto action : action_list.action_list()) {
        if(action.name() == "Home") {
            action_handle = action.handle();
        }
    }

    if(action_handle.identifier() == 0) {
        std::cout << "\nCan't reach safe position. Exiting" << std::endl;       
    } else {
        pBase->ExecuteActionFromReference(action_handle);
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Leave time to action to finish
    }
    
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