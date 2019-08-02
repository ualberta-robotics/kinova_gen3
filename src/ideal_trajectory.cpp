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
* Date: July 24, 2019
*/

#include "gen3_utilities.h"
#include "gen3_commands.h"

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <DeviceManagerClientRpc.h>

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.12"
#define PORT 10000

void pour(k_api::Base::BaseClient* pBase)
{
    std::cout << "Moving to start position..." << std::endl;
    std::vector<float> angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f}; // START POSITION
    send_joint_angle_command(pBase, angles, 7000);

    std::cout << "Moving to pickup position..." << std::endl;
    angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f}; // PICKUP
    send_joint_angle_command(pBase, angles, 7000);

    std::cout << "Closing gripper..." << std::endl;
    send_gripper_command(pBase, -1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    std::cout << "Moving to pouring position..." << std::endl;    
    angles = {78.0f, 355.0f, 123.0f, 233.0f, 351.0f, 23.0f, 82.0f}; // GET READY TO POUR
    send_joint_angle_command(pBase, angles, 7000);

    std::cout << "Pouring..." << std::endl;    
    angles = {159.724f, 0.671f, 51.111f, 250.119f, 0.444f, 357.351f, 309.919f}; // POUR
    send_joint_angle_command(pBase, angles, 12000);

    angles = {78.0f, 355.0f, 123.0f, 233.0f, 351.0f, 23.0f, 82.0f}; // GET READY TO POUR
    send_joint_angle_command(pBase, angles, 7000);

    std::cout << "Putting bottle back..." << std::endl;    
    angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f}; // PICKUP
    send_joint_angle_command(pBase, angles, 4000);

    std::cout << "Opening gripper..." << std::endl;
    send_gripper_command(pBase, 1);    
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    std::cout << "Moving to start position..." << std::endl;
    angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f}; // START POSITION
    send_joint_angle_command(pBase, angles, 7000);
    quit = true;
    return;
}

int main(int argc, char **argv)
{
    // Setup API
    auto pTransport = new k_api::TransportClientUdp();
    auto pRouter = new k_api::RouterClient(pTransport, [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); });
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
    quit = false;
    // Setup outfile if required and run program
    if (argc < 2) {
        cout << "\nTo save data to file call program with arg [TASK]" << endl;
        cout << "Running without saving data" << endl;
        pour(pBase);
    } else {
        setup_file(argv[1]);
        thread file_thread(write_to_file, pBase, false);
        if (outfile.is_open()) {
            thread loop_thread(pour, pBase);
            loop_thread.detach();
            file_thread.join();
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