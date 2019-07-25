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
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <ctime>
#include <cstddef>         // std::size_t
#include <thread>         // std::thread

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <DeviceManagerClientRpc.h>

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.12"
#define PORT 10000

ofstream outfile;
bool quit;


void setup_file(char *task, char* participant)
{
    string fname = "../sessions/";
    fname.append(task);
    // fname.append("3D");
    fname.append(participant);
    std::time_t t = std::time(nullptr);
    char mbstr[100];
    if (std::strftime(mbstr, sizeof(mbstr), "%y%m%e%H%M", std::localtime(&t))) {
        fname.append(mbstr);
    }
    outfile.open(fname);
    std::size_t found = fname.find_last_of("/\\");
    char timestr[100];
    outfile << "*************************************** INFO ***************************************\n";
    outfile << "Filename: " << fname.substr(found + 1) << "\n";
    if (std::strftime(timestr, sizeof(timestr), "%c", std::localtime(&t))) {
    outfile << "Date: " << timestr << "\n";
    }
    outfile << "Task: " << task << "\n";
    outfile << "Participant: " << participant << "\n";
    outfile << "************************************** FORMAT **************************************\n";
    // outfile << "mode\n";
    // outfile << "motion force command [linear_x linear_y linear_z angular_x angular_y angular_z]\n";
    // outfile << "gripper force command [open/close]\n";
    outfile << "current pose [x y z theta_x theta_y theta_z]\n";
    outfile << "current joint angles [j1 j2 j3 j4 j5 j6 j7]\n";
    outfile << "************************************************************************************\n";
    cout << "\nWriting to file: " << fname.substr(found + 1) << endl;
}

void write_to_file(k_api::Base::BaseClient* pBase) 
{
    // pBase->GetMeasuredJointSpeeds(); does not work
    k_api::Base::Pose pose;
    k_api::Base::Twist twist;
    k_api::Base::JointAngles ja;
    while (!quit) {
        // write mode
        // outfile << mode << "\n";
        // write motion force command
        // for (int i = 0; i < old_motion.size(); ++i) {
        //     outfile << old_motion.at(i) << " ";
        // }
        // outfile << "\n";
        // write gripper force command
        // outfile << old_grasp << "\n";
        // grab and write current pose 
        pose = pBase->GetMeasuredCartesianPose();
        outfile << pose.x() << " " << pose.y() << " " << pose.z() << " " << pose.theta_x() << " " << pose.theta_y() << " " << pose.theta_z() << "\n"; 
        // grab and write current joint angles
        ja = pBase->GetMeasuredJointAngles();
        for (int i = 0; i < ja.joint_angles_size(); ++i) {
            outfile << ja.joint_angles(i).value() << " ";
        }
        outfile << "\n";
        // grab and write current joint speeds
        // twist = pBase->GetMeasuredTwist();
        // outfile << twist.linear_x() << " " << twist.linear_y() << " " << twist.linear_z() << " " << twist.angular_x() << " " << twist.angular_y() << " " << twist.angular_z() << "\n"; 

        // cout << "\n";
        // cout << ja.joint_angles_size() << endl;
        // separate by a new line
        outfile << "\n";
    }
}

void createAngularAction(k_api::Base::Action* action, std::vector<float> &angles)
{
    // std::cout << "\nCreating angular action" << std::endl;

    action->set_name("Example angular action");
    action->set_application_data("");    

    auto reachJointAngles = action->mutable_reach_joint_angles();
    auto jointAngles = reachJointAngles->mutable_joint_angles();

    // std::vector<float> angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   
    for(size_t i = 0 ; i < angles.size(); ++i)
    {
        auto jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(angles.at(i));
    }
}

void createSequenceStart(k_api::Base::BaseClient* pBase)
{
    // std::cout << "\nCreating Pickup Sequence" << std::endl;

    auto sequence = k_api::Base::Sequence();
    sequence.set_name("Start sequence");
    // Start position angles
    std::vector<float> angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f};   
    auto task_0 = sequence.add_tasks();
    task_0->set_group_identifier(0);
    createAngularAction(task_0->mutable_action(), angles);
    // // Pickup angles
    // angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f};   
    // auto task_1 = sequence.add_tasks();
    // task_1->set_group_identifier(1);
    // createAngularAction(task_1->mutable_action(), angles);

    // std::cout << "Creating sequence on device and executing it" << std::endl;
    auto sequenceHandle = pBase->CreateSequence(sequence);
    pBase->PlaySequence(sequenceHandle);

    // std::cout << "Waiting 30 seconds for movement to finish ..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // (milliseconds)

    // std::cout << "Pickup Sequence Complete" << std::endl;
}

void createSequencePickup(k_api::Base::BaseClient* pBase)
{
    // std::cout << "\nCreating Pickup Sequence" << std::endl;

    auto sequence = k_api::Base::Sequence();
    sequence.set_name("Pickup sequence");
    // // Start position angles
    // std::vector<float> angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f};   
    // auto task_0 = sequence.add_tasks();
    // task_0->set_group_identifier(0);
    // createAngularAction(task_0->mutable_action(), angles);
    // Pickup angles
    std::vector<float> angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f};   
    auto task_1 = sequence.add_tasks();
    task_1->set_group_identifier(0);
    createAngularAction(task_1->mutable_action(), angles);

    // std::cout << "Creating sequence on device and executing it" << std::endl;
    auto sequenceHandle = pBase->CreateSequence(sequence);
    pBase->PlaySequence(sequenceHandle);

    // std::cout << "Waiting 30 seconds for movement to finish ..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // (milliseconds)

    // std::cout << "Pickup Sequence Complete" << std::endl;
}

void createSequencePour(k_api::Base::BaseClient* pBase)
{
    // std::cout << "\nCreating Pouring Sequence" << std::endl;

    auto sequence = k_api::Base::Sequence();
    sequence.set_name("Pour sequence");
    // Move to mug angles
    std::vector<float> angles = {78.932f, 1.369f, 127.485f, 243.213f, 8.213f, 4.232f, 83.745f};   
    auto task_0 = sequence.add_tasks();
    task_0->set_group_identifier(0);
    createAngularAction(task_0->mutable_action(), angles);
    // Pour water angles
    angles = {79.016f, 3.616f, 128.657f, 244.089f, 7.935f, 7.904f, 327.779f};   
    auto task_1 = sequence.add_tasks();
    task_1->set_group_identifier(1);
    createAngularAction(task_1->mutable_action(), angles);
    // Put bottle back angles
    angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f};   
    auto task_2 = sequence.add_tasks();
    task_2->set_group_identifier(2);
    createAngularAction(task_2->mutable_action(), angles);

    // std::cout << "Creating sequence on device and executing it" << std::endl;
    auto sequenceHandle = pBase->CreateSequence(sequence);
    pBase->PlaySequence(sequenceHandle);

    // std::cout << "Waiting 30 seconds for movement to finish ..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000)); // (milliseconds)

    // std::cout << "Pouring Sequence Complete" << std::endl;
}

void send_gripper_command(k_api::Base::BaseClient* pBase, float cmd)
{
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

void send_joint_angle_command(k_api::Base::BaseClient* base, std::vector<float> &target, int sleeptime)
{    
    auto action = k_api::Base::Action();
    action.set_name("joint angle action");
    action.set_application_data("");

    auto reachJointAngles = action.mutable_reach_joint_angles();
    auto jointAngles = reachJointAngles->mutable_joint_angles();

    // std::vector<float> angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   
    for(size_t i = 0 ; i < target.size(); ++i)
    {
        auto jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(target.at(i));
    }
    // std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    // std::cout << "Waiting 20 seconds for movement to finish ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime)); 
}

void send_pose_command(k_api::Base::BaseClient* base, std::array<float, 6> &target)
{    
    auto action = k_api::Base::Action();
    action.set_name("Cartesian action movement");
    action.set_application_data("");

    auto constrainedPose = action.mutable_reach_pose();
    auto pose = constrainedPose->mutable_target_pose();
    pose->set_x(target.at(0));           // x (meters)
    pose->set_y(target.at(1));           // y (meters)
    pose->set_z(target.at(2));          // z (meters)
    pose->set_theta_x(target.at(3));    // theta x (degrees)
    pose->set_theta_y(target.at(4));    // theta y (degrees)
    pose->set_theta_z(target.at(5));    // theta z (degrees)

    // std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    // std::cout << "Waiting 20 seconds for movement to finish ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(20000)); 
}

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
    // angles = {78.932f, 1.369f, 127.485f, 243.213f, 8.213f, 4.232f, 83.745f}; // GET READY TO POUR
    send_joint_angle_command(pBase, angles, 7000);

    std::cout << "Pouring..." << std::endl;    
    angles = {159.724f, 0.671f, 51.111f, 250.119f, 0.444f, 357.351f, 309.919f}; // POUR
    // angles = {140.812f, 354.46f, 69.739f, 253.373f, 349.95f, 356.267f, 314.938f}; // POUR
    // // angles = {79.016f, 3.616f, 128.657f, 244.089f, 7.935f, 7.904f, 300.779f}; // POUR
    send_joint_angle_command(pBase, angles, 12000);

    angles = {78.0f, 355.0f, 123.0f, 233.0f, 351.0f, 23.0f, 82.0f}; // GET READY TO POUR
    // angles = {78.932f, 1.369f, 127.485f, 243.213f, 8.213f, 4.232f, 83.745f}; // GET READY TO POUR
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
    return
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
    std::cout << "Session created" << std::endl;

    // Create required services
    auto pBase = new k_api::Base::BaseClient(pRouter);
    quit = false;
    if (argc <= 2) {
        cout << "\nTo save data to file call program with args [TASK] [PARTICIPANT]" << endl;
        cout << "Running without saving data" << endl;
        // loop(pBase);
        // thread loop_thread(loop, pBase);
        // loop_thread.detach();
        pour(pBase);
    } else {
        setup_file(argv[1], argv[2]);
        thread file_thread(write_to_file, pBase);
        if (outfile.is_open()) {
            thread loop_thread(pour, pBase);
            loop_thread.detach();
            file_thread.join();
            // loop(pBase);
        }
        outfile.close();
    }

    // std::cout << "Moving to start position..." << std::endl;
    // std::vector<float> angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f}; // START POSITION
    // send_joint_angle_command(pBase, angles, 7000);

    // std::cout << "Moving to pickup position..." << std::endl;
    // angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f}; // PICKUP
    // send_joint_angle_command(pBase, angles, 7000);

    // std::cout << "Closing gripper..." << std::endl;
    // send_gripper_command(pBase, -1);
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    // std::cout << "Moving to pouring position..." << std::endl;    
    // angles = {78.0f, 355.0f, 123.0f, 233.0f, 351.0f, 23.0f, 82.0f}; // GET READY TO POUR
    // // angles = {78.932f, 1.369f, 127.485f, 243.213f, 8.213f, 4.232f, 83.745f}; // GET READY TO POUR
    // send_joint_angle_command(pBase, angles, 7000);

    // std::cout << "Pouring..." << std::endl;    
    // angles = {159.724f, 0.671f, 51.111f, 250.119f, 0.444f, 357.351f, 309.919f}; // POUR
    // // angles = {140.812f, 354.46f, 69.739f, 253.373f, 349.95f, 356.267f, 314.938f}; // POUR
    // // // angles = {79.016f, 3.616f, 128.657f, 244.089f, 7.935f, 7.904f, 300.779f}; // POUR
    // send_joint_angle_command(pBase, angles, 12000);

    // angles = {78.0f, 355.0f, 123.0f, 233.0f, 351.0f, 23.0f, 82.0f}; // GET READY TO POUR
    // // angles = {78.932f, 1.369f, 127.485f, 243.213f, 8.213f, 4.232f, 83.745f}; // GET READY TO POUR
    // send_joint_angle_command(pBase, angles, 7000);

    // std::cout << "Putting bottle back..." << std::endl;    
    // angles = {104.13f, 287.682f, 97.899f, 247.06f, 355.234f, 343.534f, 26.965f}; // PICKUP
    // send_joint_angle_command(pBase, angles, 4000);

    // std::cout << "Opening gripper..." << std::endl;
    // send_gripper_command(pBase, 1);    
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    // std::cout << "Moving to start position..." << std::endl;
    // angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f}; // START POSITION
    // send_joint_angle_command(pBase, angles, 7000);

    // std::cout << "\nMoving the arm to a safe position before executing example" << std::endl;
    // auto action_type = k_api::Base::RequestedActionType();
    // action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    // auto action_list = pBase->ReadAllActions(action_type);
    // auto action_handle = k_api::Base::ActionHandle();
    // action_handle.set_identifier(0); 
    // for( auto action : action_list.action_list())
    // {
    //     if(action.name() == "Home")
    //     {
    //         action_handle = action.handle();
    //     }
    // }

    // if(action_handle.identifier() == 0)
    // {
    //     std::cout << "\nCan't reach safe position. Exiting" << std::endl;       
    // }
    // else
    // {
    //     pBase->ExecuteActionFromReference(action_handle);
    //     std::this_thread::sleep_for(std::chrono::seconds(10)); // Leave time to action to finish
    // }

    // std::array<float, 6> start_pose = {0.238, 0.004, 0.544, 87.826, 0.937, 89.605};
    // std::array<float, 6> pickup_pose = {0.506, -0.029, 0.368, 96.533, 3.709, 37.005};
    // std::array<float, 6> mug_pose = {0.485, -0.235, 0.446, 113.414, 0.45, 64.899};
    // std::array<float, 6> pour_pose = {0.492, -0.255, 0.458, 51.621, -123.893, 97.798};
    // std::cout << "Moving to start pose..." << std::endl;
    // send_pose_command(pBase, start_pose);
    // std::cout << "Moving to pickup pose..." << std::endl;
    // send_pose_command(pBase, pickup_pose);
    // std::cout << "Closing gripper..." << std::endl;
    // send_gripper_command(pBase, -1);
    // std::cout << "Moving to mug pose..." << std::endl;
    // send_pose_command(pBase, mug_pose);
    // std::cout << "Moving to pour pose..." << std::endl;
    // send_pose_command(pBase, pour_pose);
    // std::cout << "Moving to pickup pose..." << std::endl;
    // send_pose_command(pBase, pickup_pose);
    // std::cout << "Opening gripper..." << std::endl;
    // send_gripper_command(pBase, 1);
    // std::cout << "Moving to start position..." << std::endl;
    // createSequenceStart(pBase);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // (milliseconds)

    // std::cout << "Picking up the bottle..." << std::endl;
    // createSequencePickup(pBase);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // (milliseconds)

    // std::cout << "Closing gripper..." << std::endl;
    // send_gripper_command(pBase, -1);   
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // (milliseconds) 

    // std::cout << "Pouring..." << std::endl;
    // createSequencePour(pBase);
    // std::this_thread::sleep_for(std::chrono::milliseconds(15000)); // (milliseconds)

    // std::cout << "Opening gripper..." << std::endl;
    // send_gripper_command(pBase, 1);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // (milliseconds) 

    // std::cout << "Moving to start position..." << std::endl;
    // createSequenceStart(pBase);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // (milliseconds)

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