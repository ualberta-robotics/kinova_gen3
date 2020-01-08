/* 
* Author: Laura Petrich
* Date: July 25, 2019
*/


#ifndef GEN3_COMMANDS_H
#define GEN3_COMMANDS_H

#include <BaseClientRpc.h>

using namespace std;

namespace k_api = Kinova::Api;

std::vector<float> start_position_angles = {0.107f, 300.742f, 178.776f, 216.905f, 12.912f, 355.898f, 77.038f}; // START POSITION

void send_joint_angle_command(k_api::Base::BaseClient* base, std::vector<float> &target, int sleeptime)
{    
    auto action = k_api::Base::Action();
    action.set_name("joint angle action");
    action.set_application_data("");
    auto reachJointAngles = action.mutable_reach_joint_angles();
    auto jointAngles = reachJointAngles->mutable_joint_angles();
    for(size_t i = 0 ; i < target.size(); ++i) {
        auto jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(target.at(i));
    }
    base->ExecuteAction(action);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime)); 
}

void send_twist_command(k_api::Base::BaseClient* pBase, const std::array<double, 6> &target, int execution_time=0)
{
    auto command = k_api::Base::TwistCommand();
    // command.set_allocated_twist(k_api::Base::UNSPECIFIED_TWIST_MODE);
    command.set_duration(execution_time);  // Unlimited time to execute
    // k_api::Base::TwistMode mode;
    // mode = command.mode();
    // std::cout << "mode: " << command.has_twist() << std::endl;
    auto twist = command.mutable_twist();
    twist->set_linear_x(target.at(0));
    twist->set_linear_y(target.at(1));
    twist->set_linear_z(target.at(2));
    twist->set_angular_x(target.at(3));
    twist->set_angular_y(target.at(4));
    twist->set_angular_z(target.at(5));
    // pBase->SendTwistCommand(command);
    // std::cout << "SENDING NEW COMMAND: ";
    // for (int i = 0; i < target.size(); ++i) {
    //     std::cout << target.at(i) << " ";
    // }
    // std::cout << std::endl;
}

void send_gripper_command(k_api::Base::BaseClient* pBase, float target, int execution_time=0)
{
    k_api::Base::GripperCommand output;
    output.set_mode(k_api::Base::GRIPPER_FORCE);
    auto gripper = output.mutable_gripper();
    gripper->clear_finger();
    auto finger = gripper->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(target * 0.5);
    output.set_duration(execution_time);    
    pBase->SendGripperCommand(output);
}

bool send_safe_position_command(k_api::Base::BaseClient* pBase)
{
 	// Move arm to ready position
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
    if (action_handle.identifier() == 0) {
        std::cout << "\nCan't reach safe position. Exiting" << std::endl; 
        return false;      
    } else {
        pBase->ExecuteActionFromReference(action_handle);
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Leave time to action to finish
    }
    return true;
}

#endif  /* GEN3_COMMANDS_H */
