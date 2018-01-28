/**
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "iiwa_ros.h"

using namespace std;

namespace iiwa_ros {

ros::Time last_update_time;

iiwaRos::iiwaRos() { }

void iiwaRos::init(std::string ns)
{
    if(ns[0] != '/' )ns.insert(0,"/");
    if(ns.back() != '/') ns.append("/");

    holder_state_pose_.init(ns + "state/CartesianPose");
    holder_state_motion_finished.init(ns + "state/MotionFinished");
    holder_state_redundance_pose_.init(ns + "state/CartesianRedundancePose");
    holder_state_joint_position_.init(ns + "state/JointPosition");
    holder_state_joint_torque_.init(ns + "state/JointTorque");
    holder_state_wrench_.init(ns + "state/CartesianWrench");
    holder_state_joint_stiffness_.init(ns + "state/JointStiffness");
    holder_state_joint_position_velocity_.init(ns + "state/JointPositionVelocity");
    holder_state_joint_damping_.init(ns + "state/JointDamping");
    holder_state_joint_velocity_.init(ns + "state/JointVelocity");
    holder_state_destination_reached_.init(ns + "state/DestinationReached");

    holder_command_pose_.init(ns + "command/CartesianPose");
    holder_command_redundance_pose_.init(ns + "command/CartesianRedundancePose");
    holder_command_joint_position_.init(ns + "command/JointPosition");
    holder_command_joint_position_velocity_.init(ns + "command/JointPositionVelocity");
    holder_command_joint_velocity_.init(ns + "command/JointVelocity");
    
    smart_servo_service_.setServiceName(ns + "configuration/configureSmartServo");
    path_parameters_service_.setServiceName(ns + "configuration/pathParameters");
    time_to_destination_service_.setServiceName(ns + "state/timeToDestination");
    stop_movement_service_.setServiceName(ns + "command/StopMovement");
    joint_deviation_service_.setServiceName(ns + "command/SetMaxJointDeviation");
}

bool iiwaRos::getRobotIsConnected() {
    ros::Duration diff = (ros::Time::now() - last_update_time);
    return (diff < ros::Duration(0.25));
}

bool iiwaRos::isMotionFinished(){
    std_msgs::Bool b;
    holder_state_motion_finished.get(b);
    return b.data;
}

bool iiwaRos::getCartesianPose(geometry_msgs::PoseStamped& value) {
    return holder_state_pose_.get(value);
}
bool iiwaRos::getCartesianRedundancePose(iiwa_msgs::CartesianRedundancePose& value) {
    return holder_state_redundance_pose_.get(value);
}
bool iiwaRos::getJointPosition(iiwa_msgs::JointPosition& value) {
    return holder_state_joint_position_.get(value);
}
bool iiwaRos::getJointTorque(iiwa_msgs::JointTorque& value) {
    return holder_state_joint_torque_.get(value);
}
bool iiwaRos::getJointStiffness(iiwa_msgs::JointStiffness& value) {
    return holder_state_joint_stiffness_.get(value);
}
bool iiwaRos::getCartesianWrench(geometry_msgs::WrenchStamped& value) {
    return holder_state_wrench_.get(value);
}
bool iiwaRos::getJointVelocity(iiwa_msgs::JointVelocity& value) {
    return holder_state_joint_velocity_.get(value);
}
bool iiwaRos::getJointPositionVelocity(iiwa_msgs::JointPositionVelocity& value) {
    return holder_state_joint_position_velocity_.get(value);
}
bool iiwaRos::getJointDamping(iiwa_msgs::JointDamping& value) {
    return holder_state_joint_damping_.get(value);
}

void iiwaRos::setCartesianPose(const geometry_msgs::PoseStamped& position) {
    holder_command_pose_.set(position);
    holder_command_pose_.publishIfNew();
}
void iiwaRos::setCartesianRedundancePose(const iiwa_msgs::CartesianRedundancePose& pose){
    holder_command_redundance_pose_.set(pose);
    holder_command_redundance_pose_.publishIfNew();
}
void iiwaRos::setJointPosition(const iiwa_msgs::JointPosition& position)  {
    holder_command_joint_position_.set(position);
    holder_command_joint_position_.publishIfNew();
}
void iiwaRos::setJointVelocity(const iiwa_msgs::JointVelocity& velocity) {
    holder_command_joint_velocity_.set(velocity);
    holder_command_joint_velocity_.publishIfNew();
    
}
void iiwaRos::setJointPositionVelocity(const iiwa_msgs::JointPositionVelocity& value) {
    holder_command_joint_position_velocity_.set(value);
    holder_command_joint_position_velocity_.publishIfNew();
}
}
