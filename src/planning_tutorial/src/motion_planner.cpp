/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(5.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("sia5");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already 
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose. 
  geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose = start_pose;
  target_pose.position.y -= 0.0;
  group.setPoseTarget(target_pose);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  //success = group.plan(my_plan);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.

  ROS_INFO("Visualizing plan 1 (arm1) %s",success?"":"FAILED");

  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(5.0);
  group.move();
  sleep(3.0);

  geometry_msgs::Pose target_pose1 = start_pose;
target_pose1.position.z += 0.1;
target_pose1.position.x -= 0.1;
  group.setPoseTarget(target_pose1);

  success = group.plan(my_plan);

  //success = group.plan(my_plan);
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active 
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is 
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
 
  /* Uncomment below line when working with a real robot*/
/* group.move() */

  ROS_INFO("Visualizing plan 2 (arm2) %s",success?"":"FAILED");

  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(5.0);
  group.move();
  sleep(5.0);

  geometry_msgs::Pose target_pose2 = start_pose;
  target_pose2.position.x += 0.1;
  target_pose2.position.z += 0.2;
  group.setPoseTarget(target_pose2);

  success = group.plan(my_plan);

  //success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 3 (diagonal) %s",success?"":"FAILED");

  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(5.0);
  group.move();
  sleep(3.0);

  geometry_msgs::Pose target_pose3 = start_pose;
  group.setPoseTarget(target_pose3);

  success = group.plan(my_plan);

  //success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 4 (diagonal) %s",success?"":"FAILED");

  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(5.0);
  group.move();
  sleep(3.0);

// END_TUTORIAL



  ros::shutdown();  
  return 0;
}
