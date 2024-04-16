//
// MIT License
//
// Copyright (c) 2024 Leander Stephen D'Souza
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BREAKOUT_GAZEBO_BREAKOUTGAZEBOPLUGIN_H
#define BREAKOUT_GAZEBO_BREAKOUTGAZEBOPLUGIN_H

#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportIface.hh>


class breakoutGazeboPlugin: public gazebo::WorldPlugin {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber contacts_sub_;

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr updateConnection_;
  gazebo::transport::NodePtr node_;

  float ball_speed_x_limit_;
  float ball_speed_y_;
  bool start = false;
  bool orange_hit_ = false;
  bool red_hit_ = false;
  int score_ = 0;
  int highscore_ = 0;
  int lives_ = 1;
  int hit_counter_ = 0;

  std::string prev_hundreds_model_ = "zero0";
  std::string prev_tens_model_ = "zero1";
  std::string prev_ones_model_ = "zero2";
  std::string prev_highscore_hundreds_model_ = "zero3";
  std::string prev_highscore_tens_model_ = "zero4";
  std::string prev_highscore_ones_model_ = "zero5";
  std::string prev_lives_model_ = "one7";

  ignition::math::Pose3d ball_pose_;
  ignition::math::Pose3d hundreds_pose_;
  ignition::math::Pose3d tens_pose_;
  ignition::math::Pose3d ones_pose_;
  ignition::math::Pose3d highscore_hundreds_pose_;
  ignition::math::Pose3d highscore_tens_pose_;
  ignition::math::Pose3d highscore_ones_pose_;
  ignition::math::Pose3d lives_pose_;

 public:
  void Init();
  void contactsCallback(const gazebo_msgs::ContactsState::ConstPtr& msg);
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  void setBallVelocity();
  void deleteModel(std::string model_name);
  void OnUpdate();
  void updateScore();
  void updateHighScore();
  void updateLives();
  void spawnModel(std::string model_name,
    ignition::math::Pose3d pose, std::string tag);
  void respawnBall(bool wait);
  void respawnBricks();
  void increaseBallSpeed(float factor);
};

#endif  // BREAKOUT_GAZEBO_BREAKOUTGAZEBOPLUGIN_H
