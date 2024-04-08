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
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Twist.h>

#include <queue>
#include <random>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportIface.hh>


class breakoutGazeboPlugin: public gazebo::ModelPlugin {
 private:
  ros::NodeHandle nh_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr robot_model_;

  gazebo::event::ConnectionPtr updateConnection_;
  gazebo::transport::NodePtr node_;

  bool trial = false;

 public:
  void Initialize();
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
};

#endif  // BREAKOUT_GAZEBO_BREAKOUTGAZEBOPLUGIN_H
