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

#include <breakout_gazebo/breakoutGazeboPlugin.h>



void breakoutGazeboPlugin::Init() {
  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init("breakout_gazebo");

  ros::init(
    ros::M_string(), "breakout_gazebo_node",
    ros::init_options::NoSigintHandler);

  ball_speed_x_ = 7.0;
  ball_speed_y_ = -7.0;

  contacts_sub_ = nh_.subscribe(
    "/robot_bumper", 1, &breakoutGazeboPlugin::contactsCallback, this);
}


void breakoutGazeboPlugin::contactsCallback(
  const gazebo_msgs::ContactsState::ConstPtr& msg) {
  if (msg->states.size() > 0) {
    // delete model if collision2_name has the name brick in it
    if (msg->states[0].collision2_name.find("brick") != std::string::npos) {
      std::string model_name = msg->states[0].collision2_name;
      // remove ::base_link::collision from the model name
      model_name = model_name.substr(0, model_name.find("::"));
      deleteModel(model_name);
    }
  }
}

void breakoutGazeboPlugin::deleteModel(std::string model_name) {
  gazebo::physics::ModelPtr brick_model = world_->ModelByName(model_name);
  world_->RemoveModel(brick_model);
}


void breakoutGazeboPlugin::setBallVelocity() {
  gazebo::physics::ModelPtr ball_model = world_->ModelByName("bouncy_ball");
  gazebo::physics::LinkPtr ballLink = ball_model->GetLink("ball_link");
  ignition::math::Vector3d initialVelocity(ball_speed_x_, 0.0, ball_speed_y_);
  ballLink->SetLinearVel(initialVelocity);
}

void breakoutGazeboPlugin::Load(
  gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  world_ = _parent;
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&breakoutGazeboPlugin::OnUpdate, this));
}

void breakoutGazeboPlugin::OnUpdate() {
  if (!start) {
    setBallVelocity();
    start = true;
  }
}


GZ_REGISTER_WORLD_PLUGIN(breakoutGazeboPlugin)
