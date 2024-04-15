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

  ball_speed_x_ = 0.0;
  ball_speed_y_ = -14.0;

  hundreds_pose_ = ignition::math::Pose3d(-8.522, 0, 26.4, 0, 0, 0);
  tens_pose_ = ignition::math::Pose3d(-6.9725, 0, 26.4, 0, 0, 0);
  ones_pose_ = ignition::math::Pose3d(-5.423, 0, 26.4, 0, 0, 0);

  contacts_sub_ = nh_.subscribe(
    "/robot_bumper", 1, &breakoutGazeboPlugin::contactsCallback, this);
}


void breakoutGazeboPlugin::spawnModel(
  std::string model_name, ignition::math::Pose3d pose, int tag) {
  // open the model file and save it to a string
  std::string model_string, line, model_path;
  sdf::SDF sdf;
  sdf::ElementPtr sdf_model;

  // fill sdf
  model_path = ros::package::getPath("breakout_gazebo") + "/models/" +
    model_name + "/model.sdf";

  std::ifstream model_file(model_path.c_str());

  while (std::getline(model_file, line))
    model_string += line + "\n";

  sdf.SetFromString(model_string);
  sdf_model = sdf.Root()->GetElement("model");

  // set pose
  sdf_model->GetAttribute("name")->Set(model_name + std::to_string(tag));
  sdf_model->GetElement("pose")->Set(pose);

  // spawn model
  world_->InsertModelSDF(sdf);
}

std::string getLiteralWord(int digit) {
  switch (digit) {
    case 0:
      return "zero";
    case 1:
      return "one";
    case 2:
      return "two";
    case 3:
      return "three";
    case 4:
      return "four";
    case 5:
      return "five";
    case 6:
      return "six";
    case 7:
      return "seven";
    case 8:
      return "eight";
    case 9:
      return "nine";
    default:
      return "zero";
  }
}


void breakoutGazeboPlugin::updateScore() {
  score_++;
  ROS_INFO("Score: %d", score_);

  // extract three digits from the score
  int hundreds = score_ / 100;
  int tens = (score_ / 10) % 10;
  int ones = score_ % 10;

  // spawn the score models
  std::string curr_hundreds_model = getLiteralWord(hundreds);
  std::string curr_tens_model = getLiteralWord(tens);
  std::string curr_ones_model = getLiteralWord(ones);

  // if the current models are different from the previous models
  if (curr_hundreds_model + "0" != prev_hundreds_model) {
    deleteModel(prev_hundreds_model);
    spawnModel(curr_hundreds_model, hundreds_pose_, 0);
    prev_hundreds_model = curr_hundreds_model + "0";
  }

  if (curr_tens_model + "1" != prev_tens_model) {
    deleteModel(prev_tens_model);
    spawnModel(curr_tens_model, tens_pose_, 1);
    prev_tens_model = curr_tens_model + "1";
  }
  // the ones place model is always different
  // dont spawn new model if the previous model still exists
  for (int i = 0; i < 10; i++) {
    std::string model_name = getLiteralWord(i) + "2";
    if (world_->ModelByName(model_name)) {
      deleteModel(model_name);
    }
  }
  spawnModel(curr_ones_model, ones_pose_, 2);
  prev_ones_model = curr_ones_model + "2";

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

      // check if there are remaining ones model that is not prev_ones_model
      for (int i = 0; i < 10; i++) {
        std::string model_name = getLiteralWord(i) + "2";
        if (world_->ModelByName(model_name) && model_name != prev_ones_model) {
          deleteModel(model_name);
        }
      }
      updateScore();
    }
  }
}

void breakoutGazeboPlugin::deleteModel(std::string model_name) {
  gazebo::physics::ModelPtr model = world_->ModelByName(model_name);
  world_->RemoveModel(model);
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
