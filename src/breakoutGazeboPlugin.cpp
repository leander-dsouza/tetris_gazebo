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


// make a collection of bash colours and their appropriate hex values
std::map<std::string, std::string> bash_colours = {
  {"black", "\033[0;30m"}, {"red", "\033[0;31m"}, {"green", "\033[0;32m"},
  {"yellow", "\033[0;33m"}, {"blue", "\033[0;34m"}, {"magenta", "\033[0;35m"},
  {"cyan", "\033[0;36m"}, {"white", "\033[0;37m"}, {"reset", "\033[0m"}};

std::string getLiteralWord(int num) {
  switch (num) {
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

void breakoutGazeboPlugin::Init() {
  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init("breakout_gazebo");

  ros::init(
    ros::M_string(), "breakout_gazebo_node",
    ros::init_options::NoSigintHandler);

  ball_speed_x_limit_ = 7.0;
  ball_speed_y_ = -14.0;

  ball_pose_ = ignition::math::Pose3d(0, 0, 17, 0, 0, 0);
  hundreds_pose_ = ignition::math::Pose3d(-8.522, 0, 26.4, 0, 0, 0);
  tens_pose_ = ignition::math::Pose3d(-6.9725, 0, 26.4, 0, 0, 0);
  ones_pose_ = ignition::math::Pose3d(-5.423, 0, 26.4, 0, 0, 0);
  lives_pose_ = ignition::math::Pose3d(2.324, 0, 29.5, 0, 0, 0);
  highscore_hundreds_pose_ = ignition::math::Pose3d(3.8735, 0, 26.4, 0, 0, 0);
  highscore_tens_pose_ = ignition::math::Pose3d(5.423, 0, 26.4, 0, 0, 0);
  highscore_ones_pose_ = ignition::math::Pose3d(6.9725, 0, 26.4, 0, 0, 0);

  contacts_sub_ = nh_.subscribe(
    "/robot_bumper", 1, &breakoutGazeboPlugin::contactsCallback, this);
}


void breakoutGazeboPlugin::spawnModel(
  std::string model_name, ignition::math::Pose3d pose, std::string tag) {
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
  sdf_model->GetAttribute("name")->Set(model_name + tag);
  sdf_model->GetElement("pose")->Set(pose);

  // spawn model
  world_->InsertModelSDF(sdf);
}

void breakoutGazeboPlugin::updateHighScore() {
  // extract three digits from the highscore
  int hundreds = highscore_ / 100;
  int tens = (highscore_ / 10) % 10;
  int ones = highscore_ % 10;

  // spawn the highscore models
  std::string curr_hundreds_model = getLiteralWord(hundreds);
  std::string curr_tens_model = getLiteralWord(tens);
  std::string curr_ones_model = getLiteralWord(ones);

  // if the current models are different from the previous models
  if (curr_hundreds_model + "3" != prev_highscore_hundreds_model_) {
    deleteModel(prev_highscore_hundreds_model_);
    spawnModel(curr_hundreds_model, highscore_hundreds_pose_, "3");
    prev_highscore_hundreds_model_ = curr_hundreds_model + "3";
  }

  if (curr_tens_model + "4" != prev_highscore_tens_model_) {
    deleteModel(prev_highscore_tens_model_);
    spawnModel(curr_tens_model, highscore_tens_pose_, "4");
    prev_highscore_tens_model_ = curr_tens_model + "4";
  }

  // check if there are remaining are that's not prev_highscore_ones_model_
  for (int i = 0; i < 10; i++) {
    std::string model_name = getLiteralWord(i) + "5";
    if (world_->ModelByName(model_name) &&
      model_name != prev_highscore_ones_model_) {
      deleteModel(model_name);
    }
  }

  if (curr_ones_model + "5" != prev_highscore_ones_model_) {
    deleteModel(prev_highscore_ones_model_);
    spawnModel(curr_ones_model, highscore_ones_pose_, "5");
    prev_highscore_ones_model_ = curr_ones_model + "5";
  }
}

void breakoutGazeboPlugin::respawnBricks() {
  // delete all the bricks
  std::vector<std::string> colours = {"yellow", "green", "orange", "red"};
  int row_counter = 0;

  // spawn the bricks
  for (int i = 0; i < 4; i++) {
    for (int j = 1; j <= 28; j++) {
      std::string model_name = colours[i] + "_brick";

      // spawn the brick if it is not in scene
      if (world_->ModelByName(model_name + "_" + std::to_string(j))) {
        continue;
      }

      if (j > 14) {
        row_counter = i + 1;
      } else {
        row_counter = i + 2;
      }

      ignition::math::Pose3d pose(
        -11.621 + j * 1.5495, 0, 19.45 + 0.55 * row_counter, 0, 0, 0);
      spawnModel(model_name, pose, "_" + std::to_string(j));
    }
  }
}

void breakoutGazeboPlugin::updateScore() {
  if (score_ > highscore_) {
    highscore_ = score_;
    updateHighScore();
  }

  if (score_ == 448) {
    respawnBricks();
    respawnBall(true);
    ROS_INFO_STREAM(
      bash_colours["yellow"] << "Level 2" << bash_colours["reset"]);
    hit_counter_ = 0;
    red_hit_ = false;
    orange_hit_ = false;
  }

  if (score_ == 896) {
    ROS_INFO_STREAM(
      bash_colours["green"] << "You Win!" << bash_colours["reset"]);
    respawnBricks();
    lives_ = 0;
    score_ = -1;
    updateLives();
    updateScore();
    respawnBall(true);
    hit_counter_ = 0;
    red_hit_ = false;
    orange_hit_ = false;
  }

  // extract three digits from the score
  int hundreds = score_ / 100;
  int tens = (score_ / 10) % 10;
  int ones = score_ % 10;

  // spawn the score models
  std::string curr_hundreds_model = getLiteralWord(hundreds);
  std::string curr_tens_model = getLiteralWord(tens);
  std::string curr_ones_model = getLiteralWord(ones);

  // if the current models are different from the previous models
  if (curr_hundreds_model + "0" != prev_hundreds_model_) {
    deleteModel(prev_hundreds_model_);
    spawnModel(curr_hundreds_model, hundreds_pose_, "0");
    prev_hundreds_model_ = curr_hundreds_model + "0";
  }

  if (curr_tens_model + "1" != prev_tens_model_) {
    deleteModel(prev_tens_model_);
    spawnModel(curr_tens_model, tens_pose_, "1");
    prev_tens_model_ = curr_tens_model + "1";
  }

  // check if there are remaining ones model that is not prev_ones_model
  for (int i = 0; i < 10; i++) {
    std::string model_name = getLiteralWord(i) + "2";
    if (world_->ModelByName(model_name) && model_name != prev_ones_model_) {
      deleteModel(model_name);
    }
  }

  if (curr_ones_model + "2" != prev_ones_model_) {
    deleteModel(prev_ones_model_);
    spawnModel(curr_ones_model, ones_pose_, "2");
    prev_ones_model_ = curr_ones_model + "2";
  }
}

void breakoutGazeboPlugin::respawnBall(bool wait) {
  gazebo::physics::ModelPtr ball_model = world_->ModelByName("bouncy_ball");
  gazebo::physics::LinkPtr ballLink = ball_model->GetLink("ball_link");

  // stop the ball for 2 seconds
  ignition::math::Vector3d zeroVelocity(0.0, 0.0, 0.0);
  ballLink->SetLinearVel(zeroVelocity);
  ballLink->SetAngularVel(zeroVelocity);
  if (wait) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  // set the ball to the starting position
  ballLink->SetWorldPose(ball_pose_);

  // set random initial velocity but ensure magnitude is constant
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(
    -ball_speed_x_limit_, ball_speed_x_limit_);

  // set initial velocity
  float ball_speed_x = distribution(generator);

  ignition::math::Vector3d initialVelocity(ball_speed_x, 0.0, ball_speed_y_);
  ballLink->SetLinearVel(initialVelocity);
}

void breakoutGazeboPlugin::updateLives() {
  bool wait = false;
  lives_ += 1;
  ROS_INFO_STREAM(
    bash_colours["red"] << "Lives: " << lives_ << bash_colours["reset"]);

  if (lives_ == 4) {
    respawnBricks();
    ROS_INFO_STREAM(
      bash_colours["red"] << "Game Over" << bash_colours["reset"]);
    lives_ = 0;
    score_ = 0;
    updateLives();
    updateScore();
    hit_counter_ = 0;
    red_hit_ = false;
    orange_hit_ = false;
    wait = true;
  }

  // check if there are remaining ones model that is not prev_lives_model
  for (int i = 1; i < 4; i++) {
    std::string model_name = getLiteralWord(i) + "7";
    if (world_->ModelByName(model_name) && model_name != prev_lives_model_) {
      deleteModel(model_name);
    }
  }

  // delete the previous lives model
  deleteModel(prev_lives_model_);

  // spawn the current lives model
  std::string curr_lives_model = getLiteralWord(lives_);
  spawnModel(curr_lives_model, lives_pose_, "7");

  // update the previous lives model
  prev_lives_model_ = curr_lives_model + "7";

  // respawn the ball
  respawnBall(wait);
}

void breakoutGazeboPlugin::increaseBallSpeed(float factor) {
  gazebo::physics::ModelPtr ball_model = world_->ModelByName("bouncy_ball");
  gazebo::physics::LinkPtr ballLink = ball_model->GetLink("ball_link");

  ignition::math::Vector3d currentVelocity = ballLink->WorldLinearVel();
  ignition::math::Vector3d newVelocity(
    currentVelocity.X() * (1 + factor), currentVelocity.Y() * (1 + factor),
    currentVelocity.Z());
  ballLink->SetLinearVel(newVelocity);
}

void breakoutGazeboPlugin::contactsCallback(
  const gazebo_msgs::ContactsState::ConstPtr& msg) {
  if (msg->states.size() > 0) {
    // if the ball collides with bottom wall, move it to the starting position
    if (msg->states[0].collision2_name == "breakout_base::link::collision") {
      updateLives();
    }

    // delete model if collision2_name has the name brick in it
    if (msg->states[0].collision2_name.find("brick") != std::string::npos) {
      std::string model_name = msg->states[0].collision2_name;
      // remove ::base_link::collision from the model name
      model_name = model_name.substr(0, model_name.find("::"));
      deleteModel(model_name);

      // .............. Ball speed increment logic
      hit_counter_ += 1;

      if (hit_counter_ == 4) {
        // increase the speed of the ball by 50%
        increaseBallSpeed(0.5);
        ROS_INFO_STREAM(
          bash_colours["green"] << "Speed increased by 50 percent" <<
          bash_colours["reset"]);
      }
      if (hit_counter_ == 12) {
        // increase the speed of the ball by 50%
        increaseBallSpeed(0.5);
        ROS_INFO_STREAM(
          bash_colours["green"] << "Speed increased by 50 percent" <<
          bash_colours["reset"]);
      }
      if (model_name.find("orange") != std::string::npos && !orange_hit_) {
        orange_hit_ = true;
        // increase the speed of the ball by 50%
        increaseBallSpeed(0.5);
        ROS_INFO_STREAM(
          bash_colours["green"] << "Speed increased by 50 percent" <<
          bash_colours["reset"]);
      }
      if (model_name.find("red") != std::string::npos && !red_hit_) {
        red_hit_ = true;
        // increase the speed of the ball by 50%
        increaseBallSpeed(0.5);
        ROS_INFO_STREAM(
          bash_colours["green"] << "Speed increased by 50 percent" <<
          bash_colours["reset"]);
      }
      // ............................................

      if (model_name.find("yellow") != std::string::npos) {
        score_ += 1;
      } else if (model_name.find("green") != std::string::npos) {
        score_ += 3;
      } else if (model_name.find("orange") != std::string::npos) {
        score_ += 5;
      } else if (model_name.find("red") != std::string::npos) {
        score_ += 7;
      }
      updateScore();
    }
  }
}

void breakoutGazeboPlugin::deleteModel(std::string model_name) {
  gazebo::physics::ModelPtr model = world_->ModelByName(model_name);
  world_->RemoveModel(model);
}

void breakoutGazeboPlugin::Load(
  gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  world_ = _parent;
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&breakoutGazeboPlugin::OnUpdate, this));
}

void breakoutGazeboPlugin::OnUpdate() {
  if (!start) {
    respawnBall(false);
    start = true;
  }
}


GZ_REGISTER_WORLD_PLUGIN(breakoutGazeboPlugin)
