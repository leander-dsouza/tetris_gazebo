# breakout_gazebo
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)
![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)

A ROS1 package to simulate the popular Atari's [Breakout](https://en.wikipedia.org/wiki/Breakout_(video_game)) arcade game in Gazebo.

<p align=center>
<img src="https://user-images.githubusercontent.com/45683974/236690701-362b65d1-8e90-448b-912d-c8eba77f547a.gif"/>
</p>

## Installation

* Install python dependencies using pip3:

	```bash
	pip3 install -r requirements.txt
	```

* Finally, install all the dependencies using `rosdep`:

	```bash
	rosdep install --from-paths $ROS_WS/src --ignore-src -r -y
	```

* Build the workspace:

	```bash
	catkin build breakout_gazebo
	```

## Usage

* Launch the game and control the paddle using arrow keys (left and right), after activating the simulation using the space bar:

	```bash
	roslaunch breakout_gazebo breakout.launch
	```

## Gameplay Instructions

* There are two screens to the game as the original Breakout game. The second screen is unlocked after clearing the first screen (reaching 448 points).
* There are 3 lives in total. The game is reset after losing all lives.
* The game is won after clearing both screens and is reset after that.
* The scoring metric is as follows:
	* Each green brick is worth 1 point.
	* Each yellow brick is worth 3 points.
	* Each orange brick is worth 5 points.
	* Each red brick is worth 7 points.
* The speed of the ball is increased by 1.5x its original speed whenever:
	* The ball gets 4 hits on the bricks.
	* The ball gets 12 hits on the bricks.
	* When the ball breaks through the orange bricks.
	* When the ball breaks through the red bricks.


## Standardization

* The entire package is formatted against [roslint](http://wiki.ros.org/roslint) and [catkin_lint](https://github.com/fkie/catkin_lint).

###### 💾 EOF
