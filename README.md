
# Sawan Individual Project Package
ROS2 package files for Sawan Saurabh's individual project submission for MSc. Robotics, King's College London. 



## Authors

- [Sawan Saurabh](https://www.linkedin.com/in/sawan-saurabh/)
- K number: K23131775

## Installation

* Install [ROS2 humble](https://docs.ros.org/en/humble/Installation.html)

## Dependency

* [RTabMap](https://github.com/introlab/rtabmap_ros)
* robot_localization
Install using CLI: 
```bash
sudo apt install ros-humble-robot-localization 
```
*  fulfil other dependencies by using rosdep (run in ros wokspace root directory):
```bash
rosdep install --from-paths src -y --ignore-src
```

*NOTE: Please follow the installation instructions of dependencies on their respective web pages.
## Run Package

1. Clone the project

```bash
git clone https://github.com/sawansaurabh/indv_prj_pkg.git
```
2. create a ROS2 workspace and place the cloned repo in src folder.

3. For easy launching, you can use the TMUX shell script provided with the Repo. (Adjust the 'WORKSPACE_PATH' in it).
