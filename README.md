# README

## 使用说明

+ 环境配置及编译，默认拥有外网环境,ubuntu22

```bash
wget http://fishros.com/install -O fishros && . fishros
#选择安装ros2 humble和rosdep

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-asio-cmake-module
sudo apt install ros-humble-foxglove-bridge
sudo apt install libgpiod-dev

#在ros2-framework目录下
pip3 install rosdepc
sudo rosdep init
rosdepc update
rosdep install --from-paths src --ignore-src -r -y

#重启命令行
colcon build

```
