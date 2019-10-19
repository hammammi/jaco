# jaco

sudo chmod a+rw /dev/ttyACM0


--> 여기서 ROS 버전은 본인의 버전과 맞는 것을 사용하도록 한다.

arduino IDE 설치 후

$ sudo apt-get install ros-kinetic-rosserial-arduino
$ sudo apt-get install ros-kinetic-rosserial

$ cd <arduino IDE path>/libraries   --> arduino 폴더 설치된 곳의 libraries 폴더 안을 말함.
($ rm -rf ros_lib   --> 기존에 ros_lib이 설치되어 있었던 경우만)
$ rosrun rosserial_arduino make_libraries.py
  
arduino tutorial을 따라해보기바람...

jaco_msgs package를 ROS workspace folder의 src에 넣고 catkin_make

