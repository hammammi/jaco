# jaco




--> 여기서 ROS 버전은 본인의 버전과 맞는 것을 사용하도록 한다.

arduino IDE 설치 후

$ sudo apt-get install ros-kinetic-rosserial-arduino
$ sudo apt-get install ros-kinetic-rosserial

  
arduino tutorial을 따라해보기바람...

AD7705 폴더를 arduino libraries 폴더에 넣는다.
(AD770x 라이브러리를 조금 바꾼것이므로 그것을 사용해도 됨. 그러나 올려진 스케치를 그대로 사용하려면 AD7705 권장)

readP 를 arduino MEGA에 업로드한다.

어떤 포트에 연결되었는지 확인 필수.

jaco_msgs package를 ROS workspace folder의 src에 넣고 catkin_make

($ rm -rf ros_lib   --> 기존에 튜토리얼 따라한다고 ros_lib를 설치했었으면 필수)
$ rosrun rosserial_client make_library.py ~/<arduino IDE path>/libraries

(또는 
$ cd <arduino IDE path>/libraries   --> arduino 폴더 설치된 곳의 libraries 폴더 안을 말함.
$ rm -rf ros_lib   --> 기존에 ros_lib이 설치되어 있었던 경우만
$ rosrun rosserial_arduino make_libraries.py .  --> 마지막에 .을 찍어야함 (현재폴더에 설치함을 뜻함))


sudo chmod a+rw /dev/ttyUSB0
