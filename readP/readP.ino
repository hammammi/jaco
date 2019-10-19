
#include <ros.h>
#include "AD7705.h"
#include "jaco_msgs/Gripper.h"

#define intervalR 10

const int ss = 41;
const int ss2 = 45;
const double vref = 5.0;

double v;
double v1;
unsigned long range_timer;

ros::NodeHandle nh;
jaco_msgs::Gripper gripper;
ros::Publisher chatter("gripperJnt", &gripper);

AD7705 ad7705_1(ss,vref);
//AD7705 ad7705_2(ss2,vref);

void setup() {
  
  nh.initNode();
  nh.advertise(chatter);
  
//  Serial.begin(9600);
  SPI.begin();
  
  ad7705_1.reset();
  ad7705_1.init(AD7705::CHN_AIN1);
  ad7705_1.init(AD7705::CHN_AIN2);

//  ad7705_2.reset();
//  ad7705_2.init(AD7705::CHN_AIN1);
//  ad7705_2.init(AD7705::CHN_AIN2);
  
  
//  SPI.begin();
  
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis >= range_timer + intervalR){

    range_timer = currentMillis + intervalR;

    v = ad7705_1.readValue(AD7705::CHN_AIN1);
    gripper.jnt1 = v;
  //  Serial.print(" 1 ");
  //  Serial.print(v);

  //  v = ad7705_1.readADResult(AD7705::CHN_AIN2);
  
    v = ad7705_1.readValue(AD7705::CHN_AIN2);
    gripper.jnt2 = v;
    //  Serial.print(" : ");
  //  Serial.println(v);

    chatter.publish(&gripper);
  }
  
  nh.spinOnce();
  
//
//  v1 = ad7705_2.readValue(AD7705::CHN_AIN1);
//  Serial.print(" 2 ");
//  Serial.print(v1);
//
////  v = ad7705_1.readADResult(AD7705::CHN_AIN2);
//  
//  v1 = ad7705_2.readValue(AD7705::CHN_AIN2);
//  Serial.print(" : ");
//  Serial.println(v1);

}
