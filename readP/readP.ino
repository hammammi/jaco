
#include "AD7705.h"

const int ss = 48;
const int ss2 = 49;
const double vref = 5.0;

double v;
double v1;


AD7705 ad7705_1(ss,vref);
AD7705 ad7705_2(ss2,vref);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
//     pinMode(ss, OUTPUT);
//
//    digitalWrite(ss, HIGH);


  ad7705_1.reset();
  ad7705_1.init(AD7705::CHN_AIN1);
  ad7705_1.init(AD7705::CHN_AIN2);

  ad7705_2.reset();
  ad7705_2.init(AD7705::CHN_AIN1);
  ad7705_2.init(AD7705::CHN_AIN2);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  v = ad7705_1.readValue(AD7705::CHN_AIN1);
  
  Serial.print(" 1 ");
  Serial.print(v);

    
  v1 = ad7705_1.readValue(AD7705::CHN_AIN2);
  
  Serial.print(" : ");
  Serial.println(v1);
  delay(500);
//
  
  v = ad7705_2.readValue(AD7705::CHN_AIN2);
//  
  Serial.print(" 2 ");
  Serial.println(v);
//
////  v = ad7705_1.readADResult(AD7705::CHN_AIN2);
//  
//  v1 = ad7705_2.readValue(AD7705::CHN_AIN2);
//  Serial.print(" : ");
//  Serial.println(v1);

}
