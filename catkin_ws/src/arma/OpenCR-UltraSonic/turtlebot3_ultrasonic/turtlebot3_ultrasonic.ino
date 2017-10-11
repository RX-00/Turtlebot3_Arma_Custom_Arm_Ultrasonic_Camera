#include <turtlebot3.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <ArduinoHardware.h>

//#include <turtlebot3_core.h>

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Publisher
*******************************************************************************/
std_msgs::Int32 ultrasonic_distance;
ros::Publisher ultrasonic_dist_pub("ultrasonic_distance", &ultrasonic_distance);

/*******************************************************************************
* Declaration for Ultrasonic Sensor
*******************************************************************************/
#define ECHO_PIN 6
#define TRIG_PIN 7

void ultrasonic_setup(){
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void updateUltrasonicDist(){
  long duration, distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;

  if(distance >= 200 || distance <= 0){
    Serial.println("No object in range");
  }
  else{
    Serial.print(distance);
    Serial.println(" cm");
  }

  //publish the data
  
}

void setup() {
  // Initialize ROS node handle, advertise and subscribe to the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(ultrasonic_dist_pub);

  nh.loginfo("Connected to OpenCR board!");

  //prev_update_time = millis();

  ultrasonic_setup();
  
  pinMode(13, OUTPUT);
  
  SerialBT2.begin(57600);
  
  //setup_end = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  updateUltrasonicDist();
  
  nh.spinOnce();
}
