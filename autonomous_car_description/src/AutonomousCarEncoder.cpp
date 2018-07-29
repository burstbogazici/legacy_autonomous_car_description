#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"

#include <math.h>
#include <string>
#include <vector>

ros::Time messageTimestamp;

float encoderOfFrontLeftWheel = 0;
float encoderOfFrontRightWheel = 0;
float encoderOfRearLeftWheel = 0;
float encoderOfRearRightWheel = 0;

int getIndex(std::vector<std::string> names, std::string name) {
  for(size_t index = 0; index < name.size(); index++) {
    if(name.compare(names.at(index)) == 0) {
      return index;
    }
  }
  return -1;
}

void callbackSubscriber(const sensor_msgs::JointStateConstPtr& message) {
  messageTimestamp = ros::Time::now();

  int indexOfFrontLeftWheel = getIndex(message->name, "front_left_wheel_joint");
  int indexOfFrontRightWheel = getIndex(message->name, "front_right_wheel_joint");
  int indexOfRearLeftWheel = getIndex(message->name, "rear_right_wheel_joint");
  int indexOfRearRightWheel = getIndex(message->name, "rear_right_wheel_joint");

  if(indexOfFrontLeftWheel != -1 && indexOfFrontRightWheel != -1 && indexOfRearLeftWheel != -1 && indexOfRearRightWheel != -1) {
    encoderOfFrontLeftWheel = message->position[indexOfFrontLeftWheel];
    encoderOfFrontRightWheel = message->position[indexOfFrontLeftWheel];
    encoderOfRearLeftWheel = message->position[indexOfFrontLeftWheel];
    encoderOfRearRightWheel = message->position[indexOfRearRightWheel];

    encoderOfFrontLeftWheel = (encoderOfFrontLeftWheel / (2*M_PI)) * (24);
    encoderOfFrontRightWheel = (encoderOfFrontRightWheel / (2*M_PI)) * (24);
    encoderOfRearLeftWheel = (encoderOfRearLeftWheel / (2*M_PI)) * (24);
    encoderOfRearRightWheel = (encoderOfRearRightWheel / (2*M_PI)) * (24);
  }
}


int main(int argc, char **argv) {
  ros::init(argc,argv,"AutonomousCarEncoder");

  ros::NodeHandle node;

  messageTimestamp = ros::Time::now();

  ros::Subscriber subscriber = node.subscribe<sensor_msgs::JointState>("/autonomous_car/joint_states",1,callbackSubscriber);

  ros::Publisher encoderFrontPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/front",1);
  ros::Publisher encoderRearPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/rear",1);
  ros::Publisher timestampPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/timestamp",1);

  ros::Rate loop(20);
  while(ros::ok()) {
    std_msgs::Int32 encoderFrontMessage;
    encoderFrontMessage.data = (encoderOfFrontLeftWheel + encoderOfFrontRightWheel) / 2;
    encoderFrontPublisher.publish(encoderFrontMessage);

    std_msgs::Int32 encoderRearMessage;
    encoderRearMessage.data = (encoderOfRearLeftWheel + encoderOfRearRightWheel) / 2;
    encoderRearPublisher.publish(encoderRearMessage);

    std_msgs::Int32 timestampMessage;
    timestampMessage.data = (messageTimestamp.sec * 1000) + (messageTimestamp.nsec / 1000000);
    timestampPublisher.publish(encoderRearMessage);

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
