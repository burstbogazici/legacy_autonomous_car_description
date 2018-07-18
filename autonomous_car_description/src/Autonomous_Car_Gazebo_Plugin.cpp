#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

float currentSpeed = 0;
float currentAngle = 0;

void callbackSubscriber(const geometry_msgs::TwistConstPtr& message) {
    currentSpeed = message->linear.x;
    currentAngle = message->angular.z;
}


int main(int argc, char **argv) {
  ros::init(argc,argv,"Autonomous_Car_Gazebo_Plugin");

  ros::NodeHandle node;

  ros::Subscriber subscriber = node.subscribe<geometry_msgs::Twist>("/autonomous_car/twistCommand",1,callbackSubscriber);

  ros::Publisher frontLeftWheelPublisher = node.advertise<std_msgs::Float64>("/autonomous_car/front_left_wheel_velocity_controller/command",1);
  ros::Publisher frontRightWheelPublisher = node.advertise<std_msgs::Float64>("/autonomous_car/front_right_wheel_velocity_controller/command",1);
  //Publisher rearLeftWheelPublisher = node.advertise<std_msgs::Float64>("/autonomous_car/rear_left_wheel_velocity_controller/command",1000);
  //Publisher rearRightWheelPublisher = node.advertise<std_msgs::Float64>("/autonomous_car/rear_right_wheel_velocity_controller/command",1000);

  ros::Publisher frontLeftHingePublisher = node.advertise<std_msgs::Float64>("/autonomous_car/front_left_hinge_position_controller/command",1);
  ros::Publisher frontRightHingePublisher = node.advertise<std_msgs::Float64>("/autonomous_car/front_right_hinge_position_controller/command",1);

  ros::Rate loop(20);

  while(ros::ok()) {

    std_msgs::Float64 frontLeftWheelSpeedValue;
    frontLeftWheelSpeedValue.data = currentSpeed;
    frontLeftWheelPublisher.publish(frontLeftWheelSpeedValue);

    std_msgs::Float64 frontRightWheelSpeedValue;
    frontRightWheelSpeedValue.data = currentSpeed;
    frontRightWheelPublisher.publish(frontRightWheelSpeedValue);

    std_msgs::Float64 frontLeftHingeAngleValue;
    frontLeftHingeAngleValue.data = currentAngle;
    frontLeftHingePublisher.publish(frontLeftHingeAngleValue);

    std_msgs::Float64 frontRightHingeAngleValue;
    frontRightHingeAngleValue.data = currentAngle;
    frontRightHingePublisher.publish(frontRightHingeAngleValue);

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
