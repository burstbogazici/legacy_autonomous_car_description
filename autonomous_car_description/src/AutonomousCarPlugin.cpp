#include "AutonomousCarPlugin.hh"

#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <string>

namespace gazebo {

  class AutonomousCarPrivate {

    public: std::unique_ptr<ros::NodeHandle> nodeHandle;
    public: ros::Subscriber controlSubscriber;

    public: ros::CallbackQueue rosQueue;
    public: boost::thread rosQueueThread;

    public: physics::WorldPtr world;
    public: physics::ModelPtr model;
    public: sdf::ElementPtr sdf;

    public: std::string robotNamespaceName;

    public: std::string commandTopicName;
    public: std::string odometryTopicName;
    public: std::string odometryFrameName;
    public: std::string robotBaseFrameName;

    public: bool publishWheelTFBool;
    public: bool publishWheelJointStateBool;

    public: std::string rearLeftWheelJointName;
    public: std::string rearRightWheelJointName;
    public: std::string frontLeftWheelJointName;
    public: std::string frontRightWheelJointName;

    public: std::string frontLeftHingeJointName;
    public: std::string frontRightHingeJointName;

    public: physics::JointPtr frontLeftWheelJoint;
    public: physics::JointPtr frontRightWheelJoint;
    public: physics::JointPtr rearLeftWheelJoint;
    public: physics::JointPtr rearRightWheelJoint;

    public: physics::JointPtr frontLeftWheelSteeringJoint;
    public: physics::JointPtr frontRightWheelSteeringJoint;

    public: common::PID frontLeftWheelSteeringPID;
    public: common::PID frontRightWheelSteeringPID;

    public: common::Time lastMessageTime;
    public: common::Time lastSimulationTime;

    public: double maxSpeed = 0;
    public: double maxSteeringAngle = 0;

    public: double wheelSpeedCommand = 0;
    public: double frontLeftWheelSteeringCommand = 0;
    public: double frontRightWheelSteeringCommand = 0;

    public: double frontLeftWheelRadius = 0;
    public: double frontRightWheelRadius = 0;
    public: double rearLeftWheelRadius = 0;
    public: double rearRightWheelRadius = 0;

    public: double frontLeftJointFriction = 0;
    public: double frontRightJointFriction = 0;
    public: double rearLeftJointFriction = 0;
    public: double rearRightJointFriction = 0;

    public: double wheelbaseLength = 0;
    public: double frontTrackWidth = 0;
    public: double rearTrackWidth = 0;

    public: double brakePedalPercent = 0;
    public: double frontLeftSteeringAngle = 0;
    public: double frontRightSteeringAngle = 0;

    public: double frontLeftWheelAngularVelocity = 0;
    public: double frontRightWheelAngularVelocity = 0;
    public: double rearLeftWheelAngularVelocity = 0;
    public: double rearRightWheelAngularVelocity = 0;

    public: std::mutex mutex;

    public: double odom = 0.0;

    public: event::ConnectionPtr updateConnection;
  };

  AutonomousCarPlugin::AutonomousCarPlugin() : dataPointer(new AutonomousCarPrivate) {}

  AutonomousCarPlugin::~AutonomousCarPlugin() {
    this->dataPointer->updateConnection.reset();
  }

  void AutonomousCarPlugin::onTwistCommand(const geometry_msgs::TwistConstPtr& message) {
    //ROS_INFO("TWIST COMMAND RECEIVED!");

    std::lock_guard<std::mutex> lock(this->dataPointer->mutex);

    this->dataPointer->lastMessageTime = this->dataPointer->world->SimTime();

    this->dataPointer->wheelSpeedCommand = message->linear.x;
    this->dataPointer->frontLeftWheelSteeringCommand = message->angular.z;
    this->dataPointer->frontRightWheelSteeringCommand = message->angular.z;
  }

  void AutonomousCarPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    ROS_INFO("Autonomous_Car_Gazebo_Driver_Plugin LOAD STARTED! v1.3");

    AutonomousCarPrivate *dataPointer = this->dataPointer.get();

    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "Autonomous_Car_Driver_Plugin",
      ros::init_options::NoSigintHandler);
    }

    this->dataPointer->nodeHandle.reset(new ros::NodeHandle("Autonomous_Car_Gazebo_Driver_Plugin"));

    this->dataPointer->model = model;
    this->dataPointer->world = this->dataPointer->model->GetWorld();
    this->dataPointer->sdf = sdf;

    this->getParameter<std::string> (this->dataPointer->robotNamespaceName, "robotNamespace", "autonomousCar", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->commandTopicName, "commandTopic", "twistCommand", this->dataPointer->sdf);

    ros::SubscribeOptions subscribeOptions = ros::SubscribeOptions::create<geometry_msgs::Twist>("/" + this->dataPointer->robotNamespaceName + "/" + this->dataPointer->commandTopicName, 1,
      boost::bind(&AutonomousCarPlugin::onTwistCommand, this, _1),
      ros::VoidPtr(), &this->dataPointer->rosQueue);

    this->dataPointer->controlSubscriber = this->dataPointer->nodeHandle->subscribe(subscribeOptions);
    this->dataPointer->rosQueueThread = boost::thread(boost::bind (&AutonomousCarPlugin::QueueThread,this));
    this->getParameter<std::string> (this->dataPointer->odometryTopicName, "odometryTopic", "odom", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->odometryFrameName, "odometryFrame", "odom", this->dataPointer->sdf);

    this->getParameter<std::string> (this->dataPointer->robotBaseFrameName, "robotBaseFrame", "base_link", this->dataPointer->sdf);
    this->getParameter<bool> (this->dataPointer->publishWheelTFBool, "publishWheelTF", false, this->dataPointer->sdf);
    this->getParameter<bool> (this->dataPointer->publishWheelJointStateBool, "publishWheelJointState", false, this->dataPointer->sdf);

    this->getParameter<std::string> (this->dataPointer->frontLeftWheelJointName, "frontLeftWheelJoint", "front_left_wheel_joint", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->frontRightWheelJointName, "frontRightWheelJoint", "front_right_wheel_joint", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->rearLeftWheelJointName, "rearLeftWheelJoint", "rear_left_wheel_joint", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->rearRightWheelJointName, "rearRightWheelJoint", "rear_right_wheel_joint", this->dataPointer->sdf);

    this->getParameter<std::string> (this->dataPointer->frontLeftHingeJointName, "frontLeftHingeJoint", "front_left_hinge_joint", this->dataPointer->sdf);
    this->getParameter<std::string> (this->dataPointer->frontRightHingeJointName, "frontRightHingeJoint", "front_right_hinge_joint", this->dataPointer->sdf);

    this->getParameter<double> (this->dataPointer->frontTrackWidth, "frontTrackWidth", 0.34, this->dataPointer->sdf);
    this->getParameter<double> (this->dataPointer->rearTrackWidth, "rearTrackWidth", 0.34, this->dataPointer->sdf);

    this->getParameter<double> (this->dataPointer->frontLeftWheelRadius, "frontLeftWheelRadius", 0.300, this->dataPointer->sdf);
    this->getParameter<double> (this->dataPointer->frontRightWheelRadius, "frontRightWheelRadius", 0.300, this->dataPointer->sdf);
    this->getParameter<double> (this->dataPointer->rearLeftWheelRadius, "rearLeftWheelRadius", 0.215, this->dataPointer->sdf);
    this->getParameter<double> (this->dataPointer->rearRightWheelRadius, "rearRightWheelRadius", 0.215, this->dataPointer->sdf);

    this->dataPointer->frontLeftWheelJoint = this->dataPointer->model->GetJoint(this->dataPointer->frontLeftWheelJointName);
    this->dataPointer->frontRightWheelJoint = this->dataPointer->model->GetJoint(this->dataPointer->frontRightWheelJointName);
    this->dataPointer->rearLeftWheelJoint = this->dataPointer->model->GetJoint(this->dataPointer->rearLeftWheelJointName);
    this->dataPointer->rearRightWheelJoint = this->dataPointer->model->GetJoint(this->dataPointer->rearRightWheelJointName);

    this->dataPointer->frontLeftWheelSteeringJoint = this->dataPointer->model->GetJoint(this->dataPointer->frontLeftHingeJointName);
    this->dataPointer->frontRightWheelSteeringJoint = this->dataPointer->model->GetJoint(this->dataPointer->frontRightHingeJointName);

    getParameter<double> (this->dataPointer->maxSpeed, "maxSpeed", 10, this->dataPointer->sdf);
    getParameter<double> (this->dataPointer->maxSteeringAngle, "maxSteeringAngle", 0.5, this->dataPointer->sdf);

    this->dataPointer->frontLeftWheelSteeringPID.SetPGain(10000);
    this->dataPointer->frontLeftWheelSteeringPID.SetIGain(0);
    this->dataPointer->frontLeftWheelSteeringPID.SetDGain(300);

    this->dataPointer->frontRightWheelSteeringPID.SetPGain(10000);
    this->dataPointer->frontRightWheelSteeringPID.SetIGain(0);
    this->dataPointer->frontRightWheelSteeringPID.SetDGain(300);

    this->dataPointer->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&AutonomousCarPlugin::Update, this));

    ROS_INFO("Autonomous_Car_Gazebo_Driver_Plugin LOAD FINISHED!");
  }

  void AutonomousCarPlugin::Reset() {
    this->dataPointer->odom = 0;
    this->dataPointer->frontLeftWheelSteeringPID.Reset();
    this->dataPointer->frontRightWheelSteeringPID.Reset();
    this->dataPointer->lastMessageTime = 0;
    this->dataPointer->lastSimulationTime = 0;
    this->dataPointer->wheelSpeedCommand = 0;
    this->dataPointer->frontLeftWheelSteeringCommand = 0;
    this->dataPointer->frontRightWheelSteeringCommand = 0;
    this->dataPointer->brakePedalPercent = 0;
    this->dataPointer->frontLeftSteeringAngle = 0;
    this->dataPointer->frontRightSteeringAngle = 0;
    this->dataPointer->frontLeftWheelAngularVelocity  = 0;
    this->dataPointer->frontRightWheelAngularVelocity = 0;
    this->dataPointer->rearLeftWheelAngularVelocity = 0;
    this->dataPointer->rearRightWheelAngularVelocity  = 0;
  }

  void AutonomousCarPlugin::Update() {
    AutonomousCarPrivate* dataPointer = this->dataPointer.get();

    std::lock_guard<std::mutex> lock(this->dataPointer->mutex);
    common::Time currentTime = this->dataPointer->world->SimTime();
    double dt = (currentTime - this->dataPointer->lastSimulationTime).Double();
    if (dt < 0) {
      this->Reset();
      return;
    }

    dataPointer->frontLeftSteeringAngle = dataPointer->frontLeftWheelSteeringJoint->Position();
    dataPointer->frontRightSteeringAngle = dataPointer->frontRightWheelSteeringJoint->Position();

    //dataPointer->frontLeftWheelAngularVelocity = dataPointer->frontLeftWheelJoint->GetVelocity(0);
    //dataPointer->frontRightWheelAngularVelocity = dataPointer->frontRightWheelJoint->GetVelocity(0);
    //dataPointer->rearLeftWheelAngularVelocity = dataPointer->rearLeftWheelJoint->GetVelocity(0);
    //dataPointer->rearRightWheelAngularVelocity = dataPointer->rearRightWheelJoint->GetVelocity(0);

    this->dataPointer->lastSimulationTime = currentTime;

    double frontLeftSteeringError = this->dataPointer->frontLeftSteeringAngle - this->dataPointer->frontLeftWheelSteeringCommand;
    double frontLeftSteeringForce = this->dataPointer->frontLeftWheelSteeringPID.Update(frontLeftSteeringError, dt);
    this->dataPointer->frontLeftWheelSteeringJoint->SetForce(0, frontLeftSteeringForce);

    double frontRightSteeringError = this->dataPointer->frontRightSteeringAngle - this->dataPointer->frontRightWheelSteeringCommand;
    double frontRightSteeringForce = this->dataPointer->frontRightWheelSteeringPID.Update(frontRightSteeringError, dt);
    this->dataPointer->frontRightWheelSteeringJoint->SetForce(0, frontRightSteeringForce);

    this->dataPointer->frontLeftWheelJoint->SetVelocity(0,dataPointer->wheelSpeedCommand / dataPointer->frontLeftWheelRadius);
    this->dataPointer->frontRightWheelJoint->SetVelocity(0,dataPointer->wheelSpeedCommand / dataPointer->frontRightWheelRadius);
    this->dataPointer->rearLeftWheelJoint->SetVelocity(0,dataPointer->wheelSpeedCommand / dataPointer->rearLeftWheelRadius);
    this->dataPointer->rearRightWheelJoint->SetVelocity(0,dataPointer->wheelSpeedCommand / dataPointer->rearRightWheelRadius);
  }

  void AutonomousCarPlugin::QueueThread() {
    static const double timeout = 0.02;
    while (this->dataPointer->nodeHandle->ok()) {
        this->dataPointer->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(AutonomousCarPlugin)
}

