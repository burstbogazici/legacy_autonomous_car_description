#ifndef AUTONOMOUSCARPLUGIN_HH_
#define AUTONOMOUSCARPLUGIN_HH_

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <geometry_msgs/Twist.h>

namespace gazebo {

class AutonomousCarPrivate;

class AutonomousCarPlugin : public ModelPlugin {

  public: AutonomousCarPlugin();
  public: virtual ~AutonomousCarPlugin();

  private: void onTwistCommand(const geometry_msgs::TwistConstPtr& message);

  public: virtual void Reset();

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private: void Update();

  private: void QueueThread();

  private: std::unique_ptr<AutonomousCarPrivate> dataPointer;

  private:
    template <class T>
    void getParameter(T & value, const char *tagName, const T & defaultValue, sdf::ElementPtr sdf) {
        value = defaultValue;
        if (sdf->HasElement(tagName)) {
          this->getParameter<T> (value, tagName, sdf);
        }
    }

    template <class T>
    void getParameter (T & value, const char *tagName, sdf::ElementPtr sdf) {
      if (sdf->HasElement(tagName)) {
         value = sdf->GetElement (tagName)->Get<T>();
      }
    }
};
}
#endif
