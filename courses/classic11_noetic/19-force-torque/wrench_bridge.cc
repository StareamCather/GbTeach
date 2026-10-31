// GazeboTeach L19: copy actual native ForceTorqueSensor output to ROS.
// No forces, torques, poses or analytical sensor values are imposed.
#include <memory>
#include <mutex>
#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{
class GazeboTeachL19WrenchBridge final : public SensorPlugin
{
  struct State
  {
    std::mutex mutex;
    bool active = true;
    std::string frame;
    ros::Publisher publisher;
  };

  public: ~GazeboTeachL19WrenchBridge() override
  {
    if (state_)
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->active = false;
    }
    subscription_.reset();
    if (node_) node_->Fini();
    node_.reset();
    if (state_)
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->publisher.shutdown();
    }
    ros_node_.reset();
    state_.reset();
  }

  public: void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override
  {
    auto sensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(parent);
    if (!sensor) gzthrow("GazeboTeachL19WrenchBridge requires an actual ForceTorqueSensor");
    if (!ros::isInitialized()) gzthrow("Load libgazebo_ros_api_plugin.so before the L19 native wrench adapter");
    state_ = std::make_shared<State>();
    state_->frame = sdf->Get<std::string>("frameName");
    ros_node_.reset(new ros::NodeHandle(sdf->Get<std::string>("robotNamespace")));
    state_->publisher = ros_node_->advertise<geometry_msgs::WrenchStamped>(sdf->Get<std::string>("topicName"), 10);
    node_.reset(new transport::Node());
    node_->Init(parent->WorldName());
    // The callback owns only shared state, so a late transport delivery cannot
    // dereference this plugin after unloading. The stop barrier covers publish.
    const auto state = state_;
    subscription_ = node_->Subscribe<msgs::WrenchStamped>(sensor->Topic(),
      [state](const boost::shared_ptr<const msgs::WrenchStamped> &native)
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->active || !ros::ok()) return;
        geometry_msgs::WrenchStamped message;
        message.header.stamp = ros::Time(native->time().sec(), native->time().nsec());
        message.header.frame_id = state->frame;  // basis label only; no TF broadcast
        const auto &wrench = native->wrench();
        message.wrench.force.x = wrench.force().x();
        message.wrench.force.y = wrench.force().y();
        message.wrench.force.z = wrench.force().z();
        message.wrench.torque.x = wrench.torque().x();
        message.wrench.torque.y = wrench.torque().y();
        message.wrench.torque.z = wrench.torque().z();
        state->publisher.publish(message);
      });
    sensor->SetActive(true);
    gzmsg << "GazeboTeach L19 native wrench adapter: " << sensor->Topic() << " -> "
          << sdf->Get<std::string>("topicName") << std::endl;
  }

  private: std::shared_ptr<State> state_;
  private: std::unique_ptr<ros::NodeHandle> ros_node_;
  private: transport::NodePtr node_;
  private: transport::SubscriberPtr subscription_;
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboTeachL19WrenchBridge)
}
