// GazeboTeach L15: a transport adapter, not a range simulator.
// The stock Noetic CPU/GPU libraries both export gazebo::GazeboRosLaser.
// Keep this class unique: https://github.com/ros-simulation/gazebo_ros_pkgs/issues/768
#include <memory>
#include <mutex>
#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace gazebo
{
class GazeboTeachL15GpuLaser final : public SensorPlugin
{
  struct State
  {
    std::mutex mutex;
    bool active = true;
    std::string frame;
    ros::Publisher publisher;
  };

  public: ~GazeboTeachL15GpuLaser() override
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
    auto sensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(parent);
    if (!sensor) gzthrow("GazeboTeachL15GpuLaser requires an actual GpuRaySensor");
    if (!ros::isInitialized()) gzthrow("Load libgazebo_ros_api_plugin.so before the L15 GPU adapter");
    state_ = std::make_shared<State>();
    state_->frame = sdf->Get<std::string>("frameName");
    ros_node_.reset(new ros::NodeHandle(sdf->Get<std::string>("robotNamespace")));
    state_->publisher = ros_node_->advertise<sensor_msgs::LaserScan>(sdf->Get<std::string>("topicName"), 10);
    node_.reset(new transport::Node());
    node_->Init(parent->WorldName());
    // The callback owns only shared state, so a late transport delivery cannot
    // dereference this plugin after unloading. The stop barrier covers publish.
    const auto state = state_;
    subscription_ = node_->Subscribe<msgs::LaserScanStamped>(sensor->Topic(),
      [state](const boost::shared_ptr<const msgs::LaserScanStamped> &native)
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->active || !ros::ok()) return;
        const auto &scan = native->scan();
        sensor_msgs::LaserScan message;
        message.header.stamp = ros::Time(native->time().sec(), native->time().nsec());
        message.header.frame_id = state->frame;  // label only; no TF broadcast
        message.angle_min = scan.angle_min();
        message.angle_max = scan.angle_max();
        message.angle_increment = scan.angle_step();
        message.range_min = scan.range_min();
        message.range_max = scan.range_max();
        message.time_increment = 0;  // native instantaneous simulated scan
        message.scan_time = 0;       // retain stock Noetic wrapper convention
        message.ranges.assign(scan.ranges().begin(), scan.ranges().end());
        message.intensities.assign(scan.intensities().begin(), scan.intensities().end());
        state->publisher.publish(message);
      });
    sensor->SetActive(true);
    gzmsg << "GazeboTeach L15 unique GPU adapter: " << sensor->Topic() << " -> "
          << sdf->Get<std::string>("topicName") << std::endl;
  }

  private: std::shared_ptr<State> state_;
  private: std::unique_ptr<ros::NodeHandle> ros_node_;
  private: transport::NodePtr node_;
  private: transport::SubscriberPtr subscription_;
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboTeachL15GpuLaser)
}
