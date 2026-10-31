// Pure transport adapter: publish the five actual native protobuf messages as JSON.
#include <memory>
#include <mutex>
#include <string>
#include <cstdint>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <gazebo/sensors/AltimeterSensor.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo/transport/transport.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo
{
class GazeboTeachL20SensorBridge final : public SensorPlugin
{
  struct State
  {
    std::mutex mutex;
    bool active = true;
    uint64_t sequence = 0;
    std::string kind, topic;
    physics::WorldPtr world;
    ros::Publisher publisher;
  };

  template<typename Message>
  transport::SubscriberPtr Subscribe(const std::shared_ptr<State> &state)
  {
    state->topic = node_->DecodeTopicName(state->topic);
    return node_->Subscribe<Message>(state->topic,
      [state](const boost::shared_ptr<const Message> &native)
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->active || !ros::ok()) return;
        google::protobuf::util::JsonPrintOptions options;
        options.preserve_proto_field_names = true;
        std::string native_json;
        auto status = google::protobuf::util::MessageToJsonString(*native, &native_json, options);
        if (!status.ok()) {gzerr << status.ToString() << std::endl; return;}
        google::protobuf::Struct envelope;
        auto &fields = *envelope.mutable_fields();
        fields["source_kind"].set_string_value(state->kind);
        fields["native_topic"].set_string_value(state->topic);
        fields["native_type"].set_string_value(native->GetDescriptor()->full_name());
        fields["bridge_sequence"].set_string_value(std::to_string(++state->sequence));
        const auto receipt = state->world->SimTime();
        fields["receipt_world_time_ns"].set_string_value(std::to_string(int64_t(receipt.sec)*1000000000LL+receipt.nsec));
        // LogicalCameraImage genuinely has no native time field. Keep null;
        // the envelope receipt time/sequence must never be called sampling time.
        const auto time_field = native->GetDescriptor()->FindFieldByName("time");
        if (time_field && native->GetReflection()->HasField(*native, time_field))
        {
          const auto &stamp = native->GetReflection()->GetMessage(*native, time_field);
          const auto descriptor = stamp.GetDescriptor();
          const auto reflection = stamp.GetReflection();
          const int64_t sec = reflection->GetInt32(stamp, descriptor->FindFieldByName("sec"));
          const int64_t nsec = reflection->GetInt32(stamp, descriptor->FindFieldByName("nsec"));
          fields["source_stamp_ns"].set_string_value(std::to_string(sec*1000000000LL+nsec));
        }
        else fields["source_stamp_ns"].set_null_value(google::protobuf::NULL_VALUE);
        status = google::protobuf::util::JsonStringToMessage(native_json, fields["native"].mutable_struct_value());
        if (!status.ok()) {gzerr << status.ToString() << std::endl; return;}
        std_msgs::String message;
        status = google::protobuf::util::MessageToJsonString(envelope, &message.data, options);
        if (status.ok()) state->publisher.publish(message);
        else gzerr << status.ToString() << std::endl;
      });
  }

  public: ~GazeboTeachL20SensorBridge() override
  {
    if (state_) {std::lock_guard<std::mutex> lock(state_->mutex); state_->active = false;}
    subscription_.reset();
    if (node_) node_->Fini();
    node_.reset();
    if (state_) {std::lock_guard<std::mutex> lock(state_->mutex); state_->publisher.shutdown();}
    ros_node_.reset();
    state_.reset();
  }

  public: void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override
  {
    if (!ros::isInitialized()) gzthrow("L20 sensor adapter requires the Gazebo ROS API system plugin");
    state_ = std::make_shared<State>();
    state_->kind = sdf->Get<std::string>("sensorKind");
    state_->world = physics::get_world(parent->WorldName());
    ros_node_.reset(new ros::NodeHandle(sdf->Get<std::string>("robotNamespace")));
    state_->publisher = ros_node_->advertise<std_msgs::String>(sdf->Get<std::string>("topicName"), 20);
    node_.reset(new transport::Node());
    node_->Init(parent->WorldName());
    if (state_->kind == "gps" && std::dynamic_pointer_cast<sensors::GpsSensor>(parent))
    {
      // GpsSensor has no Topic override; its producer concatenates its explicit
      // SDF topic suffix "data" with parent/name. Mirror that routing only.
      state_->topic = "~/"+parent->ParentName()+"/"+parent->Name()+"/data";
      for (size_t pos = 0; (pos = state_->topic.find("::", pos)) != std::string::npos;)
        state_->topic.replace(pos, 2, "/");
      subscription_ = Subscribe<msgs::GPS>(state_);
    }
    else if (state_->kind == "magnetometer")
    {
      auto sensor = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(parent);
      if (!sensor) gzthrow("L20 magnetometer parent mismatch");
      state_->topic = sensor->GetTopic();
      subscription_ = Subscribe<msgs::Magnetometer>(state_);
    }
    else if (state_->kind == "altimeter" && std::dynamic_pointer_cast<sensors::AltimeterSensor>(parent))
    {
      // This Classic producer uses Topic(), not its legacy GetTopic override.
      // The personal SDF therefore supplies a fully scoped explicit topic.
      state_->topic = parent->Topic();
      subscription_ = Subscribe<msgs::Altimeter>(state_);
    }
    else if (state_->kind == "sonar" && std::dynamic_pointer_cast<sensors::SonarSensor>(parent))
    {
      state_->topic = parent->Topic();
      subscription_ = Subscribe<msgs::SonarStamped>(state_);
    }
    else if (state_->kind == "logical_camera" && std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(parent))
    {
      state_->topic = parent->Topic();
      subscription_ = Subscribe<msgs::LogicalCameraImage>(state_);
    }
    else gzthrow("L20 unsupported native sensor kind or parent factory");
    parent->SetActive(true);
    gzmsg << "L20 actual " << state_->kind << " " << node_->DecodeTopicName(state_->topic)
          << " -> " << sdf->Get<std::string>("topicName") << std::endl;
  }

  private: std::shared_ptr<State> state_;
  private: std::unique_ptr<ros::NodeHandle> ros_node_;
  private: transport::NodePtr node_;
  private: transport::SubscriberPtr subscription_;
};
GZ_REGISTER_SENSOR_PLUGIN(GazeboTeachL20SensorBridge)
}
