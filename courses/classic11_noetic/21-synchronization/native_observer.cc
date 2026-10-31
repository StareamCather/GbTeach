// Observe real native messages. CLOCK_MONOTONIC marks observer receipt, not capture.
#include <atomic>
#include <csignal>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <boost/crc.hpp>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

std::atomic<bool> running{true};
std::mutex output_mutex;
uint64_t sequence = 0;
void Stop(int) {running.store(false);}
int64_t Monotonic()
{
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec)*1000000000LL+ts.tv_nsec;
}

template<typename Message>
void Emit(const std::string &source, const std::string &topic,
          const boost::shared_ptr<const Message> &message)
{
  const auto receipt = Monotonic();
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string native_json;
  if (!google::protobuf::util::MessageToJsonString(*message, &native_json, options).ok()) return;
  google::protobuf::Struct envelope;
  auto &f = *envelope.mutable_fields();
  f["source"].set_string_value(source);
  f["native_topic"].set_string_value(topic);
  f["native_type"].set_string_value(message->GetDescriptor()->full_name());
  f["observer_receive_monotonic_ns"].set_string_value(std::to_string(receipt));
  if (!google::protobuf::util::JsonStringToMessage(native_json, f["native"].mutable_struct_value()).ok()) return;
  // Camera bytes remain the original base64 protobuf field, with no image synthesis.
  const std::string field = source == "imu" ? "stamp" : source == "world_stats" ? "sim_time" : "time";
  const auto descriptor = message->GetDescriptor();
  const auto time_field = descriptor->FindFieldByName(field);
  if (time_field && message->GetReflection()->HasField(*message, time_field))
  {
    const auto &stamp = message->GetReflection()->GetMessage(*message, time_field);
    auto d = stamp.GetDescriptor();
    auto r = stamp.GetReflection();
    const int64_t sec = r->GetInt32(stamp, d->FindFieldByName("sec"));
    const int64_t nsec = r->GetInt32(stamp, d->FindFieldByName("nsec"));
    f["source_stamp_ns"].set_string_value(std::to_string(sec*1000000000LL+nsec));
  }
  else f["source_stamp_ns"].set_null_value(google::protobuf::NULL_VALUE);
  f["time_basis"].set_string_value(source == "world_stats" ? "world_statistics_sim_time" :
      source == "camera" || source == "imu" ? "native_sensor_stamp" : "no_native_stamp");
  std::lock_guard<std::mutex> lock(output_mutex);
  f["observer_sequence"].set_string_value(std::to_string(++sequence));
  std::string line;
  if (google::protobuf::util::MessageToJsonString(envelope, &line, options).ok())
    std::cout << line << std::endl;
}

int main()
{
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::signal(SIGHUP, Stop);
  if (!gazebo::transport::init()) return 2;
  gazebo::transport::run();
  gazebo::transport::NodePtr node(new gazebo::transport::Node);
  node->Init("lesson21");
  std::vector<gazebo::transport::SubscriberPtr> subscriptions;
  const std::string camera = node->DecodeTopicName("~/sync_rig/link/camera/image");
  const std::string imu = node->DecodeTopicName("~/sync_rig/link/imu/native");
  const std::string stats = node->DecodeTopicName("~/world_stats");
  const std::string metadata = node->DecodeTopicName("~/lesson21/runtime_metadata");
  const std::string performance = "/gazebo/performance_metrics";
  subscriptions.push_back(node->Subscribe<gazebo::msgs::ImageStamped>(camera, [camera](const boost::shared_ptr<const gazebo::msgs::ImageStamped> &m) {Emit("camera", camera, m);}));
  subscriptions.push_back(node->Subscribe<gazebo::msgs::IMU>(imu, [imu](const boost::shared_ptr<const gazebo::msgs::IMU> &m) {Emit("imu", imu, m);}));
  subscriptions.push_back(node->Subscribe<gazebo::msgs::WorldStatistics>(stats, [stats](const boost::shared_ptr<const gazebo::msgs::WorldStatistics> &m) {Emit("world_stats", stats, m);}));
  subscriptions.push_back(node->Subscribe<gazebo::msgs::PerformanceMetrics>(performance, [performance](const boost::shared_ptr<const gazebo::msgs::PerformanceMetrics> &m) {Emit("performance_metrics", performance, m);}));
  subscriptions.push_back(node->Subscribe<gazebo::msgs::Any>(metadata, [metadata](const boost::shared_ptr<const gazebo::msgs::Any> &m) {Emit("metadata", metadata, m);}));
  while (running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  subscriptions.clear();
  node->Fini();
  node.reset();
  gazebo::transport::fini();
  return 0;
}
