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
#include <gazebo/transport/IOManager.hh>
#include <gazebo/transport/Connection.hh>
#include <gazebo/gazebo_config.h>
#include <dlfcn.h>
#include <climits>
#include <cstdlib>
#include <memory>
#include <sstream>
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

using namespace gazebo;
std::string Q(const std::string &v) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:v) {
    if(c=='"'||c=='\\')out<<'\\';
    if(c=='\n')out<<"\\n";else if(c=='\r')out<<"\\r";
    else if(c=='\t')out<<"\\t";else if(c>=32)out<<c;
  }
  return out.str()+'"';
}
// Compatibility for this exact Classic11 SDK, isolated from teaching plugins.
// The exported data symbol is a PRIVATE ABI: fail before transport init if its
// DSO/version cannot be identified. No process exit or timed sleep replaces join.
class MainThreadIOOwner {
 public:
  MainThreadIOOwner() : thread(std::this_thread::get_id()) {
    if (std::string(GAZEBO_VERSION_FULL)!="11.15.1") throw std::runtime_error("IO guard private ABI is validated only on Classic11.15.1 SDK");
    dlerror();
    void *address=dlsym(RTLD_DEFAULT,"_ZN6gazebo9transport10Connection9iomanagerE");
    const char *error=dlerror();
    if(error || !address) throw std::runtime_error("Unsupported Classic11 IOManager ABI symbol");
    Dl_info information{};char canonical[PATH_MAX];
    if(!dladdr(address,&information) || !information.dli_fname ||
       !realpath(information.dli_fname,canonical))
      throw std::runtime_error("Cannot verify Classic11 IOManager DSO");
    runtime=canonical;
    const auto separator=runtime.rfind('/');
    const std::string filename=runtime.substr(separator==std::string::npos?0:separator+1);
    if(filename!=std::string("libgazebo_transport.so.")+GAZEBO_VERSION_FULL)
      throw std::runtime_error(std::string("IOManager runtime DSO differs from compiled SDK: version=")+GAZEBO_VERSION+", full="+GAZEBO_VERSION_FULL+", runtime="+runtime);
    slot=reinterpret_cast<transport::IOManager **>(address);
    // This unconnected main-owned Connection contributes a real shared-manager
    // reference before any asynchronous callbacks exist.
    owner.reset(new transport::Connection());
    io=*slot;
    if(!io || io->GetCount()<1) throw std::runtime_error("IOManager ownership was not established");
  }
  void Stop() {
    if(stopped)return;
    if(std::this_thread::get_id()!=thread || !owner || !io || *slot!=io)
      throw std::runtime_error("IOManager changed or cleanup moved off its owning main thread");
    // Public Stop stops io_service and JOINS its worker. The sentinel prevents
    // a last weakBind temporary on that worker from deleting/joining itself.
    io->Stop();stopped=true;
    if(std::getenv("GAZEBOTEACH_IO_DIAGNOSTICS"))
      std::cerr<<"{\"io_guard_joined\":true,\"runtime_dso\":"<<Q(runtime)<<",\"compiled_version\":"<<Q(GAZEBO_VERSION)<<",\"compiled_version_full\":"<<Q(GAZEBO_VERSION_FULL)<<"}"<<std::endl;
  }
 private:
  std::thread::id thread;transport::ConnectionPtr owner;
  transport::IOManager **slot=nullptr,*io=nullptr;
  bool stopped=false;std::string runtime;
};

int main()
{
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::signal(SIGHUP, Stop);
  int code=0;bool initialized=false;std::unique_ptr<MainThreadIOOwner> ioOwner;
  transport::NodePtr node;std::vector<transport::SubscriberPtr> subscriptions;
  try {
    ioOwner.reset(new MainThreadIOOwner());initialized=true;
    if (!gazebo::transport::init("",0,3)) throw std::runtime_error("Classic master setup failed");
    gazebo::transport::run();
    node.reset(new gazebo::transport::Node);
    node->Init("lesson21");
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
  }catch(const std::exception &error){std::cerr<<error.what()<<std::endl;code=2;}
  try { if(initialized)transport::fini(); }
  catch(const std::exception &error){std::cerr<<error.what()<<std::endl;code=2;}
  try { if(ioOwner)ioOwner->Stop(); }
  catch(const std::exception &error){std::cerr<<error.what()<<std::endl;code=2;}
  if(node)node->Fini();subscriptions.clear();node.reset();ioOwner.reset();
  return code;
}
