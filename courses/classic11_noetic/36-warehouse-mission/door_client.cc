// Wire-only Classic client: retain raw native messages; no motion predictions.
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/IOManager.hh>
#include <gazebo/transport/Connection.hh>
#include <gazebo/gazebo_config.h>
#include <dlfcn.h>
#include <cstdlib>
#include <climits>
#include <memory>
#include <fstream>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <chrono>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace gazebo;
namespace {
std::mutex outputMutex;
long long Now() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string Q(const std::string &v) {
  std::ostringstream out; out << '"';
  for (unsigned char c : v) {
    if (c == '"' || c == '\\') out << '\\';
    if (c == '\n') out << "\\n"; else if (c == '\r') out << "\\r";
    else if (c == '\t') out << "\\t"; else if (c >= 32) out << c;
  }
  return out.str() + '"';
}
void Print(const std::string &v) { std::lock_guard<std::mutex> guard(outputMutex); std::cout << v << std::endl; }
std::string Json(const google::protobuf::Message &message) {
  std::string text;
  google::protobuf::util::JsonPrintOptions options; options.preserve_proto_field_names = true;
  auto result = google::protobuf::util::MessageToJsonString(message, &text, options);
  if (!result.ok()) throw std::runtime_error("Native protobuf JSON conversion failed");
  return text;
}
std::string Hex(const google::protobuf::Message &message) {
  const std::string bytes=message.SerializeAsString();
  static const char *digits="0123456789abcdef";std::string result;result.reserve(bytes.size()*2);
  for(unsigned char c:bytes){result+=digits[c>>4];result+=digits[c&15];}return result;
}
struct Receiver {
  std::mutex mutex; std::condition_variable condition;
  bool responseReady = false, statsReady = false;
  std::atomic<bool> watching{false};
  int id = 0; std::string requestName; msgs::Response response; msgs::WorldStatistics statistics;
  void Poses(ConstPosesStampedPtr &m) {
    if (watching) Print("{\"source\":\"pose\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + ",\"serialized_protobuf_hex\":" + Q(Hex(*m)) + "}");
  }
  void Stats(ConstWorldStatisticsPtr &m) {
    { std::lock_guard<std::mutex> guard(mutex); statistics.CopyFrom(*m); statsReady = true; condition.notify_all(); }
    if (watching) Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + ",\"serialized_protobuf_hex\":" + Q(Hex(*m)) + "}");
  }
  void Response(ConstResponsePtr &m) {
    std::lock_guard<std::mutex> guard(mutex);
    if (m->id() == id && m->request() == requestName) { response.CopyFrom(*m); responseReady = true; condition.notify_all(); }
  }
};
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

struct DoorReply {
  std::mutex mutex; std::condition_variable condition; std::string id,text;
  void State(ConstAnyPtr &message) {
    if(message->type()!=msgs::Any::STRING) return;
    std::lock_guard<std::mutex> guard(mutex);
    if(message->string_value().find("\"id\":\""+id+"\"")!=std::string::npos) {
      text=message->string_value(); condition.notify_all();
    }
  }
};

int main(int argc,char **argv) {
  int code=0; bool initialized=false; std::unique_ptr<MainThreadIOOwner> owner;
  DoorReply reply; transport::NodePtr node; transport::PublisherPtr publisher;
  transport::SubscriberPtr subscriber;
  try {
    if(argc!=3) throw std::runtime_error("Use door_client open|enable|status 0|1 or cap 12|20|30");
    std::string action=argv[1],value=argv[2];
    bool valid=(action=="cap"&&(value=="12"||value=="20"||value=="30")) ||
      ((action=="open"||action=="enable")&&(value=="0"||value=="1")) || (action=="status"&&value=="0");
    if(!valid) throw std::runtime_error("Bad finite command");
    if(!std::getenv("GAZEBOTEACH_SESSION_TOKEN")) throw std::runtime_error("Private session required");
    owner.reset(new MainThreadIOOwner()); initialized=true;
    if(!transport::init("",0,3)) throw std::runtime_error("Native master unavailable");
    transport::run(); node.reset(new transport::Node()); node->Init("default");
    reply.id=std::to_string(Now());
    subscriber=node->Subscribe("~/lesson36/door/state",&DoorReply::State,&reply,true);
    publisher=node->Advertise<msgs::Any>("~/lesson36/door/command");
    if(!publisher->WaitForConnection(common::Time(2,0))) throw std::runtime_error("Door plugin not connected");
    msgs::Any message; message.set_type(msgs::Any::STRING);
    message.set_string_value(action+" "+value+" "+reply.id);
    // Exactly one command. The callback ack, not a guessed delay, confirms it.
    publisher->Publish(message,true);
    { std::unique_lock<std::mutex> guard(reply.mutex);
      if(!reply.condition.wait_for(guard,std::chrono::seconds(3),[&]{return !reply.text.empty();}))
        throw std::runtime_error("Door command acknowledgement timed out");
      Print(reply.text);
    }
  } catch(const std::exception &error) { Print("{\"error\":"+Q(error.what())+"}"); code=1; }
  try { if(initialized) transport::fini(); if(owner) owner->Stop();
    if(node)node->Fini(); subscriber.reset();publisher.reset();node.reset();owner.reset();
  } catch(const std::exception &error) { Print("{\"cleanup_error\":"+Q(error.what())+"}"); code=1; }
  return code;
}

