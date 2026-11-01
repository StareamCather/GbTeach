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
#include <algorithm>
#include <cctype>
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

struct ArmReply {
  std::mutex mutex; std::condition_variable condition; std::string id,text;
  void State(ConstAnyPtr &message) {
    if(message->type()!=msgs::Any::STRING) return;
    std::lock_guard<std::mutex> guard(mutex);
    if(message->string_value().find("\"id\":\""+id+"\"")!=std::string::npos) {
      text=message->string_value(); condition.notify_all();
    }
  }
};

bool Valid(const std::string &action, int value) {
  return (action=="cap"&&(value==3||value==6||value==9)) ||
    (action=="duration"&&(value==4||value==6||value==8)) ||
    (action=="enable"&&(value==0||value==1)) ||
    ((action=="start"||action=="hold"||action=="stop"||action=="status")&&value==0);
}

void PrepareReply(ArmReply &reply,const std::string &id) {
  std::lock_guard<std::mutex> guard(reply.mutex);reply.id=id;reply.text.clear();
}

void PublishCommand(transport::PublisherPtr publisher,const std::string &action,int value,const std::string &id) {
  msgs::Any message;message.set_type(msgs::Any::STRING);
  message.set_string_value(action+" "+std::to_string(value)+" "+id);
  publisher->Publish(message,true);
}

std::string WaitReply(ArmReply &reply,std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> guard(reply.mutex);
  reply.condition.wait_until(guard,deadline,[&]{return !reply.text.empty();});
  return reply.text;
}

std::string FreshHandshake(transport::PublisherPtr publisher,ArmReply &reply) {
  const std::string id=std::to_string(Now());PrepareReply(reply,id);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(std::chrono::steady_clock::now()<deadline) {
    // Only this read-only status probe can repeat. Its fresh response proves
    // both command and response connections, including at paused time zero.
    PublishCommand(publisher,"status",0,id);
    auto until=std::min(deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(250));
    std::string value=WaitReply(reply,until);if(!value.empty())return value;
  }
  throw std::runtime_error("Fresh native response handshake timed out");
}

std::string Request(transport::PublisherPtr publisher,ArmReply &reply,
                    const std::string &action,int value,const std::string &id) {
  PrepareReply(reply,id);
  PublishCommand(publisher,action,value,id); // Every requested mutation: exactly once.
  std::string answer=WaitReply(reply,std::chrono::steady_clock::now()+std::chrono::seconds(3));
  if(answer.empty())throw std::runtime_error("Native command acknowledgement timed out; command was not retried");
  return answer;
}

int main(int argc,char **argv) {
  int code=0; bool initialized=false; std::unique_ptr<MainThreadIOOwner> owner;
  ArmReply reply; transport::NodePtr node; transport::PublisherPtr publisher;
  transport::SubscriberPtr subscriber;
  try {
    if(argc!=3) throw std::runtime_error("Use arm_client stream 0, start|hold|stop|status 0, enable 0|1, duration 4|6|8, cap 3|6|9");
    std::string action=argv[1],value=argv[2];
    const bool streaming=action=="stream"&&value=="0";
    bool valid=streaming || ((value=="0"||value=="1"||value=="3"||value=="4"||value=="6"||value=="8"||value=="9")&&Valid(action,std::stoi(value)));
    if(!valid) throw std::runtime_error("Bad finite command");
    if(!std::getenv("GAZEBOTEACH_SESSION_TOKEN")) throw std::runtime_error("Private session required");
    owner.reset(new MainThreadIOOwner()); initialized=true;
    if(!transport::init("",0,3)) throw std::runtime_error("Native master unavailable");
    transport::run(); node.reset(new transport::Node()); node->Init("default");
    subscriber=node->Subscribe("~/lesson37/arm/state",&ArmReply::State,&reply,false);
    publisher=node->Advertise<msgs::Any>("~/lesson37/arm/command");
    if(!publisher->WaitForConnection(common::Time(2,0))) throw std::runtime_error("Arm plugin not connected");
    const std::string initial=FreshHandshake(publisher,reply);
    if(streaming) {
      Print("{\"stream_ready\":true,\"status\":"+initial+"}");
      std::string line;
      while(std::getline(std::cin,line)) {
        google::protobuf::Struct command;
        if(!google::protobuf::util::JsonStringToMessage(line,&command).ok()||command.fields_size()!=3)
          throw std::runtime_error("Use one JSON object with action, value and id per line");
        const auto &fields=command.fields();
        if(!fields.count("action")||!fields.count("value")||!fields.count("id")||
           fields.at("action").kind_case()!=google::protobuf::Value::kStringValue||
           fields.at("id").kind_case()!=google::protobuf::Value::kStringValue||
           fields.at("value").kind_case()!=google::protobuf::Value::kNumberValue)
          throw std::runtime_error("Invalid native JSON command types");
        const std::string name=fields.at("action").string_value(),id=fields.at("id").string_value();
        const double number=fields.at("value").number_value();
        if(!std::isfinite(number)||number<0||number>9||std::floor(number)!=number||!Valid(name,static_cast<int>(number))||
           id.empty()||id.size()>80||!std::all_of(id.begin(),id.end(),[](unsigned char c){return std::isalnum(c)||c=='_';}))
          throw std::runtime_error("Invalid finite native command or request identity");
        Print(Request(publisher,reply,name,static_cast<int>(number),id));
      }
      // EOF is the normal owner-driven shutdown. Cleanup remains on this main
      // thread, including transport fini and the exact-SDK IO worker join.
    } else if(action=="status") {
      Print(initial);
    } else {
      Print(Request(publisher,reply,action,std::stoi(value),std::to_string(Now())));
    }
  } catch(const std::exception &error) { Print("{\"error\":"+Q(error.what())+"}"); code=1; }
  try { if(initialized) transport::fini(); if(owner) owner->Stop();
    if(node)node->Fini(); subscriber.reset();publisher.reset();node.reset();owner.reset();
  } catch(const std::exception &error) { Print("{\"cleanup_error\":"+Q(error.what())+"}"); code=1; }
  return code;
}

