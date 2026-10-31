// Persistent private Classic11 control client; no SDK lifecycle enters gzserver.
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/IOManager.hh>
#include <gazebo/transport/Connection.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <dlfcn.h>
#include <cstdlib>
#include <climits>
#include <memory>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <algorithm>
#include <cctype>
#include <cmath>
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
void Print(const std::string &v) { std::cout<<v<<std::endl; }
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

struct Statistics {
  std::mutex mutex;std::condition_variable condition;
  uint64_t sequence=0;msgs::WorldStatistics value;
  void Receive(ConstWorldStatisticsPtr &message) {
    std::lock_guard<std::mutex> lock(mutex);value.CopyFrom(*message);++sequence;condition.notify_all();
  }
  struct Sample {uint64_t seq;msgs::WorldStatistics data;};
  uint64_t Sequence() {std::lock_guard<std::mutex> lock(mutex);return sequence;}
  template<class Predicate>
  Sample Fresh(uint64_t after,std::chrono::steady_clock::time_point end,Predicate predicate) {
    std::unique_lock<std::mutex> lock(mutex);
    if(!condition.wait_until(lock,end,[&]{return sequence>after&&predicate(value);}))
      throw std::runtime_error("Fresh actual WorldStatistics confirmation timed out; mutation was not retried");
    return {sequence,value};
  }
};

std::string Json(const Statistics::Sample &sample) {
  const auto &s=sample.data;std::ostringstream out;out.precision(17);
  out<<"{\"seq\":"<<sample.seq<<",\"iterations\":"<<s.iterations()
     <<",\"paused\":"<<(s.paused()?"true":"false")<<",\"sim_time\":"
     <<s.sim_time().sec()+s.sim_time().nsec()/1e9<<",\"sim_sec\":"<<s.sim_time().sec()
     <<",\"sim_nsec\":"<<s.sim_time().nsec()<<"}";return out.str();
}

int main(int argc,char **argv) {
  int code=0;bool initialized=false;std::unique_ptr<MainThreadIOOwner> owner;
  transport::NodePtr node;transport::PublisherPtr publisher;transport::SubscriberPtr subscriber;
  Statistics stats;
  try {
    if(argc!=2||std::string(argv[1]).empty())throw std::runtime_error("Use native_control WORLD_NAME");
    if(!std::getenv("GAZEBOTEACH_SESSION_TOKEN"))throw std::runtime_error("Private owned session required");
    owner.reset(new MainThreadIOOwner());initialized=true;
    if(!transport::init("",0,3))throw std::runtime_error("Native master unavailable");
    transport::run();node.reset(new transport::Node());node->Init(argv[1]);
    subscriber=node->Subscribe("~/world_stats",&Statistics::Receive,&stats,false);
    publisher=node->Advertise<msgs::WorldControl>("~/world_control");
    if(!publisher->WaitForConnection(common::Time(3,0)))throw std::runtime_error("WorldControl publisher not connected");
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(8);
    auto ready=stats.Fresh(0,end,[](const msgs::WorldStatistics &){return true;});
    Print("{\"ready\":true,\"stats\":"+Json(ready)+"}");
    std::string line;
    while(std::getline(std::cin,line)) {
      std::string id;
      try {
        google::protobuf::Struct command;
        if(!google::protobuf::util::JsonStringToMessage(line,&command).ok())throw std::runtime_error("Malformed JSON request");
        const auto &f=command.fields();
        if(!f.count("id")||f.at("id").kind_case()!=google::protobuf::Value::kStringValue)
          throw std::runtime_error("Request needs string UUID");
        id=f.at("id").string_value();
        if(id.empty()||id.size()>80||!std::all_of(id.begin(),id.end(),[](unsigned char c){return std::isalnum(c)||c=='_';}))
          throw std::runtime_error("Invalid UUID");
        if(!f.count("action")||f.at("action").kind_case()!=google::protobuf::Value::kStringValue||
           !f.count("timeout_ms")||f.at("timeout_ms").kind_case()!=google::protobuf::Value::kNumberValue)
          throw std::runtime_error("Action and finite timeout_ms required");
        const std::string action=f.at("action").string_value();const double timeout=f.at("timeout_ms").number_value();
        if(!std::isfinite(timeout)||timeout<100||timeout>20000)throw std::runtime_error("timeout_ms outside 100..20000");
        end=std::chrono::steady_clock::now()+std::chrono::milliseconds(static_cast<int>(timeout));
        auto baseline=stats.Fresh(stats.Sequence(),end,[](const msgs::WorldStatistics &){return true;});
        auto observed=baseline;std::string extra;
        if(action=="status") {
          if(f.size()!=3)throw std::runtime_error("Unexpected status field");
        } else {
          msgs::WorldControl control;
          if(action=="pause") {
            if(f.size()!=4||!f.count("paused")||f.at("paused").kind_case()!=google::protobuf::Value::kBoolValue)
              throw std::runtime_error("Pause requires explicit bool paused");
            const bool target=f.at("paused").bool_value();control.set_pause(target);
            publisher->Publish(control,true); // Exactly one mutation.
            observed=stats.Fresh(baseline.seq,end,[&](const msgs::WorldStatistics &s){return s.paused()==target;});
          } else if(action=="step") {
            if(f.size()!=4||!f.count("count")||f.at("count").kind_case()!=google::protobuf::Value::kNumberValue)
              throw std::runtime_error("Step requires positive integral count");
            const double number=f.at("count").number_value();
            if(!std::isfinite(number)||number<1||number>1000000||std::floor(number)!=number)
              throw std::runtime_error("Step count outside 1..1000000");
            if(!baseline.data.paused())throw std::runtime_error("Step baseline is not actually paused");
            const uint64_t target=baseline.data.iterations()+static_cast<uint64_t>(number);
            control.set_pause(true);control.set_multi_step(static_cast<uint32_t>(number));
            publisher->Publish(control,true); // Unknown outcomes must not be retried.
            observed=stats.Fresh(baseline.seq,end,[&](const msgs::WorldStatistics &s){return s.iterations()>=target;});
            if(!observed.data.paused()||observed.data.iterations()!=target)
              throw std::runtime_error("Foreign stepping/unpause or iteration overshoot; exact requested count unconfirmed");
            extra=",\"baseline_iterations\":"+std::to_string(baseline.data.iterations())+",\"requested_steps\":"+std::to_string(static_cast<int>(number));
          } else if(action=="reset_models") {
            if(f.size()!=3)throw std::runtime_error("Unexpected reset_models field");
            auto reset=control.mutable_reset();reset->set_all(false);reset->set_model_only(true);reset->set_time_only(false);
            publisher->Publish(control,true);
            observed=stats.Fresh(baseline.seq,end,[](const msgs::WorldStatistics &){return true;});
            extra=",\"confirmation\":\"fresh statistics after single model-only reset request; verify model poses independently\"";
          } else throw std::runtime_error("Unsupported finite action");
        }
        Print("{\"id\":"+Q(id)+",\"ok\":true,\"stats\":"+Json(observed)+extra+"}");
      } catch(const std::exception &error) {
        Print("{\"id\":"+Q(id)+",\"ok\":false,\"error\":"+Q(error.what())+"}");
      }
    }
  } catch(const std::exception &error) {Print("{\"ready\":false,\"error\":"+Q(error.what())+"}");code=1;}
  try {
    bool joined=false;
    if(initialized)transport::fini();if(owner){owner->Stop();joined=true;}
    if(node)node->Fini();subscriber.reset();publisher.reset();node.reset();owner.reset();
    Print(std::string("{\"closed\":true,\"io_guard_joined\":")+(joined?"true":"false")+"}");
  } catch(const std::exception &error) {Print("{\"closed\":false,\"error\":"+Q(error.what())+"}");code=1;}
  return code;
}

