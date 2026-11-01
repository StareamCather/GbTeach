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
#include <random>
#include <algorithm>

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
std::string AnyJson(const msgs::Any &message) {
  if (message.type() != msgs::Any::STRING || !message.has_string_value())
    throw std::runtime_error("Expected actual Any.STRING instrumentation");
  google::protobuf::Struct value;
  auto result = google::protobuf::util::JsonStringToMessage(message.string_value(), &value);
  if (!result.ok()) throw std::runtime_error("Plugin sent malformed JSON");
  return message.string_value();
}
struct Receiver {
  std::mutex mutex; std::condition_variable condition;
  bool responseReady = false, statsReady = false;
  std::atomic<bool> watching{false};
  int id = 0; std::string requestName; msgs::Response response; msgs::WorldStatistics statistics;
  void Poses(ConstPosesStampedPtr &m) {
    if (watching) Print("{\"source\":\"pose\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + "}");
  }
  void Stats(ConstWorldStatisticsPtr &m) {
    { std::lock_guard<std::mutex> guard(mutex); statistics.CopyFrom(*m); statsReady = true; condition.notify_all(); }
    if (watching) Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + "}");
  }
  void Plugin(ConstAnyPtr &m) {
    if (watching) {
      try { Print("{\"source\":\"plugin\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
          ",\"payload\":" + AnyJson(*m) + ",\"raw_payload_json\":" + Q(m->string_value()) +
          ",\"raw_protobuf\":" + Q(m->DebugString()) + "}"); }
      catch (const std::exception &error) { Print("{\"source\":\"decode_error\",\"error\":"+Q(error.what())+"}"); }
    }
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

int main(int argc, char **argv) {
  bool initializationAttempted = false;
  std::unique_ptr<MainThreadIOOwner> ioOwner;
  int code = 0;
  // Keep owners alive on MAIN through transport stop/join. Releasing the last
  // publisher on an asynchronous publication worker can make its destructor
  // join that same worker. Callback targets must outlive the joined transport.
  Receiver receiver;
  transport::NodePtr node;
  std::vector<transport::PublisherPtr> publishers;
  std::vector<transport::SubscriberPtr> subscriptions;
  try {
    if (argc < 2) throw std::runtime_error("Use watch/snapshot/entity/world/physics/stats/control");
    std::string mode(argv[1]);
    std::map<std::string, std::string> options;
    for (int i=2; i<argc; ++i) {
      std::string key(argv[i]);
      if (key == "--reset-models" || key == "--reset-all") options[key] = "1";
      else { if (key.substr(0,2) != "--" || i+1 >= argc) throw std::runtime_error("Bad option"); options[key] = argv[++i]; }
    }
    double timeout = options.count("--timeout") ? std::stod(options["--timeout"]) : 3;
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 8) throw std::runtime_error("Timeout must be within (0,8]");
    if(options.count("--world") && options["--world"]!="lesson31")
      throw std::runtime_error("This teaching client manages only its own lesson31 world");
    if(options.count("--name") && options["--name"]!="falling_cube" && options["--name"]!="floor")
      throw std::runtime_error("This teaching client queries only its own falling_cube or floor");
    ioOwner.reset(new MainThreadIOOwner());
    initializationAttempted = true;
    if (!transport::init("", 0, 3)) throw std::runtime_error("Private Classic master connection failed");
    transport::run();
    node.reset(new transport::Node());
    node->Init(options.count("--world") ? options["--world"] : "lesson31");
    if (mode == "watch") {
      double seconds = options.count("--seconds") ? std::stod(options["--seconds"]) : 3;
      if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120) throw std::runtime_error("Watch 0..120 wall seconds");
      receiver.watching = true;
      auto poses = node->Subscribe("~/pose/info", &Receiver::Poses, &receiver);
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      auto plugin = node->Subscribe("~/system_plugin/state", &Receiver::Plugin, &receiver);
      subscriptions.insert(subscriptions.end(), {poses, stats, plugin});
      Print("{\"source\":\"client_ready\",\"receive_monotonic_ns\":"+std::to_string(Now())+"}");
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
      while(std::chrono::steady_clock::now()<deadline) {
        if(options.count("--stop-file")&&std::ifstream(options["--stop-file"]).good())break;
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
      }
    } else if (mode == "stats") {
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      subscriptions.push_back(stats);
      { std::unique_lock<std::mutex> guard(receiver.mutex);
        if (!receiver.condition.wait_for(guard, std::chrono::duration<double>(timeout), [&]{return receiver.statsReady;}))
          throw std::runtime_error("No actual native world statistics");
        Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":"+std::to_string(Now())+
              ",\"data\":"+Json(receiver.statistics)+",\"raw_protobuf\":"+Q(receiver.statistics.DebugString())+"}"); }
    } else if (mode == "snapshot" || mode == "entity" || mode == "world" || mode == "physics") {
      // Creating the request/response channel, connecting and both READ attempts
      // share ONE deadline. SDK process initialization above has its own bound.
      const auto readDeadline = std::chrono::steady_clock::now()+
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout));
      const std::string requestName = mode == "snapshot" ? "gazeboteach_system_snapshot" : mode == "world" ? "scene_info" : mode == "physics" ? "physics_info" : "entity_info";
      const std::string expectedName = options.count("--name") ? options["--name"] : "falling_cube";
      const std::string expectedWorld = options.count("--world") ? options["--world"] : "lesson31";
      if (mode == "entity" && (expectedName.empty() || expectedName.size()>512 ||
          std::any_of(expectedName.begin(),expectedName.end(),[](unsigned char c){return c<32||c==127;})))
        throw std::runtime_error("Invalid entity name");
      { std::lock_guard<std::mutex> guard(receiver.mutex); receiver.requestName=requestName; }
      auto sub = node->Subscribe("~/response", &Receiver::Response, &receiver, false);
      auto pub = node->Advertise<msgs::Request>("~/request");
      subscriptions.push_back(sub);
      publishers.push_back(pub);
      const double connectSeconds=std::chrono::duration<double>(readDeadline-std::chrono::steady_clock::now()).count();
      if (connectSeconds<=0 || !pub->WaitForConnection(common::Time(connectSeconds)))
        throw std::runtime_error("No native request subscriber within shared read deadline");
      std::random_device random;
      const auto seedTime=static_cast<unsigned long long>(Now());
      std::seed_seq seed{random(),static_cast<unsigned int>(seedTime),static_cast<unsigned int>(seedTime>>32)};
      std::mt19937 ids(seed);std::uniform_int_distribution<int> positiveID(1,INT_MAX);
      int previousID=0;bool received=false;
      for (unsigned int attempt=0;attempt<2;++attempt) {
        if (std::chrono::steady_clock::now()>=readDeadline)break;
        int id;do { id=positiveID(ids); } while(id==previousID);previousID=id;
        const std::string nonce="l31-"+std::to_string(Now())+"-"+std::to_string(id);
        { std::lock_guard<std::mutex> guard(receiver.mutex);
          receiver.id=id;receiver.responseReady=false;receiver.response.Clear(); }
        msgs::Request query;query.set_id(id);query.set_request(requestName);
        query.set_data(mode=="snapshot" ? nonce : expectedName);
        pub->Publish(query,true);
        msgs::Response reply;
        { std::unique_lock<std::mutex> guard(receiver.mutex);
          const auto now=std::chrono::steady_clock::now();
          const auto until=attempt==0 ? now+(readDeadline-now)/2 : readDeadline;
          // Only a missing READ response permits another attempt. Reuse the
          // live channel, but give attempt two an independent ID and nonce.
          // Late responses for the previous ID are ignored by Receiver.
          if (!receiver.condition.wait_until(guard,until,[&]{return receiver.responseReady;}))continue;
          reply.CopyFrom(receiver.response);
        }
        if (!reply.IsInitialized() || reply.id()!=id || reply.request()!=requestName || reply.response()!="success")
          throw std::runtime_error("Native read response correlation/success mismatch");
        // A correlated protocol/type/name/nonce failure is never retried.
        if (mode == "snapshot") {
          msgs::Any value;
          if (reply.type()!="gazebo.msgs.Any" || !value.ParseFromString(reply.serialized_data()) || !value.IsInitialized())
            throw std::runtime_error("Wrong plugin reply type/bytes");
          const auto payload=AnyJson(value);
          google::protobuf::Struct decoded;
          if (!google::protobuf::util::JsonStringToMessage(payload,&decoded).ok())
            throw std::runtime_error("Plugin reply JSON is malformed");
          const auto found=decoded.fields().find("query_nonce");
          if (found==decoded.fields().end() || found->second.kind_case()!=google::protobuf::Value::kStringValue ||
              found->second.string_value()!=nonce)throw std::runtime_error("Plugin reply nonce mismatch");
          Print("{\"source\":\"snapshot\",\"request_id\":"+std::to_string(id)+",\"response_id\":"+
                std::to_string(reply.id())+",\"payload\":"+payload+",\"raw_protobuf\":"+Q(reply.DebugString())+"}");
        } else if (mode == "physics") {
          msgs::Physics value;
          if (reply.type() != "gazebo.msgs.Physics" || !value.ParseFromString(reply.serialized_data()) || !value.IsInitialized())
            throw std::runtime_error("Wrong native physics type/bytes");
          Print("{\"source\":\"physics\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(reply.DebugString())+"}");
        } else if (mode == "world") {
          msgs::Scene value;
          if (reply.type()!="gazebo.msgs.Scene" || !value.ParseFromString(reply.serialized_data()) || !value.IsInitialized())
            throw std::runtime_error("Wrong native scene type/bytes");
          if (value.name()!=expectedWorld)throw std::runtime_error("Native scene name differs from requested world");
          Print("{\"source\":\"world\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(reply.DebugString())+"}");
        } else {
          msgs::Model value;
          if (reply.type()!="gazebo.msgs.Model" || !value.ParseFromString(reply.serialized_data()) || !value.IsInitialized() || !value.has_pose())
            throw std::runtime_error("Wrong native entity type/bytes");
          const auto separator=value.name().rfind("::");
          const auto leaf=value.name().substr(separator==std::string::npos ? 0 : separator+2);
          if (value.name()!=expectedName && leaf!=expectedName)
            throw std::runtime_error("Native entity name differs from requested model");
          Print("{\"source\":\"entity\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(reply.DebugString())+"}");
        }
        received=true;break;
      }
      if (!received)throw std::runtime_error("No matching native response before deadline");
    } else if (mode == "control") {
      auto pub = node->Advertise<msgs::WorldControl>("~/world_control");
      publishers.push_back(pub);
      if (!pub->WaitForConnection(common::Time(1,0))) throw std::runtime_error("No native world control subscriber");
      msgs::WorldControl command;
      bool mutation = false;
      if (options.count("--pause")) {
        if (options["--pause"] != "true" && options["--pause"] != "false") throw std::runtime_error("Pause must be true/false");
        command.set_pause(options["--pause"] == "true"); mutation = true;
      }
      if (options.count("--steps")) {
        const std::string text=options["--steps"];size_t consumed=0;
        const unsigned long long count=std::stoull(text,&consumed);
        if(text.empty() || text[0]=='-' || consumed!=text.size() || count<1 || count>1000)
          throw std::runtime_error("Step count outside 1..1000 integer ticks");
        if(options.count("--reset-models") || options.count("--reset-all"))
          throw std::runtime_error("Send one finite step or reset operation per request");
        command.set_pause(true); command.set_multi_step(static_cast<unsigned int>(count)); mutation = true;
      }
      if (options.count("--reset-models")) {
        // WorldReset.all defaults TRUE in Classic proto2; set it explicitly.
        command.mutable_reset()->set_all(false);
        command.mutable_reset()->set_time_only(false);
        command.mutable_reset()->set_model_only(true);
        command.set_pause(true); mutation = true;
      }
      if (options.count("--reset-all")) {
        command.mutable_reset()->set_all(true);
        command.mutable_reset()->set_time_only(false);
        command.mutable_reset()->set_model_only(false);
        command.set_pause(true); mutation = true;
      }
      if (!mutation) throw std::runtime_error("No control requested");
      pub->Publish(command, true);
      Print("{\"source\":\"control_sent\",\"raw_protobuf\":"+Q(command.DebugString())+
            ",\"note\":\"One native publication; inspect actual state for completion\"}");
    } else throw std::runtime_error("Unknown observer mode");
  } catch (const std::exception &error) {
    Print("{\"source\":\"client_error\",\"error\":"+Q(error.what())+"}"); code = 1;
  }
  // fini stops and joins the ConnectionManager run thread before clearing its
  // native registries. All callback targets and publisher owners remain alive.
  try { if (initializationAttempted) transport::fini(); }
  catch(const std::exception &error) { Print("{\"source\":\"client_error\",\"stage\":\"transport_fini\",\"error\":"+Q(error.what())+"}");code=1; }
  try { if (ioOwner) ioOwner->Stop(); }
  catch(const std::exception &error) { Print("{\"source\":\"client_error\",\"stage\":\"io_stop\",\"error\":"+Q(error.what())+"}");code=1; }
  try {
    if (node) node->Fini();
    subscriptions.clear();publishers.clear();node.reset();
  } catch(const std::exception &error) { Print("{\"source\":\"client_error\",\"stage\":\"node_cleanup\",\"error\":"+Q(error.what())+"}");code=1; }
  ioOwner.reset();
  return code;
}
