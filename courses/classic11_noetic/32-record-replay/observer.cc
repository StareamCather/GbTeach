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
#include <gazebo/util/LogPlay.hh>
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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

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
struct Receiver {
  std::mutex mutex; std::condition_variable condition;
  bool responseReady = false, statsReady = false, logReady = false;
  std::atomic<bool> watching{false};
  int id = 0; std::string requestName; msgs::Response response; msgs::WorldStatistics statistics;
  msgs::LogStatus logStatus;
  void Log(ConstLogStatusPtr &m) {
    { std::lock_guard<std::mutex> guard(mutex);logStatus.CopyFrom(*m);logReady=true;condition.notify_all(); }
    if(watching) Print("{\"source\":\"log_status\",\"receive_monotonic_ns\":"+std::to_string(Now())+
        ",\"data\":"+Json(*m)+",\"raw_protobuf\":"+Q(m->DebugString())+"}");
  }
  void Poses(ConstPosesStampedPtr &m) {
    if (watching) Print("{\"source\":\"pose\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + "}");
  }
  void Stats(ConstWorldStatisticsPtr &m) {
    { std::lock_guard<std::mutex> guard(mutex); statistics.CopyFrom(*m); statsReady = true; condition.notify_all(); }
    if (watching) Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":" + std::to_string(Now()) +
        ",\"data\":" + Json(*m) + ",\"raw_protobuf\":" + Q(m->DebugString()) + "}");
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
    if (argc < 2) throw std::runtime_error("Use watch/entity/world/physics/stats/control/log-status/log-control/log-read/playback-control");
    std::string mode(argv[1]);
    std::map<std::string, std::string> options;
    for (int i=2; i<argc; ++i) {
      std::string key(argv[i]);
      if (key == "--reset-models" || key == "--reset-all") options[key] = "1";
      else { if (key.substr(0,2) != "--" || i+1 >= argc) throw std::runtime_error("Bad option"); options[key] = argv[++i]; }
    }
    double timeout = options.count("--timeout") ? std::stod(options["--timeout"]) : 3;
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 8) throw std::runtime_error("Timeout must be within (0,8]");
    if(options.count("--world") && options["--world"]!="lesson32")
      throw std::runtime_error("This teaching client manages only its own lesson32 world");
    if(options.count("--name") && options["--name"]!="falling_cube" && options["--name"]!="floor")
      throw std::runtime_error("This teaching client queries only its own falling_cube or floor");
    // OFFLINE path: no master, Node, IOManager or transport initialized. Callers
    // must prevalidate the closed XML because LogPlay::Open repairs bad logs.
    if(mode=="log-read") {
      if(!options.count("--file"))throw std::runtime_error("Use --file closed native state.log");
      std::ifstream input(options["--file"],std::ios::binary);
      std::string bytes((std::istreambuf_iterator<char>(input)),std::istreambuf_iterator<char>());
      if(bytes.size()>16*1024*1024 || bytes.find("</gazebo_log>")==std::string::npos)
        throw std::runtime_error("Refuse an incomplete or oversized native log before LogPlay::Open");
      boost::property_tree::ptree xml;std::istringstream document(bytes);
      boost::property_tree::read_xml(document,xml);
      if(xml.count("gazebo_log")!=1)throw std::runtime_error("Expected one complete gazebo_log root");
      auto play=util::LogPlay::Instance();play->Open(options["--file"]);
      std::string frame;unsigned int index=0;
      while(play->Step(frame)) {
        if(index>=10000)throw std::runtime_error("Native frame count exceeds bounded lesson");
        Print("{\"source\":\"log_frame\",\"frame_index\":"+std::to_string(index++)+",\"raw_sdf\":"+Q(frame)+"}");
      }
      Print("{\"source\":\"log_metadata\",\"log_version\":"+Q(play->LogVersion())+
        ",\"gazebo_version\":"+Q(play->GazeboVersion())+",\"rand_seed\":"+std::to_string(play->RandSeed())+
        ",\"start_time_ns\":"+std::to_string(play->LogStartTime().sec*1000000000LL+play->LogStartTime().nsec)+
        ",\"end_time_ns\":"+std::to_string(play->LogEndTime().sec*1000000000LL+play->LogEndTime().nsec)+
        ",\"chunk_count\":"+std::to_string(play->ChunkCount())+",\"frame_count\":"+std::to_string(index)+
        ",\"encoding\":"+Q(play->Encoding())+",\"header\":"+Q(play->Header())+"}");
      return 0;
    }
    ioOwner.reset(new MainThreadIOOwner());
    initializationAttempted = true;
    if (!transport::init("", 0, 3)) throw std::runtime_error("Private Classic master connection failed");
    transport::run();
    node.reset(new transport::Node());
    node->Init(options.count("--world") ? options["--world"] : "lesson32");
    if (mode == "watch") {
      double seconds = options.count("--seconds") ? std::stod(options["--seconds"]) : 3;
      if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120) throw std::runtime_error("Watch 0..120 wall seconds");
      receiver.watching = true;
      auto poses = node->Subscribe("~/pose/info", &Receiver::Poses, &receiver);
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      auto log = node->Subscribe("~/log/status", &Receiver::Log, &receiver);
      subscriptions.insert(subscriptions.end(), {poses, stats, log});
      Print("{\"source\":\"client_ready\",\"receive_monotonic_ns\":"+std::to_string(Now())+"}");
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
      while(std::chrono::steady_clock::now()<deadline) {
        if(options.count("--stop-file")&&std::ifstream(options["--stop-file"]).good())break;
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
      }
    } else if(mode=="log-status") {
      subscriptions.push_back(node->Subscribe("~/log/status",&Receiver::Log,&receiver));
      auto query=node->Advertise<msgs::LogControl>("~/log/control");publishers.push_back(query);
      if(!query->WaitForConnection(common::Time(1,0)))throw std::runtime_error("No native recorder status-query subscriber");
      // Classic has no Subscriber readiness API and does not periodically
      // publish idle status. LogControl(start=false) asks OnLogControl to
      // PublishLogStatus without starting anything. The explicit false makes
      // a nonempty protobuf wire payload. Repeating this query is safe; start/stop,
      // pause, seek and finite-step publications elsewhere remain one-shot.
      msgs::LogControl emptyQuery;emptyQuery.set_start(false);unsigned int queryCount=0;
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
      while(true) {
        query->Publish(emptyQuery,true);++queryCount;
        std::unique_lock<std::mutex> guard(receiver.mutex);
        if(receiver.condition.wait_for(guard,std::chrono::milliseconds(100),[&]{return receiver.logReady;}))break;
        if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("No actual native LogStatus after bounded start=false queries");
      }
      std::unique_lock<std::mutex> guard(receiver.mutex);
      Print("{\"source\":\"log_status\",\"data\":"+Json(receiver.logStatus)+",\"raw_protobuf\":"+Q(receiver.logStatus.DebugString())+
          ",\"query_raw_protobuf\":"+Q(emptyQuery.DebugString())+",\"query_count\":"+std::to_string(queryCount)+"}");
    } else if(mode=="log-control") {
      auto pub=node->Advertise<msgs::LogControl>("~/log/control");publishers.push_back(pub);
      if(!pub->WaitForConnection(common::Time(1,0)))throw std::runtime_error("No native recorder subscriber");
      msgs::LogControl command;
      if(options["--action"]=="start") {
        if(!options.count("--path") || options["--path"].empty() || options["--path"][0]!='/')throw std::runtime_error("Use absolute recording --path");
        auto encoding=options["--encoding"];
        if(encoding!="zlib" && encoding!="txt" && encoding!="bz2")throw std::runtime_error("Unknown native encoding");
        command.set_base_path(options["--path"]);command.set_encoding(encoding);command.set_start(true);
      } else if(options["--action"]=="stop")command.set_stop(true);
      else throw std::runtime_error("Use --action start|stop");
      pub->Publish(command,true);
      Print("{\"source\":\"log_control_sent\",\"raw_protobuf\":"+Q(command.DebugString())+"}");
    } else if(mode=="playback-control") {
      auto pub=node->Advertise<msgs::LogPlaybackControl>("~/playback_control");publishers.push_back(pub);
      if(!pub->WaitForConnection(common::Time(1,0)))throw std::runtime_error("No native playback subscriber");
      msgs::LogPlaybackControl command;command.set_pause(true);
      if(options.count("--seek-ns")) {
        auto ns=std::stoll(options["--seek-ns"]);if(ns<0 || ns>1000000000LL)throw std::runtime_error("Seek within this bounded one-second lesson log");
        command.mutable_seek()->set_sec(ns/1000000000LL);command.mutable_seek()->set_nsec(ns%1000000000LL);
      } else if(options.count("--steps")) {
        auto count=std::stoi(options["--steps"]);if(count<1 || count>1000)throw std::runtime_error("Use 1..1000 playback states");
        command.set_multi_step(count);
      } else throw std::runtime_error("Use playback --steps or --seek-ns");
      pub->Publish(command,true);Print("{\"source\":\"playback_control_sent\",\"raw_protobuf\":"+Q(command.DebugString())+"}");
    } else if (mode == "stats") {
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      subscriptions.push_back(stats);
      { std::unique_lock<std::mutex> guard(receiver.mutex);
        if (!receiver.condition.wait_for(guard, std::chrono::duration<double>(timeout), [&]{return receiver.statsReady;}))
          throw std::runtime_error("No actual native world statistics");
        Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":"+std::to_string(Now())+
              ",\"data\":"+Json(receiver.statistics)+",\"raw_protobuf\":"+Q(receiver.statistics.DebugString())+"}"); }
    } else if (mode == "entity" || mode == "world" || mode == "physics") {
      { std::lock_guard<std::mutex> guard(receiver.mutex);
        receiver.id = static_cast<int>((Now() & 0x3fffffff)+1);
        receiver.requestName = mode == "world" ? "scene_info" : mode == "physics" ? "physics_info" : "entity_info";
      }
      const std::string nonce = "l32-"+std::to_string(Now());
      auto sub = node->Subscribe("~/response", &Receiver::Response, &receiver, true);
      auto pub = node->Advertise<msgs::Request>("~/request");
      subscriptions.push_back(sub);
      publishers.push_back(pub);
      if (!pub->WaitForConnection(common::Time(1,0))) throw std::runtime_error("No native request subscriber");
      msgs::Request query; query.set_id(receiver.id); query.set_request(receiver.requestName);
      query.set_data(options.count("--name") ? options["--name"] : "falling_cube"); pub->Publish(query);
      { std::unique_lock<std::mutex> guard(receiver.mutex);
        if (!receiver.condition.wait_for(guard, std::chrono::duration<double>(timeout), [&]{return receiver.responseReady;}))
          throw std::runtime_error("No matching native response before deadline"); }
      msgs::Response reply;
      { std::lock_guard<std::mutex> guard(receiver.mutex); reply.CopyFrom(receiver.response); }
      if (reply.response() != "success") throw std::runtime_error("Native request rejected");
      if (mode == "physics") {
        msgs::Physics value;
        if (reply.type() != "gazebo.msgs.Physics" || !value.ParseFromString(reply.serialized_data())) throw std::runtime_error("Wrong native physics type/bytes");
        Print("{\"source\":\"physics\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(value.DebugString())+"}");
      } else if (mode == "world") {
        msgs::Scene value;
        if (reply.type() != "gazebo.msgs.Scene" || !value.ParseFromString(reply.serialized_data())) throw std::runtime_error("Wrong native scene type/bytes");
        Print("{\"source\":\"world\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(value.DebugString())+"}");
      } else {
        msgs::Model value;
        if (reply.type() != "gazebo.msgs.Model" || !value.ParseFromString(reply.serialized_data())) throw std::runtime_error("Wrong native entity type/bytes");
        Print("{\"source\":\"entity\",\"data\":"+Json(value)+",\"raw_protobuf\":"+Q(value.DebugString())+"}");
      }
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
