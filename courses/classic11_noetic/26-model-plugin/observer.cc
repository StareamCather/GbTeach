// Wire-only Classic client: retain raw native messages; no motion predictions.
#include <gazebo/transport/transport.hh>
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

int main(int argc, char **argv) {
  bool initialized = false;
  int code = 0;
  // Keep owners alive on MAIN through transport stop/join. Releasing the last
  // publisher on an asynchronous publication worker can make its destructor
  // join that same worker. Callback targets must outlive the joined transport.
  Receiver receiver;
  transport::NodePtr node;
  std::vector<transport::PublisherPtr> publishers;
  std::vector<transport::SubscriberPtr> subscriptions;
  try {
    if (argc < 2) throw std::runtime_error("Use watch/snapshot/entity/stats/control");
    std::string mode(argv[1]);
    std::map<std::string, std::string> options;
    for (int i=2; i<argc; ++i) {
      std::string key(argv[i]);
      if (key == "--reset-models") options[key] = "1";
      else { if (key.substr(0,2) != "--" || i+1 >= argc) throw std::runtime_error("Bad option"); options[key] = argv[++i]; }
    }
    double timeout = options.count("--timeout") ? std::stod(options["--timeout"]) : 3;
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 8) throw std::runtime_error("Timeout must be within (0,8]");
    if (!transport::init("", 0, 3)) throw std::runtime_error("Private Classic master connection failed");
    initialized = true; transport::run();
    node.reset(new transport::Node());
    node->Init(options.count("--world") ? options["--world"] : "lesson26");
    if (mode == "watch") {
      double seconds = options.count("--seconds") ? std::stod(options["--seconds"]) : 3;
      if (!std::isfinite(seconds) || seconds <= 0 || seconds > 120) throw std::runtime_error("Watch 0..120 wall seconds");
      receiver.watching = true;
      auto poses = node->Subscribe("~/pose/info", &Receiver::Poses, &receiver);
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      auto plugin = node->Subscribe("~/model_plugin/state", &Receiver::Plugin, &receiver);
      subscriptions.insert(subscriptions.end(), {poses, stats, plugin});
      Print("{\"source\":\"client_ready\",\"receive_monotonic_ns\":"+std::to_string(Now())+"}");
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    } else if (mode == "stats") {
      auto stats = node->Subscribe("~/world_stats", &Receiver::Stats, &receiver);
      subscriptions.push_back(stats);
      { std::unique_lock<std::mutex> guard(receiver.mutex);
        if (!receiver.condition.wait_for(guard, std::chrono::duration<double>(timeout), [&]{return receiver.statsReady;}))
          throw std::runtime_error("No actual native world statistics");
        Print("{\"source\":\"world_stats\",\"receive_monotonic_ns\":"+std::to_string(Now())+
              ",\"data\":"+Json(receiver.statistics)+",\"raw_protobuf\":"+Q(receiver.statistics.DebugString())+"}"); }
    } else if (mode == "snapshot" || mode == "entity") {
      { std::lock_guard<std::mutex> guard(receiver.mutex);
        receiver.id = static_cast<int>((Now() & 0x3fffffff)+1);
        receiver.requestName = mode == "snapshot" ? "gazeboteach_model_snapshot" : "entity_info"; }
      const std::string nonce = "l26-"+std::to_string(Now());
      auto sub = node->Subscribe("~/response", &Receiver::Response, &receiver, true);
      auto pub = node->Advertise<msgs::Request>("~/request");
      subscriptions.push_back(sub);
      publishers.push_back(pub);
      if (!pub->WaitForConnection(common::Time(1,0))) throw std::runtime_error("No native request subscriber");
      msgs::Request query; query.set_id(receiver.id); query.set_request(receiver.requestName);
      query.set_data(mode == "snapshot" ? nonce : "rotor_lab"); pub->Publish(query);
      { std::unique_lock<std::mutex> guard(receiver.mutex);
        if (!receiver.condition.wait_for(guard, std::chrono::duration<double>(timeout), [&]{return receiver.responseReady;}))
          throw std::runtime_error("No matching native response before deadline"); }
      msgs::Response reply;
      { std::lock_guard<std::mutex> guard(receiver.mutex); reply.CopyFrom(receiver.response); }
      if (reply.response() != "success") throw std::runtime_error("Native request rejected");
      if (mode == "snapshot") {
        msgs::Any value;
        if (reply.type() != "gazebo.msgs.Any" || !value.ParseFromString(reply.serialized_data())) throw std::runtime_error("Wrong plugin reply type/bytes");
        const auto payload = AnyJson(value);
        google::protobuf::Struct decoded; google::protobuf::util::JsonStringToMessage(payload, &decoded);
        if (decoded.fields().at("query_nonce").string_value() != nonce) throw std::runtime_error("Plugin reply nonce mismatch");
        Print("{\"source\":\"snapshot\",\"request_id\":"+std::to_string(receiver.id)+",\"response_id\":"+
              std::to_string(reply.id())+",\"payload\":"+payload+",\"raw_protobuf\":"+Q(reply.DebugString())+"}");
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
        unsigned int count = std::stoul(options["--steps"]);
        if (count < 1 || count > 10000) throw std::runtime_error("Step count outside 1..10000");
        command.set_pause(true); command.set_multi_step(count); mutation = true;
      }
      if (options.count("--reset-models")) {
        // WorldReset.all defaults TRUE in Classic proto2; set it explicitly.
        command.mutable_reset()->set_all(false);
        command.mutable_reset()->set_time_only(false);
        command.mutable_reset()->set_model_only(true);
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
  if (initialized) transport::fini();
  if (node) node->Fini();
  subscriptions.clear();
  publishers.clear();
  node.reset();
  return code;
}
