// Native transport instrument. Only world multi-step is written; no force/PID.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/util/json_util.h>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unistd.h>

std::mutex stateMutex;
std::condition_variable changed;
gazebo::msgs::WorldStatistics currentStats;
gazebo::msgs::Response currentReply;
bool haveStats = false;
int expectedReplyId = -1;

void OnStats(ConstWorldStatisticsPtr &message)
{
  std::lock_guard<std::mutex> lock(stateMutex);
  currentStats = *message; haveStats = true; changed.notify_all();
}
void OnReply(ConstResponsePtr &message)
{
  std::lock_guard<std::mutex> lock(stateMutex);
  if (message->id() != expectedReplyId) return;
  currentReply = *message; changed.notify_all();
}
std::string Json(const google::protobuf::Message &message)
{
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  std::string result;
  if (!google::protobuf::util::MessageToJsonString(message, &result, options).ok())
    throw std::runtime_error("Cannot serialize native message");
  return result;
}
std::string Short(const std::string &name)
{
  auto position = name.rfind("::");
  return position == std::string::npos ? name : name.substr(position+2);
}
const gazebo::msgs::Joint &Joint(const gazebo::msgs::Model &model, const std::string &name)
{
  for (const auto &joint : model.joint()) if (Short(joint.name()) == name) return joint;
  throw std::runtime_error("Actual joint missing: " + name);
}
const gazebo::msgs::Link &Link(const gazebo::msgs::Model &model, const std::string &name)
{
  for (const auto &link : model.link()) if (Short(link.name()) == name) return link;
  throw std::runtime_error("Actual link missing: " + name);
}
void Pose(const gazebo::msgs::Pose &pose)
{
  const auto &p = pose.position(); const auto &q = pose.orientation();
  std::cout << '[' << p.x() << ',' << p.y() << ',' << p.z() << ','
            << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
}
void Stats(const gazebo::msgs::WorldStatistics &stats)
{
  const double t = stats.sim_time().sec()+stats.sim_time().nsec()*1e-9;
  std::cout << "{\"iterations\":" << stats.iterations() << ",\"sim_time_s\":" << t
            << ",\"paused\":" << (stats.paused() ? "true" : "false") << '}';
}

int main(int argc, char **argv)
{
  if (argc < 3) { std::cerr << "Expected WORLD inspect|sample [steps count wall_seconds]\n"; return 2; }
  const std::string world = argv[1], mode = argv[2];
  if (mode != "inspect" && mode != "sample") return 2;
  int steps = 0, count = 0; double wallLimit = 12;
  if (mode == "sample")
  {
    if (argc != 6) return 2;
    steps = std::stoi(argv[3]); count = std::stoi(argv[4]); wallLimit = std::stod(argv[5]);
    if (steps < 1 || steps > 200 || count < 20 || count > 180 || wallLimit < 10 || wallLimit > 65) return 2;
  }
  const auto deadline = std::chrono::steady_clock::now()+std::chrono::milliseconds(static_cast<int>(wallLimit*1000));
  if (!gazebo::client::setup()) return 2;
  int status = 1;
  try
  {
    gazebo::transport::NodePtr node(new gazebo::transport::Node()); node->Init(world);
    auto responseSub = node->Subscribe("~/response", OnReply);
    auto statsSub = node->Subscribe("~/world_stats", OnStats);
    auto requestPub = node->Advertise<gazebo::msgs::Request>("~/request");
    auto controlPub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    if (!requestPub->WaitForConnection(gazebo::common::Time(2,0)) ||
        !controlPub->WaitForConnection(gazebo::common::Time(2,0)))
      throw std::runtime_error("Native request/control subscriber discovery timeout");
    {
      std::unique_lock<std::mutex> lock(stateMutex);
      if (!changed.wait_for(lock, std::chrono::seconds(2), []{return haveStats;}))
        throw std::runtime_error("No actual world_stats");
    }
    int requestId = (getpid()%10000)*10000;
    auto request = [&](const std::string &kind, const std::string &data)
    {
      if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Native instrument deadline");
      gazebo::msgs::Request query; query.set_id(++requestId); query.set_request(kind); query.set_data(data);
      { std::lock_guard<std::mutex> lock(stateMutex); expectedReplyId = requestId; currentReply.Clear(); }
      requestPub->Publish(query, true);
      std::unique_lock<std::mutex> lock(stateMutex);
      if (!changed.wait_for(lock, std::chrono::milliseconds(1500), [&]{return currentReply.has_id() && currentReply.id() == requestId;}))
        throw std::runtime_error("Native read-only response timeout");
      auto reply = currentReply;
      if (reply.request() != kind || reply.response() != "success") throw std::runtime_error("Native request failed");
      return reply;
    };
    auto model = [&](const std::string &name)
    {
      auto reply = request("entity_info", name); gazebo::msgs::Model result;
      if (reply.type() != result.GetTypeName() || !result.ParseFromString(reply.serialized_data()) || Short(result.name()) != name)
        throw std::runtime_error("Native Model response type/name mismatch");
      return result;
    };
    auto physicsReply = request("physics_info", ""); gazebo::msgs::Physics physics;
    if (physicsReply.type() != physics.GetTypeName() || !physics.ParseFromString(physicsReply.serialized_data()))
      throw std::runtime_error("Native physics_info parse failure");
    auto pendulum = model("pendulum_rig"), slider = model("slider_rig");
    std::cout << std::setprecision(17);
    std::cout << "{\"kind\":\"initial\",\"physics\":" << Json(physics)
              << ",\"pendulum_model\":" << Json(pendulum) << ",\"slider_model\":" << Json(slider) << "}\n" << std::flush;
    if (mode == "sample")
    {
      gazebo::msgs::WorldStatistics baseline;
      { std::lock_guard<std::mutex> lock(stateMutex); baseline = currentStats; }
      if (!baseline.paused()) throw std::runtime_error("Sample mode requires paused world");
      for (int index = 0; index <= count; ++index)
      {
        gazebo::msgs::WorldStatistics observed;
        if (index > 0)
        {
          uint64_t target;
          { std::lock_guard<std::mutex> lock(stateMutex); target = currentStats.iterations()+steps; }
          gazebo::msgs::WorldControl control; control.set_pause(true); control.set_multi_step(steps);
          controlPub->Publish(control, true);  // Never reissue a state-changing request.
          std::unique_lock<std::mutex> lock(stateMutex);
          if (!changed.wait_for(lock, std::chrono::seconds(2), [&]{return currentStats.iterations() >= target;}))
            throw std::runtime_error("Actual multi-step acknowledgement timeout");
          observed = currentStats;
          if (observed.iterations() != target || !observed.paused()) throw std::runtime_error("Unexpected native step count/pause");
          lock.unlock(); pendulum = model("pendulum_rig"); slider = model("slider_rig");
        }
        else observed = baseline;
        const auto &swing = Joint(pendulum, "swing_joint"), &slide = Joint(slider, "slide_joint");
        if (swing.angle_size() != 1 || slide.angle_size() != 1 || !std::isfinite(swing.angle(0)) || !std::isfinite(slide.angle(0)))
          throw std::runtime_error("Actual joint Position missing/nonfinite");
        std::cout << "{\"kind\":\"sample\",\"stats\":"; Stats(observed);
        std::cout << ",\"pendulum_q_rad\":" << swing.angle(0) << ",\"slider_q_m\":" << slide.angle(0);
        std::cout << ",\"pendulum_model_pose\":"; Pose(pendulum.pose());
        std::cout << ",\"pendulum_frame_pose\":"; Pose(Link(pendulum,"frame").pose());
        std::cout << ",\"pendulum_bob_pose\":"; Pose(Link(pendulum,"bob").pose());
        std::cout << ",\"slider_model_pose\":"; Pose(slider.pose());
        std::cout << ",\"slider_guide_pose\":"; Pose(Link(slider,"guide").pose());
        std::cout << ",\"slider_carriage_pose\":"; Pose(Link(slider,"carriage").pose());
        std::cout << "}\n" << std::flush;
      }
    }
    status = 0;
  }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; }
  gazebo::client::shutdown(); return status;
}
