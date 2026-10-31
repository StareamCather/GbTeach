// First ModelPlugin: read a real joint, apply a torque, let the engine integrate.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <cmath>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif

namespace gazebo {
class RotorModelPlugin : public ModelPlugin {
 public:
  ~RotorModelPlugin() override {
    updateConnection.reset();
    requestSubscription.reset();
    if (node) node->Fini();
  }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override {
    model = parent;
    world = parent->GetWorld();
    jointName = sdf->Get<std::string>("joint");
    drive = sdf->Get<double>("drive_torque");
    drag = sdf->Get<double>("linear_drag");
    configSHA = sdf->Get<std::string>("config_sha256");
    joint = model->GetJoint(jointName);
    if (!joint || !std::isfinite(drive) || !std::isfinite(drag) || drag <= 0 ||
        std::abs(drive) > .12 || drag > .12) {
      gzthrow("L26 needs its actual joint, finite drive and positive drag");
    }
    node.reset(new transport::Node());
    node->Init(world->Name());
    statePublisher = node->Advertise<msgs::Any>("~/model_plugin/state", 100);
    responsePublisher = node->Advertise<msgs::Response>("~/response", 100);
    ++loadCount;
    epochStart = world->SimTime().Double();
    { std::lock_guard<std::mutex> guard(mutex); Capture("load", 0.0); }
    requestSubscription = node->Subscribe("~/request", &RotorModelPlugin::OnRequest, this);
    updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RotorModelPlugin::OnUpdate, this, std::placeholders::_1));
  }

  void Init() override {
    std::lock_guard<std::mutex> guard(mutex);
    ++initCount;
    Capture("init", 0.0);
    Publish();
  }

  void Reset() override {
    std::lock_guard<std::mutex> guard(mutex);
    ++resetCount;
    epochUpdates = 0;
    epochStart = world->SimTime().Double();
    nextPublish = epochStart;
    // Model-only reset clears physics velocities but can retain internal angles.
    // Observe that native result; never invent a zero angle or use state setters.
    Capture("reset", 0.0);
    Publish();
  }

 private:
  std::string Quote(const std::string &value) const {
    std::ostringstream out; out << '"';
    for (char c : value) {
      if (c == '"' || c == '\\') out << '\\';
      if (c == '\n') out << "\\n"; else if (c >= 32) out << c;
    }
    return out.str() + '"';
  }

  void Capture(const std::string &hook, double force) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"world\":" << Quote(world->Name())
        << ",\"model\":" << Quote(model->GetName())
        << ",\"joint\":" << Quote(joint->GetScopedName())
        << ",\"source_sha256\":" << Quote(GZTEACH_SOURCE_SHA)
        << ",\"config_sha256\":" << Quote(configSHA)
        << ",\"sim_time_s\":" << world->SimTime().Double()
        << ",\"epoch_time_s\":" << epochStart
        << ",\"omega_rad_s\":" << joint->GetVelocity(0)
        << ",\"q_rad\":" << joint->Position(0)
        << ",\"drive_torque\":" << drive << ",\"drag\":" << drag
        << ",\"applied_torque_nm\":" << force
        << ",\"torque_command_issued\":" << (hook == "update" ? "true" : "false")
        << ",\"load_count\":" << loadCount << ",\"init_count\":" << initCount
        << ",\"reset_count\":" << resetCount << ",\"updates\":" << updates
        << ",\"epoch_updates\":" << epochUpdates
        << ",\"last_hook\":" << Quote(hook)
        << ",\"capture_hook\":" << Quote(hook == "update" ? "world_update_begin" : hook == "reset" ? "reset_after_joint_reset" : hook)
        << ",\"observation_phase\":" << Quote(hook == "update" ? "callback_before_physics" : hook == "reset" ? "after_native_joint_reset" : "initialization") << "} ";
    state = out.str();
  }

  void Publish() {
    msgs::Any message;
    message.set_type(msgs::Any::STRING);
    message.set_string_value(state);
    statePublisher->Publish(message);
  }

  void OnUpdate(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> guard(mutex);
    const double omega = joint->GetVelocity(0);
    const double force = drive - drag * omega;
    joint->SetForce(0, force);
    ++updates;
    ++epochUpdates;
    Capture("update", force);
    const double now = info.simTime.Double();
    if (now + 1e-10 >= nextPublish || now < lastTime) {
      Publish();
      nextPublish = now + .02;
    }
    lastTime = now;
  }

  void OnRequest(ConstRequestPtr &request) {
    if (request->request() != "gazeboteach_model_snapshot") return;
    std::string copy;
    { std::lock_guard<std::mutex> guard(mutex); copy = state; }
    // Reply from an actual stored physics callback, even for a paused client.
    // The source sample timestamp is kept; receipt time is not substituted.
    const auto closing = copy.rfind('}');
    if (closing == std::string::npos) return;
    copy.insert(closing, ",\"query_nonce\":" + Quote(request->data()));
    msgs::Any payload;
    payload.set_type(msgs::Any::STRING);
    payload.set_string_value(copy);
    msgs::Response response;
    response.set_id(request->id()); response.set_request(request->request());
    response.set_response("success"); response.set_type("gazebo.msgs.Any");
    response.set_serialized_data(payload.SerializeAsString());
    responsePublisher->Publish(response);
  }

  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::JointPtr joint;
  transport::NodePtr node;
  transport::PublisherPtr statePublisher, responsePublisher;
  transport::SubscriberPtr requestSubscription;
  event::ConnectionPtr updateConnection;
  std::mutex mutex;
  std::string jointName, configSHA, state;
  double drive = 0, drag = 0, epochStart = 0, nextPublish = 0, lastTime = 0;
  unsigned long long loadCount = 0, initCount = 0, resetCount = 0, updates = 0, epochUpdates = 0;
};
GZ_REGISTER_MODEL_PLUGIN(RotorModelPlugin)
}
