// Read actual server state and count successful native camera Update events.
// No mode/rate changes, sensor forcing, ROS, or fabricated measurements.
#include <memory>
#include <mutex>
#include <ctime>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/transport/transport.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

namespace gazebo
{
class GazeboTeachL21Metadata final : public WorldPlugin
{
  struct Sample
  {
    bool hookAttached = false;
    uint64_t count = 0, same = 0, regressed = 0;
    int64_t hookSim = 0, hookWall = 0, first = 0, last = 0, lastWall = 0;
  };
  struct Counter
  {
    std::mutex mutex;
    bool active = true;
    Sample sample;
  };
  struct State
  {
    std::mutex lifecycle;
    bool active = true;
    physics::WorldPtr world;
    transport::PublisherPtr publisher;
    event::ConnectionPtr cameraConnection;
    std::shared_ptr<Counter> counter{new Counter};
    int64_t previousWall = 0;
  };
  std::shared_ptr<State> state_;
  transport::NodePtr node_;
  event::ConnectionPtr update_;

  static int64_t Nanos(const common::Time &value)
  {
    return int64_t(value.sec)*1000000000LL+value.nsec;
  }
  static int64_t Monotonic()
  {
    timespec clock;
    clock_gettime(CLOCK_MONOTONIC, &clock);
    return int64_t(clock.tv_sec)*1000000000LL+clock.tv_nsec;
  }

  static void AttachCamera(const std::shared_ptr<State> &state,
                           const sensors::SensorPtr &sensor)
  {
    if (state->cameraConnection) return;
    const auto counter = state->counter;
    const auto hookSim = Nanos(state->world->SimTime());
    const auto hookWall = Monotonic();
    {
      std::lock_guard<std::mutex> lock(counter->mutex);
      counter->sample.hookSim = hookSim;
      counter->sample.hookWall = hookWall;
    }
    std::weak_ptr<sensors::Sensor> weakSensor(sensor);
    state->cameraConnection = sensor->ConnectUpdated([counter, weakSensor]()
    {
      {std::lock_guard<std::mutex> lock(counter->mutex); if (!counter->active) return;}
      const auto sensor = weakSensor.lock();
      if (!sensor) return;
      // Sensor::Update emits this only after UpdateImpl succeeds. Read this
      // sensor's getter before our short mutex; never call scheduling methods.
      const auto measurement = Nanos(sensor->LastMeasurementTime());
      const auto wall = Monotonic();
      std::lock_guard<std::mutex> lock(counter->mutex);
      if (!counter->active) return;
      auto &sample = counter->sample;
      if (sample.count == 0) sample.first = measurement;
      else
      {
        if (measurement == sample.last) ++sample.same;
        if (measurement < sample.last) ++sample.regressed;
      }
      ++sample.count;
      sample.last = measurement;
      sample.lastWall = wall;
    });
    std::lock_guard<std::mutex> lock(counter->mutex);
    counter->sample.hookAttached = bool(state->cameraConnection);
  }

  static void Update(const std::shared_ptr<State> &state)
  {
    // A late world callback touches shared state only; shutdown waits for an
    // active publisher to finish, then marks it inactive before disconnection.
    std::lock_guard<std::mutex> lifetime(state->lifecycle);
    if (!state->active) return;
    const auto wall = Monotonic();
    if (wall-state->previousWall < 100000000LL) return;
    state->previousWall = wall;
    const auto now = state->world->SimTime();
    google::protobuf::Struct data;
    auto &f = *data.mutable_fields();
    f["rendering_lockstep_enabled"].set_bool_value(rendering::lockstep_enabled());
    f["world"].set_string_value(state->world->Name());
    f["world_sim_time_ns"].set_string_value(std::to_string(Nanos(now)));
    f["iterations"].set_string_value(std::to_string(state->world->Iterations()));
    f["max_step_size_s"].set_number_value(state->world->Physics()->GetMaxStepSize());
    f["real_time_update_rate_hz"].set_number_value(state->world->Physics()->GetRealTimeUpdateRate());
    for (const auto &name : {"camera", "imu"})
    {
      auto sensor = sensors::get_sensor(state->world->Name()+"::sync_rig::link::"+name);
      if (!sensor) continue;
      auto &entry = *f[name].mutable_struct_value()->mutable_fields();
      entry["strict_rate"].set_bool_value(sensor->StrictRate());
      entry["update_rate_hz"].set_number_value(sensor->UpdateRate());
      entry["scoped_name"].set_string_value(sensor->ScopedName());
      if (std::string(name) == "camera")
      {
        AttachCamera(state, sensor);
        Sample sample;
        {
          std::lock_guard<std::mutex> lock(state->counter->mutex);
          sample = state->counter->sample;
        }
        auto &p = *entry["producer_update"].mutable_struct_value()->mutable_fields();
        p["hook_attached"].set_bool_value(sample.hookAttached);
        p["hook_requested_world_sim_time_ns"].set_string_value(std::to_string(sample.hookSim));
        p["hook_requested_monotonic_ns"].set_string_value(std::to_string(sample.hookWall));
        p["event_count"].set_string_value(std::to_string(sample.count));
        p["first_measurement_time_ns"].set_string_value(std::to_string(sample.first));
        p["last_measurement_time_ns"].set_string_value(std::to_string(sample.last));
        p["producer_last_callback_monotonic_ns"].set_string_value(std::to_string(sample.lastWall));
        p["same_stamp_count"].set_string_value(std::to_string(sample.same));
        p["regressed_stamp_count"].set_string_value(std::to_string(sample.regressed));
      }
    }
    msgs::Any message;
    message.set_type(msgs::Any::STRING);
    std::string json;
    if (google::protobuf::util::MessageToJsonString(data, &json).ok())
    {
      message.set_string_value(json);
      state->publisher->Publish(message);
    }
  }

  public: void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    state_.reset(new State);
    state_->world = world;
    node_.reset(new transport::Node);
    node_->Init(world->Name());
    state_->publisher = node_->Advertise<msgs::Any>("~/lesson21/runtime_metadata", 20);
    const auto shared = state_;
    update_ = event::Events::ConnectWorldUpdateEnd([shared]() {Update(shared);});
  }

  public: ~GazeboTeachL21Metadata() override
  {
    if (!state_) return;
    {std::lock_guard<std::mutex> lock(state_->lifecycle); state_->active = false;}
    {std::lock_guard<std::mutex> lock(state_->counter->mutex); state_->counter->active = false;}
    update_.reset();
    state_->cameraConnection.reset();
    state_->publisher.reset();
    if (node_) node_->Fini();
    node_.reset();
    state_.reset();
  }
};
GZ_REGISTER_WORLD_PLUGIN(GazeboTeachL21Metadata)
}
