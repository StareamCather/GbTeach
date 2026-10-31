// Real RaySensor callbacks feed a small, explicitly stateful distance filter.
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif
namespace gazebo {
class ProximitySensorPlugin final : public SensorPlugin {
  struct State {
    std::mutex mutex;
    bool active=true,available=false,hasFilter=false,hasPrevious=false;
    double threshold=.9,alpha=.35,nearest=0,filtered=0,previous=0,minimum=.1,maximum=6;
    long long stamp=0;
    unsigned long long loads=0,inits=0,resets=0,callbacks=0,epoch=0,rewinds=0,duplicates=0;
    std::string world,sensor,topic,configSHA,cache,hook="load",alarm="unknown";
    std::vector<double> ranges;
    size_t valid=0;
    transport::PublisherPtr publisher,response;
  };
  std::shared_ptr<State> state;
  transport::NodePtr node;
  transport::SubscriberPtr requests;
  event::ConnectionPtr updated;
  static std::string Q(const std::string &text) {
    std::ostringstream out;out<<'"';
    for(unsigned char c:text){if(c=='"'||c=='\\')out<<'\\';if(c=='\n')out<<"\\n";else if(c>=32)out<<c;}
    return out.str()+'"';
  }
  static void Capture(State &s) {
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"world\":"<<Q(s.world)<<",\"sensor_scoped_name\":"<<Q(s.sensor)<<",\"native_topic\":"<<Q(s.topic)
      <<",\"source_sha256\":"<<Q(GZTEACH_SOURCE_SHA)<<",\"config_sha256\":"<<Q(s.configSHA)
      <<",\"threshold_m\":"<<s.threshold<<",\"alpha\":"<<s.alpha
      <<",\"load_count\":"<<s.loads<<",\"init_count\":"<<s.inits<<",\"reset_count\":"<<s.resets
      <<",\"callback_count\":"<<s.callbacks<<",\"epoch\":"<<s.epoch<<",\"rewind_count\":"<<s.rewinds
      <<",\"duplicate_stamp_count\":"<<s.duplicates<<",\"source_time_ns\":"<<Q(std::to_string(s.stamp))
      <<",\"source_time_s\":"<<s.stamp*1e-9<<",\"sample_available\":"<<(s.available?"true":"false")
      <<",\"range_min_m\":"<<s.minimum<<",\"range_max_m\":"<<s.maximum
      <<",\"valid_count\":"<<s.valid<<",\"nearest_m\":";
    if(s.valid)out<<s.nearest;else out<<"null";
    out<<",\"previous_filtered_m\":";if(s.hasPrevious)out<<s.previous;else out<<"null";
    out<<",\"filtered_m\":";if(s.hasFilter)out<<s.filtered;else out<<"null";
    out<<",\"filter_initialized_from_current\":"<<(s.hasFilter&&!s.hasPrevious?"true":"false")
      <<",\"alarm_state\":"<<Q(s.alarm)<<",\"last_hook\":"<<Q(s.hook)<<",\"raw_ranges_m\":[";
    bool comma=false;
    for(double v:s.ranges){if(comma)out<<',';comma=true;if(std::isfinite(v))out<<v;else out<<"null";}
    out<<"],\"raw_kinds\":[";comma=false;
    for(double v:s.ranges){if(comma)out<<',';comma=true;out<<Q(std::isnan(v)?"nan":std::isinf(v)?(v>0?"positive_infinity":"negative_infinity"):"finite");}
    out<<"]}";s.cache=out.str();
  }
  static void Publish(State &s) {
    msgs::Any message;message.set_type(msgs::Any::STRING);message.set_string_value(s.cache);s.publisher->Publish(message);
  }
 public:
  ~ProximitySensorPlugin() override {
    if(state){std::lock_guard<std::mutex> lock(state->mutex);state->active=false;}
    updated.reset();requests.reset();if(node)node->Fini();node.reset();state.reset();
  }
  void Load(sensors::SensorPtr parent,sdf::ElementPtr sdf) override {
    auto sensor=std::dynamic_pointer_cast<sensors::RaySensor>(parent);
    if(!sensor)gzthrow("L28 requires an actual CPU RaySensor");
    state=std::make_shared<State>();auto shared=state;
    shared->threshold=sdf->Get<double>("threshold_m");shared->alpha=sdf->Get<double>("alpha");
    if(!std::isfinite(shared->threshold)||shared->threshold<.2||shared->threshold>4||
       !std::isfinite(shared->alpha)||shared->alpha<.05||shared->alpha>1)gzthrow("Invalid finite threshold/alpha");
    shared->configSHA=sdf->Get<std::string>("config_sha256");shared->world=sensor->WorldName();
    shared->sensor=sensor->ScopedName();shared->topic=sensor->Topic();
    shared->minimum=sensor->RangeMin();shared->maximum=sensor->RangeMax();
    node.reset(new transport::Node());node->Init(shared->world);
    shared->publisher=node->Advertise<msgs::Any>("~/sensor_plugin/state",200);
    shared->response=node->Advertise<msgs::Response>("~/response",100);
    shared->loads=1;Capture(*shared);
    requests=node->Subscribe<msgs::Request>("~/request",[shared](const boost::shared_ptr<const msgs::Request> &r){
      if(r->request()!="gazeboteach_sensor_snapshot")return;
      std::string copy;
      {std::lock_guard<std::mutex> lock(shared->mutex);if(!shared->active)return;copy=shared->cache;}
      auto close=copy.rfind('}');if(close==std::string::npos)return;
      copy.insert(close,",\"query_nonce\":"+Q(r->data()));
      msgs::Any payload;payload.set_type(msgs::Any::STRING);payload.set_string_value(copy);
      msgs::Response response;response.set_id(r->id());response.set_request(r->request());response.set_response("success");
      response.set_type("gazebo.msgs.Any");response.set_serialized_data(payload.SerializeAsString());shared->response->Publish(response);
    });
    std::weak_ptr<sensors::RaySensor> weak(sensor);
    updated=sensor->ConnectUpdated([shared,weak]{
      auto sensor=weak.lock();if(!sensor)return;
      // Sensor::Update emits this after a successful native UpdateImpl. Getter
      // calls precede our mutex; request callbacks only copy cached JSON.
      std::vector<double> values;sensor->Ranges(values);
      const auto time=sensor->LastMeasurementTime();const long long stamp=static_cast<long long>(time.sec)*1000000000LL+time.nsec;
      std::lock_guard<std::mutex> lock(shared->mutex);if(!shared->active)return;
      auto &s=*shared;
      if(s.available&&stamp<s.stamp){++s.epoch;++s.rewinds;s.hasFilter=false;}
      if(s.available&&stamp==s.stamp)++s.duplicates;
      s.hasPrevious=s.hasFilter;s.previous=s.filtered;
      s.ranges=values;s.valid=0;s.nearest=std::numeric_limits<double>::infinity();
      for(double value:values)if(std::isfinite(value)&&value>=s.minimum&&value<s.maximum){++s.valid;s.nearest=std::min(s.nearest,value);}
      if(s.valid){s.filtered=s.hasFilter?s.alpha*s.nearest+(1-s.alpha)*s.filtered:s.nearest;s.hasFilter=true;s.alarm=s.filtered<=s.threshold?"near":"clear";}
      else{s.hasFilter=false;s.hasPrevious=false;s.alarm="unknown";}
      s.stamp=stamp;s.available=true;++s.callbacks;s.hook="sensor_updated";Capture(s);Publish(s);
    });
    sensor->SetActive(true);
  }
  void Init() override {std::lock_guard<std::mutex> lock(state->mutex);++state->inits;state->hook="init";Capture(*state);}
  void Reset() override {
    // SensorManager normally resets sampling clocks, not this plugin hook.
    // Count an actual invocation if a different caller explicitly makes one.
    std::lock_guard<std::mutex> lock(state->mutex);++state->resets;state->hasFilter=false;state->hasPrevious=false;
    state->alarm="unknown";state->hook="reset";Capture(*state);
  }
};
GZ_REGISTER_SENSOR_PLUGIN(ProximitySensorPlugin)
}
