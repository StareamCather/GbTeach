// Process-scoped Classic11 SystemPlugin. No SDF <plugin> loads this class.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <unistd.h>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif
namespace gazebo {
class LifecycleSystemPlugin : public SystemPlugin {
 public:
  ~LifecycleSystemPlugin() override {
    updateConnection.reset(); resetConnection.reset(); createdConnection.reset();
    requestSub.reset(); if(node)node->Fini();
  }
  void Load(int argc, char ** /*argv*/) override {
    // Load precedes transport setup and world construction. Never wait for a
    // world here: that would prevent the server from constructing it.
    std::ifstream file("build/system_plugin.cfg"); std::map<std::string,std::string> input;
    std::string key,value;
    while(file>>key>>value) {
      if(!input.emplace(key,value).second)gzthrow("Duplicate personal SystemPlugin config key");
    }
    if(input.size()!=4 || input["world"]!="lesson31")gzthrow("Prepare the personal L31 config before gzserver");
    worldName=input["world"]; configSHA=input["config_sha256"];
    gravity=std::stod(input["gravity_m_s2"]); stride=std::stoul(input["record_every_steps"]);
    if(!std::isfinite(gravity)||gravity<1||gravity>20||stride<1||stride>100||configSHA.size()!=64)
      gzthrow("Invalid personal SystemPlugin parameters");
    argumentCount=argc; ++loads; Hook("load");
    createdConnection=event::Events::ConnectWorldCreated(
      std::bind(&LifecycleSystemPlugin::WorldCreated,this,std::placeholders::_1));
  }
  void Init() override {
    std::lock_guard<std::mutex> guard(mutex); ++inits; Hook("init");
  }
  void Reset() override {
    // Classic11 World::Reset iterates WorldPlugin objects, not process g_plugins.
    // Do not call this ourselves and pretend that the SDK dispatched it.
    std::lock_guard<std::mutex> guard(mutex); ++resets; Hook("system_plugin_reset");
    Capture("system_plugin_reset"); if(publisher)Publish();
  }
 private:
  std::string Q(const std::string &value) const {
    std::ostringstream out;out<<'"';
    for(unsigned char c:value){if(c=='"'||c=='\\')out<<'\\';if(c=='\n')out<<"\\n";else if(c>=32)out<<c;}
    return out.str()+'"';
  }
  void Hook(const std::string &name) {
    std::ostringstream out; out<<"{\"sequence\":"<<hooks.size()+1<<",\"hook\":"<<Q(name)
      <<",\"world_available\":"<<(world?"true":"false")
      <<",\"named_world_exists\":"<<(physics::has_world(worldName)?"true":"false")<<",\"updates\":"<<updates<<'}';
    // History is intentionally bounded during long interactive sessions.
    if(hooks.size()<128)hooks.push_back(out.str());
    gzmsg<<"L31 SystemPlugin "<<name<<" world_available="<<(world?"true":"false")<<"\n";
  }
  std::string Measurement(const common::UpdateInfo &info) {
    std::ostringstream out;out<<std::setprecision(17);
    const auto pose=cube->WorldPose();const auto velocity=cube->WorldLinearVel();
    out<<"{\"phase\":\"world_update_begin_prephysics\",\"updates\":"<<updates
      <<",\"epoch\":"<<epoch<<",\"epoch_updates\":"<<epochUpdates
      <<",\"sim_time_s\":"<<info.simTime.Double()<<",\"sim_time_ns\":"<<info.simTime.sec*1000000000LL+info.simTime.nsec
      <<",\"world_iterations\":"<<world->Iterations()<<",\"model_id\":"<<cube->GetId()
      <<",\"x_m\":"<<pose.Pos().X()<<",\"y_m\":"<<pose.Pos().Y()<<",\"z_m\":"<<pose.Pos().Z()
      <<",\"vz_m_s\":"<<velocity.Z()<<",\"actual_gravity_z\":"<<world->Gravity().Z()<<'}';
    return out.str();
  }
  void Capture(const std::string &hook) {
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"plugin_type\":\"SystemPlugin\",\"scope\":\"gzserver_process\",\"process_id\":"<<getpid()
      <<",\"world\":"<<Q(worldName)<<",\"source_sha256\":"<<Q(GZTEACH_SOURCE_SHA)
      <<",\"config_sha256\":"<<Q(configSHA)<<",\"argument_count\":"<<argumentCount
      <<",\"load_count\":"<<loads<<",\"init_count\":"<<inits<<",\"reset_count\":"<<resets
      <<",\"world_created_count\":"<<created<<",\"world_reset_event_count\":"<<resetEvents
      <<",\"updates\":"<<updates<<",\"epoch\":"<<epoch<<",\"epoch_updates\":"<<epochUpdates
      <<",\"record_count\":"<<records<<",\"epoch_record_count\":"<<epochRecords
      <<",\"gravity_m_s2\":"<<gravity<<",\"record_every_steps\":"<<stride
      <<",\"world_available\":"<<(world?"true":"false")
      <<",\"gravity_applied\":"<<(applied?"true":"false")
      <<",\"actual_gravity_z\":"<<(world?std::to_string(world->Gravity().Z()):"null")
      <<",\"last_hook\":"<<Q(hook)<<",\"last_error\":"<<Q(error)
      <<",\"last_prephysics\":"<<lastMeasurement<<",\"last_record\":"<<lastRecord
      <<",\"lifecycle\":[";
    bool comma=false;for(const auto &h:hooks){if(comma)out<<',';out<<h;comma=true;}
    out<<"],\"recent_records\":[";comma=false;
    for(const auto &r:recentRecords){if(comma)out<<',';out<<r;comma=true;}
    out<<"]}"; state=out.str();
  }
  void Publish(){msgs::Any value;value.set_type(msgs::Any::STRING);value.set_string_value(state);publisher->Publish(value);}
  void WorldCreated(const std::string &name) {
    if(name!=worldName)return;
    std::lock_guard<std::mutex> guard(mutex);
    if(world){error="Second owned world creation rejected";return;}
    world=physics::get_world(name); ++created;
    cube=world->ModelByName("falling_cube");
    if(!cube)gzthrow("L31 world requires the real falling_cube");
    cubeId=cube->GetId();
    node.reset(new transport::Node());node->Init(name);
    publisher=node->Advertise<msgs::Any>("~/system_plugin/state",100);
    response=node->Advertise<msgs::Response>("~/response",100);
    requestSub=node->Subscribe("~/request",&LifecycleSystemPlugin::Request,this);
    updateConnection=event::Events::ConnectWorldUpdateBegin(std::bind(&LifecycleSystemPlugin::Update,this,std::placeholders::_1));
    resetConnection=event::Events::ConnectWorldReset(std::bind(&LifecycleSystemPlugin::WorldReset,this));
    // World::Load emits worldCreated before World::Init. This lesson explicitly
    // chooses the first actual prephysics callback as its mutation boundary,
    // after world initialization and before the first ODE integration.
    Hook("world_created");Capture("world_created");Publish();
  }
  void WorldReset() {
    std::lock_guard<std::mutex> guard(mutex);
    // This process event has NO world argument. Count only when this world's
    // own observed clock has actually rewound. Never infer ownership from it.
    if(!world || lastTime<=0 || world->SimTime().Double()>=lastTime)return;
    ++resetEvents; ++epoch; epochUpdates=0;epochRecords=0;applied=false;lastTime=0;
    lastMeasurement="null";lastRecord="null";recentRecords.clear();
    Hook("owned_world_reset_event");Capture("owned_world_reset_event");Publish();
  }
  void Update(const common::UpdateInfo &info) {
    if(info.worldName!=worldName)return;
    std::lock_guard<std::mutex> guard(mutex);
    auto current=world->ModelByName("falling_cube");
    if(!current || current->GetId()!=cubeId){error="Owned cube removed or replaced; no mutation of replacement";Capture("ownership_error");return;}
    // Source-time rewind also covers a native time-only reset, which does not
    // dispatch worldReset. Reset() itself remains an honest SDK call counter.
    if(info.simTime.Double()<lastTime){++epoch;epochUpdates=0;epochRecords=0;applied=false;recentRecords.clear();lastRecord="null";Hook("owned_clock_rewind");}
    if(!applied){world->SetGravity(ignition::math::Vector3d(0,0,-gravity));applied=true;}
    ++updates;++epochUpdates;
    if(updates==1)Hook("first_world_update_begin");
    lastMeasurement=Measurement(info);
    const bool record=epochUpdates%stride==0;
    if(record){++records;++epochRecords;lastRecord=lastMeasurement;recentRecords.push_back(lastRecord);if(recentRecords.size()>128)recentRecords.erase(recentRecords.begin());}
    lastTime=info.simTime.Double();Capture("world_update_begin");
    if(record||epochUpdates==1)Publish();
  }
  void Request(ConstRequestPtr &request) {
    if(request->request()!="gazeboteach_system_snapshot")return;
    std::string copy;
    {std::lock_guard<std::mutex> guard(mutex);copy=state;}
    const auto close=copy.rfind('}');if(close==std::string::npos)return;
    copy.insert(close,",\"query_nonce\":"+Q(request->data()));
    msgs::Any value;value.set_type(msgs::Any::STRING);value.set_string_value(copy);
    msgs::Response reply;reply.set_id(request->id());reply.set_request(request->request());reply.set_response("success");
    reply.set_type("gazebo.msgs.Any");reply.set_serialized_data(value.SerializeAsString());response->Publish(reply);
  }
  physics::WorldPtr world;physics::ModelPtr cube;unsigned int cubeId=0;
  transport::NodePtr node;transport::PublisherPtr publisher,response;transport::SubscriberPtr requestSub;
  event::ConnectionPtr createdConnection,updateConnection,resetConnection;std::mutex mutex;
  std::string worldName,configSHA,state,error,lastMeasurement="null",lastRecord="null";
  std::vector<std::string> hooks,recentRecords;
  double gravity=9.8,lastTime=0;unsigned int stride=10;int argumentCount=0;bool applied=false;
  unsigned long long loads=0,inits=0,resets=0,created=0,resetEvents=0,updates=0,epoch=0,epochUpdates=0,records=0,epochRecords=0;
};
GZ_REGISTER_SYSTEM_PLUGIN(LifecycleSystemPlugin)
}
