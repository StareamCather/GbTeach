// WorldPlugin: queue offline SDF factories, observe actual IDs, then reclaim.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif
namespace gazebo {
class BatchWorldPlugin : public WorldPlugin {
  struct Item {
    std::string name;
    unsigned int id = 0, index = 0;
    double requested = 0, observed = 0, z = 0;
    bool appeared = false, deleteSent = false, removed = false;
  };
 public:
  ~BatchWorldPlugin() override {
    connection.reset(); requestSub.reset(); if (node) node->Fini();
  }
  void Load(physics::WorldPtr parent, sdf::ElementPtr sdf) override {
    world = parent;
    count = sdf->Get<int>("count"); height = sdf->Get<double>("drop_height");
    prefix = sdf->Get<std::string>("session_prefix"); configSHA = sdf->Get<std::string>("config_sha256");
    if (count < 1 || count > 5 || !std::isfinite(height) || height < .6 || height > 2 || prefix.substr(0,5) != "gt27_")
      gzthrow("L27 requires finite personal SDF inputs and a private prefix");
    node.reset(new transport::Node()); node->Init(world->Name());
    publisher = node->Advertise<msgs::Any>("~/world_plugin/state",100);
    response = node->Advertise<msgs::Response>("~/response",100);
    ++loads; Capture("load");
    requestSub = node->Subscribe("~/request", &BatchWorldPlugin::Request, this);
    connection = event::Events::ConnectWorldUpdateBegin(std::bind(&BatchWorldPlugin::Update,this,std::placeholders::_1));
  }
  void Init() override { std::lock_guard<std::mutex> lock(mutex); ++inits; Capture("init"); Publish(); }
  void Reset() override {
    // World::Reset holds the physics mutex. Never reenter model deletion here.
    std::lock_guard<std::mutex> lock(mutex);
    ++resets; epochUpdates = 0; clearRequested = true; batchRequested = false;
    phase = "clearing"; nextPublish = 0; Capture("reset"); Publish();
  }
 private:
  std::string Q(const std::string &v) const {
    std::ostringstream out; out << '"';
    for (unsigned char c:v) { if(c=='"'||c=='\\')out<<'\\'; if(c=='\n')out<<"\\n";else if(c>=32)out<<c; }
    return out.str()+'"';
  }
  void Capture(const std::string &hook) {
    std::ostringstream out; out << std::setprecision(17)
      << "{\"world\":"<<Q(world->Name())<<",\"source_sha256\":"<<Q(GZTEACH_SOURCE_SHA)
      <<",\"config_sha256\":"<<Q(configSHA)<<",\"session_prefix\":"<<Q(prefix)
      <<",\"load_count\":"<<loads<<",\"init_count\":"<<inits<<",\"reset_count\":"<<resets
      <<",\"updates\":"<<updates<<",\"epoch_updates\":"<<epochUpdates
      <<",\"sim_time_s\":"<<world->SimTime().Double()<<",\"count\":"<<count
      <<",\"drop_height\":"<<height<<",\"interval_s\":0.35,\"phase\":"<<Q(phase)
      <<",\"batch_number\":"<<batches<<",\"requested\":"<<items.size()
      <<",\"observed\":"<<observedCount<<",\"last_error\":"<<Q(error)
      <<",\"last_hook\":"<<Q(hook)<<",\"pending_names\":[";
    bool comma=false;
    for(const auto &i:items) if(!i.appeared) {if(comma)out<<',';out<<Q(i.name);comma=true;}
    out<<"],\"models\":[";comma=false;
    for(const auto &i:items) {if(comma)out<<',';comma=true;
      out<<"{\"name\":"<<Q(i.name)<<",\"id\":"<<i.id<<",\"index\":"<<i.index
         <<",\"insert_requested_time_s\":"<<i.requested<<",\"first_observed_time_s\":"<<i.observed
         <<",\"z\":"<<i.z<<",\"appeared\":"<<(i.appeared?"true":"false")
         <<",\"removed\":"<<(i.removed?"true":"false")<<'}'; }
    out<<"]}"; state=out.str();
  }
  void Publish() { msgs::Any m;m.set_type(msgs::Any::STRING);m.set_string_value(state);publisher->Publish(m); }
  void Insert(double now) {
    const unsigned int index=items.size();
    const std::string name=prefix+"_b"+std::to_string(batches)+"_i"+std::to_string(index);
    if(world->ModelByName(name)) {error="Private requested name already exists; no ownership assumed";phase="error";return;}
    const double x=(static_cast<double>(index)-(count-1)*.5)*.35;
    std::ostringstream text; text<<std::setprecision(17)
      <<"<sdf version='1.6'><model name='"<<name<<"'><pose>"<<x<<" 0 "<<height<<" 0 0 0</pose>"
      <<"<link name='block'><inertial><mass>0.5</mass><inertia><ixx>0.0021333333333333334</ixx><iyy>0.0021333333333333334</iyy><izz>0.0021333333333333334</izz></inertia></inertial>"
      <<"<collision name='body'><geometry><box><size>.16 .16 .16</size></box></geometry>"
      <<"<surface><bounce><restitution_coefficient>0</restitution_coefficient></bounce></surface></collision>"
      <<"<visual name='body'><geometry><box><size>.16 .16 .16</size></box></geometry><material><ambient>"
      <<(index%2?".15 .65 .9 1":".95 .4 .12 1")<<"</ambient><diffuse>"<<(index%2?".15 .65 .9 1":".95 .4 .12 1")
      <<"</diffuse></material></visual></link></model></sdf>";
    // Preserve the locally formatted 17-digit raw SDF at the factory entry.
    // This is NOT an end-to-end precision fix: SDK9's subsequent Root()->Clone()
    // clones Param through GetAsString (six significant figures), so actual
    // inertias must still be read back. InsertModelSDF adds an earlier ToString.
    // Both paths are offline and asynchronous; existence is NOT assumed here.
    world->InsertModelString(text.str());
    Item item;item.name=name;item.index=index;item.requested=now;items.push_back(item);
    nextInsert=now+.35;
  }
  void Update(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex);
    ++updates;++epochUpdates;const double now=info.simTime.Double();
    for(auto &i:items) {
      auto actual=world->ModelByName(i.name);
      if(!i.appeared && actual) {i.appeared=true;i.id=actual->GetId();i.observed=now;++observedCount;}
      if(i.appeared && !i.removed) {
        if(!actual) {i.removed=true;continue;}
        if(actual->GetId()!=i.id) {i.removed=true;error="Owned ID disappeared; replacement is external and will not be deleted";continue;}
        i.z=actual->WorldPose().Pos().Z();
        if(clearRequested && !i.deleteSent) {world->RemoveModel(actual);i.deleteSent=true;}
      }
    }
    if(clearRequested) {
      phase="clearing";bool done=true;for(auto &i:items)if(!i.appeared||!i.removed)done=false;
      if(done){items.clear();observedCount=0;clearRequested=false;phase="idle";}
    } else if(batchRequested) {
      batchRequested=false; ++batches; nextInsert=now;phase="spawning";
    }
    if(phase=="spawning") {
      if(items.size()<static_cast<size_t>(count) && now+1e-9>=nextInsert)Insert(now);
      if(items.size()==static_cast<size_t>(count) && observedCount==count)phase="complete";
    }
    Capture("world_update_begin");
    if(now+1e-10>=nextPublish || now<lastTime){Publish();nextPublish=now+.02;}lastTime=now;
  }
  void Request(ConstRequestPtr &r) {
    const std::string verb=r->request();
    if(verb!="gazeboteach_world_snapshot"&&verb!="gazeboteach_world_batch"&&verb!="gazeboteach_world_clear")return;
    std::string copy;bool accepted=true;
    {std::lock_guard<std::mutex> lock(mutex);
      if(verb!="gazeboteach_world_snapshot") {
        const std::string identity=verb+":"+r->data();
        if(receipts.count(identity)==0) {
          if(verb=="gazeboteach_world_batch") {
            if(phase!="idle"||!items.empty()||clearRequested||batchRequested)accepted=false;
            else batchRequested=true;
          } else {clearRequested=true;batchRequested=false;}
          if(accepted) {receipts.insert(identity);if(receipts.size()>1024)receipts.erase(receipts.begin());}
        }
        // Snapshot remains the last real callback; command acceptance is separate.
      }
      copy=state;
    }
    const auto closing=copy.rfind('}');if(closing==std::string::npos)return;
    copy.insert(closing,",\"accepted\":"+std::string(accepted?"true":"false")+",\"query_nonce\":"+Q(r->data()));
    msgs::Any payload;payload.set_type(msgs::Any::STRING);payload.set_string_value(copy);
    msgs::Response reply;reply.set_id(r->id());reply.set_request(verb);reply.set_response(accepted?"success":"error");
    reply.set_type("gazebo.msgs.Any");reply.set_serialized_data(payload.SerializeAsString());response->Publish(reply);
  }
  physics::WorldPtr world;transport::NodePtr node;transport::PublisherPtr publisher,response;
  transport::SubscriberPtr requestSub;event::ConnectionPtr connection;std::mutex mutex;
  std::vector<Item> items;std::set<std::string> receipts;
  std::string prefix,configSHA,state,error,phase="idle";
  int count=3;double height=1.2,nextInsert=0,nextPublish=0,lastTime=0;
  unsigned long long loads=0,inits=0,resets=0,updates=0,epochUpdates=0,batches=0;
  unsigned int observedCount=0;bool clearRequested=false,batchRequested=false;
};
GZ_REGISTER_WORLD_PLUGIN(BatchWorldPlugin)
}
