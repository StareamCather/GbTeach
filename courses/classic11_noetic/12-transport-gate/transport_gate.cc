// Classic transport callbacks only validate/copy cached data. Native
// JointController and physics owner callbacks alone touch the force-driven rig.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace gazebo {
namespace {
std::string Env(const char *k){const char *v=std::getenv(k);if(!v)throw std::runtime_error(std::string("Missing ")+k);return v;}
std::string Q(const std::string &v){std::ostringstream o;o<<'"';for(unsigned char c:v){if(c=='"'||c=='\\')o<<'\\';if(c=='\n')o<<"\\n";else if(c=='\r')o<<"\\r";else if(c=='\t')o<<"\\t";else if(c>=32)o<<c;}o<<'"';return o.str();}
std::string Vec(const ignition::math::Vector3d &v){std::ostringstream o;o<<std::setprecision(17)<<'['<<v.X()<<','<<v.Y()<<','<<v.Z()<<']';return o.str();}
std::string Pose(const ignition::math::Pose3d &v){std::ostringstream o;o<<std::setprecision(17)<<'['<<v.Pos().X()<<','<<v.Pos().Y()<<','<<v.Pos().Z()<<','<<v.Rot().W()<<','<<v.Rot().X()<<','<<v.Rot().Y()<<','<<v.Rot().Z()<<']';return o.str();}
bool Safe(const std::string &v,size_t n){return !v.empty()&&v.size()<=n&&v.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")==std::string::npos;}
struct Command {std::string run,client,nonce,attempt;long long epoch=-1;int seq=0;double target=0;bool targetValid=false;};
struct Snapshot {unsigned long long iteration=0;double time=0,q=0,v=0,force=0,cmd=0,pe=0,ie=0,de=0;ignition::math::Pose3d pose;ignition::math::Vector3d axis{1,0,0};std::string stage="initial";};
struct State {
  std::mutex mutex;bool closing=false,pending=false,active=false,needApplied=false;
  physics::ModelPtr model;physics::WorldPtr world;physics::LinkPtr panel;physics::JointPtr joint;physics::JointControllerPtr controller;
  transport::NodePtr node;transport::PublisherPtr ackPub,statePub,responsePub;transport::SubscriberPtr commandSub,requestSub;
  std::string run,worldName,directory,scoped,configSha,worldSha,sourceSha,clientSha,modelJSON,error,stage="idle";
  std::map<std::string,double> parameters;std::map<std::string,int> sequences;std::set<std::string> nonces;
  unsigned long long epoch=0,received=0,applied=0,completed=0,rejected=0,resets=0,step=0;
  double targetApplied=0,consecutive=0,lastClock=-1,dt=0,lastState=-1;
  Command command;Snapshot snapshot,before;std::ofstream csv,ackLog,stateLog;

  Snapshot Read(const std::string &when){
    Snapshot s;s.iteration=world->Iterations();s.time=world->SimTime().Double();s.q=joint->Position(0);s.v=joint->GetVelocity(0);
    // This is the controller SetForce input ledger, not reaction/total force.
    s.pose=panel->WorldPose();s.axis=joint->GlobalAxis(0);s.force=joint->GetForce(0);s.stage=when;
    auto pids=controller->GetPositionPIDs();auto pid=pids.at(scoped);pid.GetErrors(s.pe,s.ie,s.de);s.cmd=pid.GetCmd();
    const double values[]={s.time,s.q,s.v,s.force,s.cmd,s.pe,s.ie,s.de,s.pose.Pos().X(),s.pose.Pos().Y(),s.pose.Pos().Z(),s.pose.Rot().W(),s.pose.Rot().X(),s.pose.Rot().Y(),s.pose.Rot().Z(),s.axis.X(),s.axis.Y(),s.axis.Z()};
    for(double v:values)if(!std::isfinite(v))throw std::runtime_error("Non-finite actual gate SDK state");
    return s;
  }
  std::string SnapshotJSON() const {
    const auto &s=snapshot;std::ostringstream o;o<<std::setprecision(17)<<"{\"iteration\":"<<s.iteration<<",\"sim_time_s\":"<<s.time
      <<",\"sample_stage\":"<<Q(s.stage)<<",\"q_m\":"<<s.q<<",\"qd_m_s\":"<<s.v<<",\"force_N\":"<<s.force
      <<",\"pid_command_N\":"<<s.cmd<<",\"pid_p_error\":"<<s.pe<<",\"pid_i_error\":"<<s.ie<<",\"pid_d_error\":"<<s.de
      <<",\"panel_world_pose\":"<<Pose(s.pose)<<",\"axis_xyz\":"<<Vec(s.axis)<<'}';return o.str();
  }
  std::string Payload(const Command &c,const std::string &event,const std::string &reason="",const std::string &query="") const {
    std::ostringstream o;o<<std::setprecision(17)<<"{\"schema\":1,\"run_id\":"<<Q(run)<<",\"world\":"<<Q(worldName)<<",\"epoch\":"<<epoch
      <<",\"client_id\":"<<Q(c.client)<<",\"seq\":"<<c.seq<<",\"nonce\":"<<Q(c.nonce)<<",\"attempt_nonce\":"<<Q(c.attempt)<<",\"stage\":"<<Q(event)<<",\"reason\":"<<Q(reason)
      <<",\"target_requested_m\":";
    if(c.targetValid)o<<c.target;else o<<"null";
    o<<",\"target_applied_m\":"<<targetApplied<<",\"snapshot\":"<<SnapshotJSON()
      <<",\"counters\":{\"received\":"<<received<<",\"applied\":"<<applied<<",\"completed\":"<<completed<<",\"rejected\":"<<rejected<<",\"reset\":"<<resets<<'}'
      <<",\"pending\":"<<(pending?"true":"false")<<",\"active\":"<<(active?"true":"false")<<",\"parameters\":{";
    bool first=true;for(const auto &x:parameters){if(!first)o<<',';first=false;o<<Q(x.first)<<':'<<x.second;}
    o<<"},\"model\":"<<modelJSON<<",\"config_sha256\":"<<Q(configSha)<<",\"world_sha256\":"<<Q(worldSha)
      <<",\"source_sha256\":"<<Q(sourceSha)<<",\"client_source_sha256\":"<<Q(clientSha)<<",\"error\":"<<Q(error);
    if(!query.empty())o<<",\"query_nonce\":"<<Q(query);
    o<<'}';return o.str();
  }
  void Publish(const transport::PublisherPtr &pub,const std::string &json){msgs::Any m;m.set_type(msgs::Any::STRING);m.set_string_value(json);pub->Publish(m);}
  void Status(){std::ofstream o(directory+"/status.json.tmp");o<<Payload(command,stage)<<'\n';o.close();std::rename((directory+"/status.json.tmp").c_str(),(directory+"/status.json").c_str());}
  void Ack(const Command &c,const std::string &event,const std::string &reason=""){auto json=Payload(c,event,reason);Publish(ackPub,json);ackLog<<json<<'\n';ackLog.flush();}
  void OnCommand(ConstRequestPtr &msg){
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    Command c;c.seq=msg->id();c.target=msg->dbl_data();c.targetValid=msg->has_dbl_data()&&std::isfinite(c.target);
    std::istringstream in(msg->data());std::string epochText,extra,reason;
    if(!(in>>c.run>>c.client>>c.nonce>>epochText>>c.attempt)||(in>>extra))reason="invalid envelope: RUN CLIENT COMMAND_NONCE EPOCH ATTEMPT_NONCE required";
    else {try{size_t used=0;c.epoch=std::stoll(epochText,&used);if(used!=epochText.size())reason="invalid epoch";}catch(...){reason="invalid epoch";}}
    if(reason.empty()&&!msg->IsInitialized())reason="Request required fields missing";
    if(reason.empty()&&(!Safe(c.client,32)||c.nonce.size()!=32||c.nonce.find_first_not_of("0123456789abcdef")!=std::string::npos||c.attempt.size()!=32||c.attempt.find_first_not_of("0123456789abcdef")!=std::string::npos))reason="invalid client_id or UUID hex command/attempt nonce";
    if(reason.empty()&&(c.run!=run||c.epoch!=static_cast<long long>(epoch)))reason="wrong run or epoch";
    if(reason.empty()&&(msg->request()!="set_target"))reason="unsupported verb";
    if(reason.empty()&&(!c.targetValid||c.target<0||c.target>.7))reason="missing/nonfinite/out-of-range target";
    auto previous=sequences.find(c.client);int previousSeq=previous==sequences.end()?0:previous->second;
    if(reason.empty()&&(c.seq<=0||c.seq<=previousSeq||nonces.count(c.client+" "+c.nonce)))reason="duplicate/stale seq or nonce";
    if(reason.empty()&&(pending||active))reason="busy: previous accepted transaction is not complete";
    if(reason.empty()&&!error.empty())reason="gate error: reload required";
    if(!reason.empty()){++rejected;Ack(c,"rejected",reason);Status();return;}
    sequences[c.client]=c.seq;nonces.insert(c.client+" "+c.nonce);command=c;pending=true;stage="received";++received;
    Ack(command,"received");Status();
  }
  void OnQuery(ConstRequestPtr &msg){
    if(msg->request()!="gazeboteach_transport_snapshot")return;
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    msgs::Any payload;payload.set_type(msgs::Any::STRING);payload.set_string_value(Payload(command,stage,"",msg->data()));
    msgs::Response response;response.set_id(msg->id());response.set_request(msg->request());response.set_response("success");response.set_type(payload.GetTypeName());payload.SerializeToString(response.mutable_serialized_data());responsePub->Publish(response);
  }
  void InstallHold(){controller->Reset();controller->SetPositionPID(scoped,common::PID(80,0,20,0,0,20,-20));controller->SetPositionTarget(scoped,0);targetApplied=0;}
  void Begin(const common::UpdateInfo &info){
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    try{
      double now=info.simTime.Double();dt=lastClock<0?0:now-lastClock;lastClock=now;
      if(pending&&std::abs(dt-.001)<1e-8){pending=false;active=true;needApplied=true;targetApplied=command.target;step=0;consecutive=0;controller->SetPositionTarget(scoped,targetApplied);}
    }catch(const std::exception &e){Fail(e.what());}
  }
  void Before(const common::UpdateInfo &){
    std::lock_guard<std::mutex> lock(mutex);if(closing||!active)return;
    try{if(std::abs(dt-.001)>1e-8)throw std::runtime_error("Actual transaction physics dt is not 0.001 s");before=Read("before_physics");if(needApplied){snapshot=before;stage="applied";++applied;Ack(command,"applied");needApplied=false;}}
    catch(const std::exception &e){Fail(e.what());}
  }
  void End(){
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    try{
      snapshot=Read("after_physics");
      if(active){
        ++step;bool good=std::abs(snapshot.q-targetApplied)<=.005&&std::abs(snapshot.v)<=.01;
        consecutive=good?consecutive+dt:0;
        if(consecutive>=.2-1e-9){stage="completed";active=false;++completed;}
        if(std::abs(snapshot.force)>20+1e-7||std::abs(snapshot.pose.Pos().X()-snapshot.q)>1e-5||std::abs(snapshot.pose.Pos().Y())>1e-5||std::abs(snapshot.pose.Pos().Z()-.6)>1e-5)throw std::runtime_error("Actual gate force/pose geometry exceeds fixed rig bounds");
        csv<<std::setprecision(17)<<run<<','<<epoch<<','<<command.client<<','<<command.seq<<','<<command.nonce<<','<<command.attempt<<','<<step<<','<<step*.001<<','<<snapshot.iteration<<','<<snapshot.time<<','<<dt<<','<<targetApplied<<','
          <<before.q<<','<<before.v<<','<<snapshot.q<<','<<snapshot.v<<','<<before.force<<','<<before.cmd<<','<<before.pe<<','<<before.ie<<','<<before.de<<','
          <<snapshot.pose.Pos().X()<<','<<snapshot.pose.Pos().Y()<<','<<snapshot.pose.Pos().Z()<<','<<snapshot.pose.Rot().W()<<','<<snapshot.pose.Rot().X()<<','<<snapshot.pose.Rot().Y()<<','<<snapshot.pose.Rot().Z()<<','
          <<snapshot.axis.X()<<','<<snapshot.axis.Y()<<','<<snapshot.axis.Z()<<','<<consecutive<<','<<stage<<','<<configSha<<'\n';
        if(step%100==0||world->IsPaused()||!active)csv.flush();
        if(!active){Ack(command,"completed");Status();}
        else if(step>=3000)throw std::runtime_error("Gate did not physically complete within 3 simulated seconds");
      }
      if(snapshot.time-lastState>=.05-1e-9||lastState<0){auto json=Payload(command,stage);Publish(statePub,json);stateLog<<json<<'\n';stateLog.flush();lastState=snapshot.time;Status();}
      else if(world->IsPaused())Status();
    }catch(const std::exception &e){Fail(e.what());}
  }
  void Fail(const std::string &message){error=message;pending=false;active=false;needApplied=false;stage="failed";controller->Reset();Ack(command,"failed",message);csv.flush();Status();gzerr<<message<<'\n';}
  void Reset(){
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    if(pending||active){pending=false;active=false;Ack(command,"cancelled","public model reset cancelled this transaction");}
    ++epoch;++resets;sequences.clear();nonces.clear();command=Command();pending=active=needApplied=false;stage="idle";step=0;consecutive=0;lastClock=-1;lastState=-1;error.clear();
    InstallHold();snapshot=Read("initial");Ack(command,"idle");Publish(statePub,Payload(command,stage));Status();
  }
};
}
class TransportGate : public ModelPlugin {
  std::shared_ptr<State> s;event::ConnectionPtr begin,before,end;
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    s=std::make_shared<State>();s->model=model;s->world=model->GetWorld();s->worldName=s->world->Name();s->run=Env("GAZEBOTEACH_SESSION_TOKEN");s->directory=Env("GAZEBOTEACH_SESSION_DIR")+"/control";mkdir(s->directory.c_str(),0755);
    std::ifstream in(Env("GAZEBOTEACH_WORKSPACE")+"/build/transport_gate.conf");std::getline(in,s->configSha);std::getline(in,s->worldSha);std::getline(in,s->sourceSha);std::getline(in,s->clientSha);std::string key;double value;while(in>>key>>value)s->parameters[key]=value;
    if(s->parameters.size()!=2||s->sourceSha.size()!=64)throw std::runtime_error("Missing frozen gate build configuration");
    s->panel=model->GetLink("panel");s->joint=model->GetJoint("slide_joint");s->controller=model->GetJointController();
    if(!s->panel||!s->joint||s->joint->GetParent()||s->joint->GetChild()!=s->panel)throw std::runtime_error("Actual gate topology differs");
    s->scoped=s->joint->GetScopedName();s->InstallHold();
    auto shape=boost::dynamic_pointer_cast<physics::BoxShape>(s->panel->GetCollision("panel_collision")->GetShape());if(!shape)throw std::runtime_error("Actual gate collision is not a box");
    auto inertia=s->panel->GetInertial();auto pid=s->controller->GetPositionPIDs().at(s->scoped);std::ostringstream m;m<<std::setprecision(17)<<"{\"mass_kg\":"<<inertia->Mass()<<",\"Ixx_kg_m2\":"<<inertia->IXX()<<",\"Iyy_kg_m2\":"<<inertia->IYY()<<",\"Izz_kg_m2\":"<<inertia->IZZ()
      <<",\"damping_Ns_m\":"<<s->joint->GetDamping(0)<<",\"lower_limit_m\":"<<s->joint->LowerLimit(0)<<",\"upper_limit_m\":"<<s->joint->UpperLimit(0)<<",\"effort_limit_N\":"<<s->joint->GetEffortLimit(0)
      <<",\"physics_step_s\":"<<s->world->Physics()->GetMaxStepSize()<<",\"pid_p\":"<<pid.GetPGain()<<",\"pid_i\":"<<pid.GetIGain()<<",\"pid_d\":"<<pid.GetDGain()<<",\"pid_command_limit_N\":"<<pid.GetCmdMax()<<",\"pid_command_min_N\":"<<pid.GetCmdMin()<<",\"box_size_xyz\":"<<Vec(shape->Size())
      <<",\"initial_axis_xyz\":"<<Vec(s->joint->GlobalAxis(0))<<",\"initial_world_pose\":"<<Pose(s->panel->WorldPose())<<",\"joint_parent\":\"world\",\"joint_child\":\"panel\"}";s->modelJSON=m.str();
    s->snapshot=s->Read("initial");s->csv.open(s->directory+"/gate.csv");s->csv<<"run_id,epoch,client_id,seq,nonce,attempt_nonce,transaction_step,transaction_elapsed_s,iteration,sim_time_s,dt_s,target_m,q_before_m,qd_before_m_s,q_after_m,qd_after_m_s,force_N,pid_command_N,pid_p_error,pid_i_error,pid_d_error,panel_x_m,panel_y_m,panel_z_m,panel_qw,panel_qx,panel_qy,panel_qz,axis_x,axis_y,axis_z,completion_consecutive_s,stage,config_sha256\n";s->csv.flush();s->ackLog.open(s->directory+"/ack-events.jsonl");s->stateLog.open(s->directory+"/state-events.jsonl");
    s->node.reset(new transport::Node());s->node->Init(s->worldName);s->ackPub=s->node->Advertise<msgs::Any>("~/transport_lab/ack");s->statePub=s->node->Advertise<msgs::Any>("~/transport_lab/state");s->responsePub=s->node->Advertise<msgs::Response>("~/response");
    s->commandSub=s->node->Subscribe("~/transport_lab/command",&State::OnCommand,s.get(),false);s->requestSub=s->node->Subscribe("~/request",&State::OnQuery,s.get(),false);
    s->Ack(s->command,"idle");s->Publish(s->statePub,s->Payload(s->command,s->stage));s->Status();
    auto state=s;begin=event::Events::ConnectWorldUpdateBegin([state](const common::UpdateInfo &i){state->Begin(i);});before=event::Events::ConnectBeforePhysicsUpdate([state](const common::UpdateInfo &i){state->Before(i);});end=event::Events::ConnectWorldUpdateEnd([state](){state->End();});
  }
  void Reset() override {if(s)s->Reset();}
  ~TransportGate() override {if(s){std::lock_guard<std::mutex> lock(s->mutex);s->closing=true;s->csv.flush();}begin.reset();before.reset();end.reset();}
};
GZ_REGISTER_MODEL_PLUGIN(TransportGate)
}
