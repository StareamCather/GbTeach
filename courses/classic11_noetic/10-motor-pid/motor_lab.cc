// Native Classic JointController remains the only feedback controller.
// This helper initializes bounded trials, injects a load and records real steps.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/ode/objects.h>
#include <gazebo/common/Events.hh>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace gazebo {
namespace {
std::string Env(const char *key) {
  const char *value=std::getenv(key);
  if(!value) throw std::runtime_error(std::string("Missing ")+key);
  return value;
}
std::string Quote(const std::string &value) {
  std::ostringstream out;out<<'"';
  for(char c:value) {if(c=='"'||c=='\\')out<<'\\'; if(c=='\n')out<<"\\n";else out<<c;}
  out<<'"';return out.str();
}
const char *phases[]={"torque","speed-p","speed-pi","position-p","position-pd","saturation"};
std::string PoseJSON(const ignition::math::Pose3d &pose) {
  std::ostringstream out;out<<std::setprecision(17)<<'['<<pose.Pos().X()<<','<<pose.Pos().Y()<<','<<pose.Pos().Z()<<','
    <<pose.Rot().W()<<','<<pose.Rot().X()<<','<<pose.Rot().Y()<<','<<pose.Rot().Z()<<']';return out.str();
}
bool SamePose(const ignition::math::Pose3d &a,const ignition::math::Pose3d &b) {
  auto delta=a.Rot().Inverse()*b.Rot();
  const double values[]={a.Pos().X(),a.Pos().Y(),a.Pos().Z(),b.Pos().X(),b.Pos().Y(),b.Pos().Z(),delta.W(),delta.X(),delta.Y(),delta.Z()};
  for(double value:values)if(!std::isfinite(value))return false;
  return (a.Pos()-b.Pos()).Length()<1e-9&&std::abs(std::abs(delta.W())-1)<1e-9&&std::abs(delta.X())<1e-9&&std::abs(delta.Y())<1e-9&&std::abs(delta.Z())<1e-9;
}
int PhaseIndex(const std::string &name) {for(int i=0;i<6;++i) if(name==phases[i]) return i; return -1;}
double Duration(int phase) {const double values[]={1,6,6,3,3,6};return values[phase];}
struct State {
  std::mutex mutex;
  std::atomic<bool> closing{false};
  physics::ModelPtr model;physics::WorldPtr world;physics::JointPtr joint;
  physics::LinkPtr rotor;physics::JointControllerPtr controller;physics::ODELinkPtr odeRotor;
  ignition::math::Pose3d capturedPose,preInitPose,preCounterPose,postPose;
  double preInitQ=0,preInitW=0;int counterCalls=0,nativeBodyCount=0;
  bool nativeHinge=false,nativeMatches=false,nativeWorld=false,nativeSame=false;
  unsigned int initialJointId=0,currentJointId=0;
  ignition::math::Vector3d nativeAnchorEvidence,nativeAxisEvidence,sdkAnchorEvidence,sdkAxisEvidence;
  std::string scoped,directory,configSha,worldSha,sourceSha,error,episode="waiting",lastRequest,phase="waiting";
  std::map<std::string,double> p;
  std::ofstream csv,resets;
  int phaseIndex=-1,autoIndex=0;unsigned long long step=0,demo=0;
  bool active=false,autoMode=true,complete=false,commandsEmpty=true;
  double worldStamp=0,dt=.001,preQ=0,preW=0,lastRaw=0,unwrapped=0;
  double command=0,load=0,applied=0,pe=0,ie=0,de=0,pg=0,ig=0,dg=0,raw=0;
  double resetQ=0,resetW=0,postResetQ=0,postResetW=0; bool correctedCounter=false;
  ignition::math::Vector3d resetAxis{1,0,0},postResetAxis{1,0,0};
  double resetPE=0,resetIE=0,resetDE=0,resetCmd=0,lastStatus=-1,pollAt=-1,lastClock=-1;

  void Status() {
    auto inertia=rotor->GetInertial();
    auto shape=boost::dynamic_pointer_cast<physics::CylinderShape>(rotor->GetCollision("rotor_collision")->GetShape());
    auto axis=joint->GlobalAxis(0);
    std::ofstream out(directory+"/status.json.tmp");
    out<<std::setprecision(17)<<"{\"schema\":1,\"episode\":"<<Quote(episode)<<",\"phase\":"<<Quote(phase)
       <<",\"phase_step\":"<<step<<",\"phase_elapsed_s\":"<<step*.001
       <<",\"phase_duration_s\":"<<(phaseIndex<0?0:Duration(phaseIndex))
       <<",\"phase_complete\":"<<(complete?"true":"false")<<",\"active\":"<<(active?"true":"false")
       <<",\"finished\":"<<(!active&&!autoMode?"true":"false")<<",\"error\":"<<Quote(error)
       <<",\"config_sha256\":"<<Quote(configSha)<<",\"world_sha256\":"<<Quote(worldSha)
       <<",\"source_sha256\":"<<Quote(sourceSha)<<",\"sim_time_s\":"<<world->SimTime().Double()
       <<",\"parameters\":{";
    bool first=true;for(auto &item:p){if(!first)out<<',';first=false;out<<Quote(item.first)<<':'<<item.second;}
    out<<"},\"reset_evidence\":{\"p_error\":"<<resetPE<<",\"i_error\":"<<resetIE
       <<",\"d_error\":"<<resetDE<<",\"command_Nm\":"<<resetCmd<<",\"commands_empty\":"<<(commandsEmpty?"true":"false")
       <<",\"native_counter_handle\":{\"body_joint_count\":"<<nativeBodyCount<<",\"joint_type\":"<<Quote(nativeHinge?"hinge":"unmatched")
       <<",\"rotor_body_matches\":"<<(nativeMatches?"true":"false")<<",\"other_body_is_world\":"<<(nativeWorld?"true":"false")
       <<",\"anchor_world_m\":["<<nativeAnchorEvidence.X()<<','<<nativeAnchorEvidence.Y()<<','<<nativeAnchorEvidence.Z()<<']'
       <<",\"axis_world\":["<<nativeAxisEvidence.X()<<','<<nativeAxisEvidence.Y()<<','<<nativeAxisEvidence.Z()<<']'
       <<",\"sdk_anchor_world_m\":["<<sdkAnchorEvidence.X()<<','<<sdkAnchorEvidence.Y()<<','<<sdkAnchorEvidence.Z()<<']'
       <<",\"sdk_axis_world\":["<<sdkAxisEvidence.X()<<','<<sdkAxisEvidence.Y()<<','<<sdkAxisEvidence.Z()<<']'
       <<",\"initial_sdk_joint_id\":"<<initialJointId<<",\"current_sdk_joint_id\":"<<currentJointId
       <<",\"sdk_joint_identity_unchanged\":"<<(nativeSame?"true":"false")<<'}'
       <<",\"initializer_method\":\"captured_absolute_link_pose_then_native_counter_sync\""
       <<",\"pre_init_q_rad\":"<<preInitQ<<",\"pre_init_omega_rad_s\":"<<preInitW
       <<",\"pre_init_world_pose\":"<<PoseJSON(preInitPose)<<",\"captured_initial_world_pose\":"<<PoseJSON(capturedPose)
       <<",\"pre_counter_world_pose\":"<<PoseJSON(preCounterPose)<<",\"post_world_pose\":"<<PoseJSON(postPose)
       <<",\"counter_sync_api\":\"OpenDE C API via public ODELink.GetODEId/body joint enumeration\""
       <<",\"counter_sync_called\":"<<(counterCalls==1?"true":"false")<<",\"counter_sync_calls\":"<<counterCalls
       <<",\"counter_offset_removed\":"<<(correctedCounter?"true":"false")
       <<",\"pre_counter_q_rad\":"<<resetQ<<",\"pre_counter_omega_rad_s\":"<<resetW
       <<",\"pre_counter_world_x_axis\":["<<resetAxis.X()<<','<<resetAxis.Y()<<','<<resetAxis.Z()<<']'
       <<",\"counter_correction_applied\":"<<(correctedCounter?"true":"false")
       <<",\"post_q_rad\":"<<postResetQ<<",\"post_omega_rad_s\":"<<postResetW
       <<",\"post_world_x_axis\":["<<postResetAxis.X()<<','<<postResetAxis.Y()<<','<<postResetAxis.Z()<<']'
       <<"},\"model\":{\"mass_kg\":"<<inertia->Mass()<<",\"radius_m\":"<<shape->GetRadius()<<",\"length_m\":"<<shape->GetLength()
       <<",\"Ixx_kg_m2\":"<<inertia->IXX()<<",\"Iyy_kg_m2\":"<<inertia->IYY()<<",\"Izz_kg_m2\":"<<inertia->IZZ()
       <<",\"damping_Nms_rad\":"<<joint->GetDamping(0)<<",\"axis_xyz\":["<<axis.X()<<','<<axis.Y()<<','<<axis.Z()<<']'
       <<",\"effort_limit_Nm\":"<<joint->GetEffortLimit(0)<<",\"physics_step_s\":"<<world->Physics()->GetMaxStepSize()
       <<",\"gravity_z\":"<<world->Gravity().Z()<<"}}\n";
    out.close();std::rename((directory+"/status.json.tmp").c_str(),(directory+"/status.json").c_str());
    lastStatus=world->SimTime().Double();
  }

  void BeginPhase(const std::string &id,int which) {
    controller->Reset();
    commandsEmpty=controller->GetForces().empty()&&controller->GetPositions().empty()&&controller->GetVelocities().empty();
    // A phase starts with an explicitly verified full zero state. This is the
    // only pose/counter initialization; native force feedback owns every later step.
    preInitQ=joint->Position(0);preInitW=joint->GetVelocity(0);preInitPose=rotor->WorldPose();
    counterCalls=0;correctedCounter=false;
    rotor->SetWorldPose(capturedPose);joint->SetVelocity(0,0);
    preCounterPose=rotor->WorldPose();
    resetQ=joint->Position(0);resetW=joint->GetVelocity(0);
    resetAxis=preCounterPose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
    if(!SamePose(preCounterPose,capturedPose)||std::abs(resetW)>1e-7)
      throw std::runtime_error("Full phase pose/velocity initialization failed before native counter synchronization");
    // Gazebo's ODE hinge can retain complete turns after the body returns to zero.
    // Synchronize that encoder reference exactly once, including already-zero cases.
    dBodyID body=odeRotor->GetODEId();
    nativeBodyCount=body?dBodyGetNumJoints(body):0;
    auto currentJoint=model->GetJoint("rotor_joint");currentJointId=currentJoint?currentJoint->GetId():0;
    nativeSame=currentJoint.get()==joint.get()&&currentJointId==initialJointId;
    if(!body||nativeBodyCount!=1)throw std::runtime_error("Expected exactly one native rotor hinge before counter sync");
    dJointID native=dBodyGetJoint(body,0);
    nativeHinge=native&&dJointGetType(native)==dJointTypeHinge;
    if(!nativeHinge)
      throw std::runtime_error("Native rotor joint is not a hinge");
    dBodyID first=dJointGetBody(native,0),second=dJointGetBody(native,1);
    nativeMatches=first==body||second==body;nativeWorld=(first==body&&!second)||(second==body&&!first);
    if(!nativeWorld)
      throw std::runtime_error("Native rotor hinge does not connect its body directly to world");
    dVector3 anchor,axis;dJointGetHingeAnchor(native,anchor);dJointGetHingeAxis(native,axis);
    auto sdkAxis=joint->GlobalAxis(0);auto sdkAnchor=joint->Anchor(0);
    nativeAnchorEvidence.Set(anchor[0],anchor[1],anchor[2]);nativeAxisEvidence.Set(axis[0],axis[1],axis[2]);
    sdkAxisEvidence=sdkAxis;sdkAnchorEvidence=sdkAnchor;
    for(int i=0;i<3;++i) {
      double wantAxis=i==2?1.:0.,wantAnchor=i==2?.6:0.;
      if(!std::isfinite(anchor[i])||!std::isfinite(axis[i])||std::abs(anchor[i]-wantAnchor)>1e-9||
         std::abs(axis[i]-wantAxis)>1e-9||std::abs(axis[i]-sdkAxis[i])>1e-9||std::abs(anchor[i]-sdkAnchor[i])>1e-9)
        throw std::runtime_error("Public OpenDE hinge handle disagrees with the SDK anchor/axis");
    }
    dJointSetHingeCumulativeAngle(native,0);++counterCalls;correctedCounter=std::abs(resetQ)>1e-7;
    postPose=rotor->WorldPose();postResetQ=joint->Position(0);postResetW=joint->GetVelocity(0);
    postResetAxis=postPose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
    if(!SamePose(postPose,capturedPose))throw std::runtime_error("Counter synchronization unexpectedly changed the physical pose");
    phaseIndex=which;phase=phases[which];episode=id;step=0;complete=false;active=true;
    preQ=joint->Position(0);preW=joint->GetVelocity(0);
    lastRaw=preQ;unwrapped=preQ;
    resets<<std::setprecision(17)<<"full_state_initializer "<<episode<<" pre_init_q="<<preInitQ<<" pre_counter_q="<<resetQ<<" post_q="<<postResetQ<<" sync_calls="<<counterCalls<<" offset_removed="<<correctedCounter<<'\n';resets.flush();
    if(std::abs(preQ)>1e-7||std::abs(preW)>1e-7) throw std::runtime_error("Native phase initialization did not yield zero q/velocity: q="+std::to_string(preQ)+" w="+std::to_string(preW));
    double cap=which==5?p.at("saturation_limit_Nm"):p.at("command_limit_Nm");
    pg=which==1?p.at("speed_p_kp"):(which==2||which==5?p.at("speed_pi_kp"):(which==3||which==4?p.at("position_kp"):0));
    ig=which==2||which==5?p.at("speed_pi_ki"):0;
    dg=which==4?p.at("position_kd"):0;
    common::PID pid(pg,ig,dg,p.at("integral_term_limit_Nm"),-p.at("integral_term_limit_Nm"),cap,-cap);
    if(which==3||which==4) controller->SetPositionPID(scoped,pid);
    else controller->SetVelocityPID(scoped,pid);
    auto map=(which==3||which==4)?controller->GetPositionPIDs():controller->GetVelocityPIDs();
    auto actual=map.at(scoped);actual.GetErrors(resetPE,resetIE,resetDE);resetCmd=actual.GetCmd();
    if(!commandsEmpty||resetPE!=0||resetIE!=0||resetDE!=0||resetCmd!=0) throw std::runtime_error("Full native PID reset failed");
    if(which==0) controller->SetForce(scoped,p.at("torque_command_Nm"));
    else if(which==3||which==4) controller->SetPositionTarget(scoped,p.at("position_target_rad"));
    else controller->SetVelocityTarget(scoped,p.at("velocity_target_rad_s"));
    resets<<Quote(episode)<<' '<<Quote(phase)<<" reset "<<resetPE<<' '<<resetIE<<' '<<resetDE<<' '<<resetCmd<<' '<<commandsEmpty<<'\n';resets.flush();
    Status();
  }

  void ReadRequest() {
    std::ifstream in(directory+"/phase.request");std::string id,name;
    if(!(in>>id>>name)||id==lastRequest)return;
    lastRequest=id;int which=PhaseIndex(name);
    if(id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos||which<0)
      throw std::runtime_error("Invalid bounded motor phase request");
    autoMode=false;error.clear();BeginPhase(id,which);
  }

  void Begin(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    try {
      worldStamp=info.simTime.Double();
      dt=(info.simTime-controller->GetLastUpdateTime()).Double();
      if(worldStamp<lastClock) {controller->Reset();active=false;autoMode=true;autoIndex=0;pollAt=-1;}
      lastClock=worldStamp;
      // Model.Update will repair a backward native timestamp after a global reset.
      // Do not initialize or record a trial until its real next interval is 1 ms.
      if(std::abs(dt-.001)>1e-9) {
        if(active)throw std::runtime_error("Actual native JointController dt is not 0.001 s");
        return;
      }
      ReadRequest();  // Atomic, idempotent request: next paused step acknowledges it.
      if(!active&&autoMode&&error.empty()) {
        if(autoIndex<6) {int which=autoIndex++;BeginPhase("demo-"+std::to_string(demo)+"-"+std::to_string(which),which);}
        else {autoMode=false;Status();}
      }
    } catch(const std::exception &e) {error=e.what();controller->Reset();active=false;autoMode=false;Status();gzerr<<error<<"\n";}
  }

  void Before(const common::UpdateInfo &) {
    std::lock_guard<std::mutex> lock(mutex);if(closing||!active)return;
    preQ=joint->Position(0);preW=joint->GetVelocity(0);
    if(phaseIndex==0) {command=p.at("torque_command_Nm");pe=ie=de=pg=ig=dg=0;raw=command;}
    else {
      auto map=(phaseIndex==3||phaseIndex==4)?controller->GetPositionPIDs():controller->GetVelocityPIDs();
      auto pid=map.at(scoped);pid.GetErrors(pe,ie,de);command=pid.GetCmd();
      pg=pid.GetPGain();ig=pid.GetIGain();dg=pid.GetDGain();raw=-(pg*pe+ig*ie+dg*de);
    }
    // step is the number of already completed intervals: load begins at pre t=2.
    load=(phaseIndex==1||phaseIndex==2||phaseIndex==5)&&step*.001>=p.at("load_start_s")?p.at("load_torque_Nm"):0;
    joint->SetForce(0,-load);
    applied=joint->GetForce(0); // External input ledger; not constraint reaction or damping.
  }

  void End() {
    std::lock_guard<std::mutex> lock(mutex);if(closing||!active)return;
    double q=joint->Position(0),w=joint->GetVelocity(0);
    unwrapped+=std::remainder(q-lastRaw,2*M_PI);lastRaw=q;
    ++step;
    auto axis=rotor->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitX);
    const char *mode=phaseIndex==0?"force":(phaseIndex==3||phaseIndex==4?"position":"velocity");
    csv<<std::setprecision(17)<<episode<<','<<phase<<','<<step<<','<<step*.001<<','<<worldStamp<<','<<dt<<','
       <<preQ<<','<<preW<<','<<q<<','<<w<<','<<unwrapped<<','<<command<<','<<pe<<','<<ie<<','<<de<<','
       <<pg<<','<<ig<<','<<dg<<','<<ig*ie<<','<<raw<<','<<load<<','<<applied<<','<<axis.X()<<','<<axis.Y()<<','<<mode<<','<<configSha<<'\n';
    if(step%100==0||world->IsPaused())csv.flush();
    if(step>=static_cast<unsigned long long>(std::llround(Duration(phaseIndex)/.001))) {
      csv.flush();complete=true;active=false;controller->Reset();Status();
    } else if(world->IsPaused()||worldStamp-lastStatus>=.1)Status();
  }

  void Reset() {
    std::lock_guard<std::mutex> lock(mutex);controller->Reset();
    active=false;autoMode=true;autoIndex=0;++demo;step=0;complete=false;
    phase="reset-pending";episode="reset-"+std::to_string(demo);lastClock=-1;pollAt=-1;error.clear();Status();
  }
};
}
class MotorLaboratory : public ModelPlugin {
  std::shared_ptr<State> state;
  event::ConnectionPtr begin,before,end;
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    auto s=std::make_shared<State>();state=s;s->model=model;s->world=model->GetWorld();
    s->joint=model->GetJoint("rotor_joint");s->rotor=model->GetLink("rotor");s->controller=model->GetJointController();
    if(!s->joint||!s->rotor||!s->controller)throw std::runtime_error("Missing actual rotor/native JointController");
    s->odeRotor=boost::dynamic_pointer_cast<physics::ODELink>(s->rotor);
    if(!s->odeRotor)throw std::runtime_error("This measured laboratory requires an actual ODE rotor link");
    s->capturedPose=s->rotor->WorldPose();
    if(!SamePose(s->capturedPose,ignition::math::Pose3d(0,0,.6,0,0,0)))
      throw std::runtime_error("Captured initial rotor pose differs from the fixed SDF absolute origin");
    s->initialJointId=s->joint->GetId();
    s->scoped=s->joint->GetScopedName();s->directory=Env("GAZEBOTEACH_SESSION_DIR")+"/control";
    if(::mkdir(s->directory.c_str(),0700)!=0&&errno!=EEXIST)throw std::runtime_error("Cannot create session telemetry");
    std::ifstream config(Env("GAZEBOTEACH_WORKSPACE")+"/build/motor_lab.conf");
    if(!(config>>s->configSha>>s->worldSha>>s->sourceSha))throw std::runtime_error("Missing prepared motor configuration");
    std::string key;double value;while(config>>key>>value)s->p[key]=value;
    if(s->p.size()!=14)throw std::runtime_error("Incomplete motor configuration");
    s->csv.open(s->directory+"/control.csv");s->resets.open(s->directory+"/phase-resets.log");
    if(!s->csv||!s->resets)throw std::runtime_error("Cannot create motor raw telemetry");
    s->csv<<"episode,phase,phase_step,phase_elapsed_s,sim_time_s,dt_s,q_before_rad,omega_before_rad_s,q_after_rad,omega_after_rad_s,q_unwrapped_after_rad,motor_command_Nm,pid_p_error,pid_i_error,pid_d_error,pid_p_gain,pid_i_gain,pid_d_gain,pid_i_term_Nm,pid_raw_command_Nm,load_torque_Nm,joint_applied_Nm,rotor_x_axis_x,rotor_x_axis_y,controller_mode,config_sha256\n";
    s->Status();
    begin=event::Events::ConnectWorldUpdateBegin([s](const common::UpdateInfo &i){s->Begin(i);});
    before=event::Events::ConnectBeforePhysicsUpdate([s](const common::UpdateInfo &i){s->Before(i);});
    end=event::Events::ConnectWorldUpdateEnd([s](){s->End();});
  }
  void Reset() override {if(state)state->Reset();}
  ~MotorLaboratory() override {
    if(state)state->closing=true;begin.reset();before.reset();end.reset();
    if(state){std::lock_guard<std::mutex> lock(state->mutex);state->controller->Reset();}
    state.reset();
  }
};
GZ_REGISTER_MODEL_PLUGIN(MotorLaboratory)
}
