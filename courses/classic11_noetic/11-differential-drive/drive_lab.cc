// The installed DiffDrivePlugin alone drives the wheels. This helper only
// publishes its real Classic Pose command and observes actual physics states.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
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
std::string Env(const char *key) {const char *v=std::getenv(key);if(!v)throw std::runtime_error(std::string("Missing ")+key);return v;}
std::string Quote(const std::string &v) {std::ostringstream o;o<<'"';for(char c:v){if(c=='"'||c=='\\')o<<'\\';if(c=='\n')o<<"\\n";else o<<c;}o<<'"';return o.str();}
std::string Vec(const ignition::math::Vector3d &v) {std::ostringstream o;o<<std::setprecision(17)<<'['<<v.X()<<','<<v.Y()<<','<<v.Z()<<']';return o.str();}
double Delta(double x) {return std::remainder(x,2*M_PI);}
const char *Names[]={"straight","spin","arc"};
struct Sample {double ql=0,qr=0,wl=0,wr=0,x=0,y=0,yaw=0,z=0,vx=0,vy=0,vz=0,wz=0,roll=0,pitch=0;};
struct Stop {double duration=0,consecutive=0,wl=0,wr=0,linear=0,wz=0;bool confirmed=false;};
std::string StopJSON(const Stop &s) {std::ostringstream o;o<<std::setprecision(17)<<"{\"duration_s\":"<<s.duration<<",\"consecutive_s\":"<<s.consecutive<<",\"left_w_rad_s\":"<<s.wl<<",\"right_w_rad_s\":"<<s.wr<<",\"body_linear_speed_m_s\":"<<s.linear<<",\"body_yaw_rate_rad_s\":"<<s.wz<<",\"confirmed\":"<<(s.confirmed?"true":"false")<<'}';return o.str();}
struct State {
  std::mutex mutex;bool closing=false;
  physics::ModelPtr model;physics::WorldPtr world;physics::LinkPtr chassis,leftWheel,rightWheel,caster;
  physics::JointPtr left,right;transport::NodePtr node;transport::PublisherPtr publisher;
  std::map<std::string,double> p;std::map<std::string,double> geometry;
  ignition::math::Vector3d leftAxis,rightAxis,leftAnchor,rightAnchor;
  std::string directory,configSha,worldSha,sourceSha,stockSha,mode,error;
  std::string episode="waiting",phase="waiting",stage="waiting",lastRequest;
  std::ofstream csv;bool active=false,complete=false,autoMode=true;int autoIndex=0,which=-1,resetCount=0;
  bool resetZeroConfirmed=false;
  unsigned long long phaseStep=0,stageStep=0,measureStep=0,serial=0;
  double lastClock=-1,dt=0,stamp=0,lastStatus=-1,commandV=0,commandW=0,targetV=0,targetW=0,legacyX=0,legacyYaw=0;
  double readyConsecutive=0,encoderX=0,encoderY=0,encoderYaw=0,unwrappedYaw=0,previousYaw=0;
  Sample before,baseline;double baselineStamp=0;Stop initialStop,finalStop;

  Sample Read() {
    Sample s;s.ql=left->Position(0);s.qr=right->Position(0);s.wl=left->GetVelocity(0);s.wr=right->GetVelocity(0);
    auto a=(left->Anchor(0)+right->Anchor(0))*.5;auto pose=chassis->WorldPose();auto v=chassis->WorldLinearVel();auto w=chassis->WorldAngularVel();
    s.x=a.X();s.y=a.Y();s.z=a.Z();s.yaw=pose.Rot().Yaw();s.roll=pose.Rot().Roll();s.pitch=pose.Rot().Pitch();
    s.vx=v.X();s.vy=v.Y();s.vz=v.Z();s.wz=w.Z();
    const double fields[]={s.ql,s.qr,s.wl,s.wr,s.x,s.y,s.z,s.yaw,s.roll,s.pitch,s.vx,s.vy,s.vz,s.wz};
    for(double value:fields)if(!std::isfinite(value))throw std::runtime_error("Non-finite actual wheel/chassis state");
    return s;
  }
  void Send(double v,double omega) {
    msgs::Pose msg;msgs::Set(msg.mutable_position(),ignition::math::Vector3d(v,0,0));
    msgs::Set(msg.mutable_orientation(),ignition::math::Quaterniond(0,0,-omega));
    legacyX=msg.position().x();legacyYaw=msgs::ConvertIgn(msg.orientation()).Euler().Z();
    publisher->Publish(msg);commandV=v;commandW=omega;
  }
  double Mu(const physics::LinkPtr &link,const std::string &name,bool second=false) {
    auto c=link->GetCollision(name);if(!c||!c->GetSurface()||!c->GetSurface()->FrictionPyramid())throw std::runtime_error("Missing actual wheel surface");
    auto f=c->GetSurface()->FrictionPyramid();return second?f->MuSecondary():f->MuPrimary();
  }
  void Status() {
    std::ofstream out(directory+"/status.json.tmp");out<<std::setprecision(17)
      <<"{\"schema\":1,\"episode\":"<<Quote(episode)<<",\"phase\":"<<Quote(phase)<<",\"stage\":"<<Quote(stage)
      <<",\"phase_step\":"<<phaseStep<<",\"phase_elapsed_s\":"<<phaseStep*.001
      <<",\"measurement_step\":"<<measureStep<<",\"measurement_duration_s\":2,\"phase_complete\":"<<(complete?"true":"false")
      <<",\"active\":"<<(active?"true":"false")<<",\"finished\":"<<(!active&&!autoMode?"true":"false")
      <<",\"error\":"<<Quote(error)<<",\"sim_time_s\":"<<world->SimTime().Double()
      <<",\"config_sha256\":"<<Quote(configSha)<<",\"world_sha256\":"<<Quote(worldSha)
      <<",\"source_sha256\":"<<Quote(sourceSha)<<",\"stock_plugin_sha256\":"<<Quote(stockSha)
      <<",\"command_topic\":"<<Quote("/gazebo/"+world->Name()+"/"+model->GetName()+"/vel_cmd")
      <<",\"command_message_type\":\"gazebo.msgs.Pose\",\"parameters\":{";
    bool first=true;for(const auto &x:p){if(!first)out<<',';first=false;out<<Quote(x.first)<<':'<<x.second;}
    out<<"},\"model\":{";first=true;for(const auto &x:geometry){if(!first)out<<',';first=false;out<<Quote(x.first)<<':'<<x.second;}
    auto floor=world->ModelByName("floor")->GetLink("ground");
    out<<",\"initial_left_axis_xyz\":"<<Vec(leftAxis)<<",\"initial_right_axis_xyz\":"<<Vec(rightAxis)
      <<",\"initial_left_anchor_xyz\":"<<Vec(leftAnchor)<<",\"initial_right_anchor_xyz\":"<<Vec(rightAnchor)
      <<",\"left_mu\":"<<Mu(leftWheel,"wheel_collision")<<",\"left_mu2\":"<<Mu(leftWheel,"wheel_collision",true)
      <<",\"right_mu\":"<<Mu(rightWheel,"wheel_collision")<<",\"right_mu2\":"<<Mu(rightWheel,"wheel_collision",true)
      <<",\"floor_mu\":"<<Mu(floor,"floor_collision")<<",\"floor_mu2\":"<<Mu(floor,"floor_collision",true)
      <<",\"caster_mu\":"<<Mu(caster,"caster_collision")<<",\"caster_mu2\":"<<Mu(caster,"caster_collision",true)
      <<",\"surface_mode\":"<<Quote(mode)<<",\"stock_subscriber_connected\":"<<(publisher->HasConnections()?"true":"false")
      <<"},\"reset_evidence\":{\"count\":"<<resetCount<<",\"episode\":"<<Quote(resetCount?"reset-"+std::to_string(serial):"")
      <<",\"zero_published\":"<<(resetCount?"true":"false")<<",\"zero_confirmed\":"<<(resetZeroConfirmed?"true":"false")
      <<"},\"baseline\":{\"gt_x_m\":"<<baseline.x<<",\"gt_y_m\":"<<baseline.y<<",\"gt_yaw_rad\":"<<baseline.yaw
      <<",\"left_q_rad\":"<<baseline.ql<<",\"right_q_rad\":"<<baseline.qr<<",\"sim_time_s\":"<<baselineStamp
      <<"},\"stop_evidence\":{\"initial\":"<<StopJSON(initialStop)<<",\"final\":"<<StopJSON(finalStop)<<"}}\n";
    out.close();std::rename((directory+"/status.json.tmp").c_str(),(directory+"/status.json").c_str());lastStatus=world->SimTime().Double();
  }
  void Start(const std::string &id,int index) {
    if(mode=="slip"&&index!=0)throw std::runtime_error("Slip surface supports the straight comparison only");
    episode=id;which=index;phase=Names[index];stage="stopping_initial";phaseStep=stageStep=measureStep=0;
    active=true;complete=false;initialStop=Stop();finalStop=Stop();readyConsecutive=0;encoderX=encoderY=encoderYaw=unwrappedYaw=0;
    targetV=index==1?0:p.at("forward_speed_m_s");targetW=index==0?0:p.at(index==1?"spin_yaw_rate_rad_s":"arc_yaw_rate_rad_s");
    Send(0,0);Status();
  }
  void Request() {
    std::ifstream in(directory+"/phase.request");std::string id,name;if(!(in>>id>>name)||id==lastRequest)return;
    int index=-1;for(int i=0;i<3;++i)if(name==Names[i])index=i;
    if(index<0||id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos)
      throw std::runtime_error("Invalid finite differential phase request");
    lastRequest=id;autoMode=false;error.clear();Start(id,index);
  }
  void Begin(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    try {
      stamp=info.simTime.Double();dt=lastClock<0?0:stamp-lastClock;lastClock=stamp;
      if(std::abs(dt-.001)>1e-8){if(active&&dt>0)throw std::runtime_error("Actual physics dt is not 0.001 s");return;}
      Request();
      if(!active&&autoMode&&publisher->HasConnections()){
        const int count=mode=="slip"?1:3;
        if(autoIndex<count){int index=autoIndex++;Start("demo-"+std::to_string(serial)+"-"+std::to_string(index),index);}
        else {autoMode=false;Status();}
      }
      if(!active)return;
      if(stage=="ramp_up")Send(targetV*std::min(1.,(stageStep+1)/500.),targetW*std::min(1.,(stageStep+1)/500.));
      else if(stage=="ramp_down")Send(targetV*std::max(0.,1.-(stageStep+1)/500.),targetW*std::max(0.,1.-(stageStep+1)/500.));
      else if(stage=="settling"||stage=="measure"){if(stageStep%10==0)Send(targetV,targetW);}
      else if(stageStep%10==0)Send(0,0);
    }catch(const std::exception &e){Fail(e.what());}
  }
  void Before(const common::UpdateInfo &) {
    std::lock_guard<std::mutex> lock(mutex);if(closing||!active||std::abs(dt-.001)>1e-8)return;
    try {
      before=Read();
      if(stage=="measure"&&measureStep==0){baseline=before;baselineStamp=stamp;previousYaw=before.yaw;encoderX=encoderY=encoderYaw=unwrappedYaw=0;}
    }catch(const std::exception &e){Fail(e.what());}
  }
  bool Stopped(const Sample &s,Stop &evidence) {
    evidence.duration+=dt;evidence.wl=s.wl;evidence.wr=s.wr;
    evidence.linear=std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);evidence.wz=s.wz;
    bool good=std::abs(s.wl)<.1&&std::abs(s.wr)<.1&&evidence.linear<.01&&std::abs(s.wz)<.03;
    evidence.consecutive=good?evidence.consecutive+dt:0;
    evidence.confirmed=evidence.consecutive>=.05-1e-9;
    if(!evidence.confirmed&&evidence.duration>2+1e-9)throw std::runtime_error("Actual wheel/body zero-command stop was not confirmed within 2 s");
    return evidence.confirmed;
  }
  void End() {
    std::lock_guard<std::mutex> lock(mutex);if(closing||!active||std::abs(dt-.001)>1e-8)return;
    try {
      auto after=Read();++phaseStep;++stageStep;
      if(std::abs(after.roll)>.25||std::abs(after.pitch)>.25)throw std::runtime_error("Actual differential chassis tipped; inspect contact/model before grading");
      if(stage=="measure") {
        ++measureStep;
        double dl=geometry.at("wheel_radius_m")*Delta(after.ql-before.ql),dr=geometry.at("wheel_radius_m")*Delta(after.qr-before.qr);
        double ds=(dl+dr)*.5,da=(dr-dl)/geometry.at("wheel_separation_m");
        double sinc=std::abs(da)<1e-10?1:std::sin(da*.5)/(da*.5);
        encoderX+=ds*sinc*std::cos(encoderYaw+da*.5);encoderY+=ds*sinc*std::sin(encoderYaw+da*.5);encoderYaw+=da;
        unwrappedYaw+=Delta(after.yaw-previousYaw);previousYaw=after.yaw;
      }
      csv<<std::setprecision(17)<<episode<<','<<phase<<','<<stage<<','<<phaseStep<<','<<phaseStep*.001<<','
        <<(stage=="measure"?measureStep:0)<<','<<(stage=="measure"?measureStep*.001:0)<<','<<stamp<<','<<dt<<','
        <<commandV<<','<<commandW<<','<<legacyX<<','<<legacyYaw<<','
        <<before.ql<<','<<before.qr<<','<<before.wl<<','<<before.wr<<','<<after.ql<<','<<after.qr<<','<<after.wl<<','<<after.wr<<','
        <<before.x<<','<<before.y<<','<<before.yaw<<','<<after.x<<','<<after.y<<','<<after.yaw<<','<<unwrappedYaw<<','<<after.z<<','
        <<after.vx<<','<<after.vy<<','<<after.vz<<','<<after.wz<<','<<after.roll<<','<<after.pitch<<','
        <<encoderX<<','<<encoderY<<','<<encoderYaw<<','<<configSha<<'\n';
      if(phaseStep%100==0||world->IsPaused())csv.flush();
      if(stage=="stopping_initial"&&Stopped(after,initialStop)){stage="ramp_up";stageStep=0;}
      else if(stage=="ramp_up"&&stageStep>=500){stage="settling";stageStep=0;}
      else if(stage=="settling") {
        double wl=(targetV-targetW*geometry.at("wheel_separation_m")*.5)/geometry.at("wheel_radius_m");
        double wr=(targetV+targetW*geometry.at("wheel_separation_m")*.5)/geometry.at("wheel_radius_m");
        bool ready=std::abs(after.wl-wl)<std::max(.1,.05*std::abs(wl))&&std::abs(after.wr-wr)<std::max(.1,.05*std::abs(wr));
        readyConsecutive=ready?readyConsecutive+dt:0;
        if(stageStep>=300&&readyConsecutive>=.05-1e-9){stage="measure";stageStep=0;}
        else if(stageStep>2000)throw std::runtime_error("Native stock wheel-speed command did not become ready");
      }
      else if(stage=="measure"&&measureStep>=2000){stage="ramp_down";stageStep=0;}
      else if(stage=="ramp_down"&&stageStep>=500){stage="stopping_final";stageStep=0;Send(0,0);}
      else if((stage=="stopping_final"||stage=="reset-stopping")&&Stopped(after,finalStop)){
        bool reset=stage=="reset-stopping";stage=reset?"stopped":"finished";complete=!reset;active=false;
        if(reset){resetZeroConfirmed=true;std::ofstream log(directory+"/reset-events.jsonl",std::ios::app);log<<"{\"event\":\"actual_zero_confirmed\",\"episode\":"<<Quote(episode)<<",\"stop\":"<<StopJSON(finalStop)<<"}\n";}
        csv.flush();Status();
      }
      if(world->IsPaused()||stamp-lastStatus>=.1)Status();
    }catch(const std::exception &e){Fail(e.what());}
  }
  void Fail(const std::string &message) {error=message;Send(0,0);active=false;autoMode=false;stage="error";csv.flush();Status();gzerr<<message<<'\n';}
  void Reset() {
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    Send(0,0);autoMode=false;active=true;complete=false;stage="reset-stopping";phase="reset";episode="reset-"+std::to_string(++serial);
    ++resetCount;resetZeroConfirmed=false;baseline=Sample();baselineStamp=0;encoderX=encoderY=encoderYaw=unwrappedYaw=0;
    phaseStep=stageStep=measureStep=0;initialStop=Stop();finalStop=Stop();error.clear();lastClock=-1;
    std::ofstream log(directory+"/reset-events.jsonl",std::ios::app);log<<"{\"event\":\"public_reset_zero_published\",\"episode\":"<<Quote(episode)<<",\"sim_time_s\":"<<std::setprecision(17)<<world->SimTime().Double()<<"}\n";
    Status();
  }
};
}
class DifferentialLesson : public ModelPlugin {
  std::shared_ptr<State> state;event::ConnectionPtr begin,before,end;
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    state=std::make_shared<State>();auto s=state;s->model=model;s->world=model->GetWorld();
    s->directory=Env("GAZEBOTEACH_SESSION_DIR")+"/control";mkdir(s->directory.c_str(),0755);
    std::ifstream conf(Env("GAZEBOTEACH_WORKSPACE")+"/build/drive_lab.conf");
    std::getline(conf,s->configSha);std::getline(conf,s->worldSha);std::getline(conf,s->sourceSha);std::getline(conf,s->stockSha);
    std::string key;conf>>key>>s->mode;double value;while(conf>>key>>value)s->p[key]=value;
    if(s->p.size()!=4||s->configSha.size()!=64||s->sourceSha.size()!=64)throw std::runtime_error("Missing frozen drive build configuration");
    s->chassis=model->GetLink("chassis");s->leftWheel=model->GetLink("left_wheel");s->rightWheel=model->GetLink("right_wheel");s->caster=model->GetLink("caster");
    s->left=model->GetJoint("left_joint");s->right=model->GetJoint("right_joint");
    if(!s->chassis||!s->leftWheel||!s->rightWheel||!s->caster||!s->left||!s->right)throw std::runtime_error("Missing actual fixed rig links/joints");
    if(s->left->GetParent()!=s->chassis||s->right->GetParent()!=s->chassis||s->left->GetChild()!=s->leftWheel||s->right->GetChild()!=s->rightWheel)throw std::runtime_error("Actual differential joint topology differs");
    s->leftAxis=s->left->GlobalAxis(0);s->rightAxis=s->right->GlobalAxis(0);s->leftAnchor=s->left->Anchor(0);s->rightAnchor=s->right->Anchor(0);
    s->geometry["left_collision_bbox_max_m"]=s->leftWheel->BoundingBox().Size().Max();s->geometry["right_collision_bbox_max_m"]=s->rightWheel->BoundingBox().Size().Max();
    s->geometry["wheel_radius_m"]=s->geometry["left_collision_bbox_max_m"]*.5;
    s->geometry["wheel_separation_m"]=s->leftAnchor.Distance(s->rightAnchor);
    s->geometry["left_mass_kg"]=s->leftWheel->GetInertial()->Mass();s->geometry["right_mass_kg"]=s->rightWheel->GetInertial()->Mass();
    s->geometry["chassis_mass_kg"]=s->chassis->GetInertial()->Mass();s->geometry["caster_mass_kg"]=s->caster->GetInertial()->Mass();
    s->geometry["physics_step_s"]=s->world->Physics()->GetMaxStepSize();
    for(auto item:std::map<std::string,physics::LinkPtr>{{"left",s->leftWheel},{"right",s->rightWheel},{"chassis",s->chassis},{"caster",s->caster}}){
      auto inertia=item.second->GetInertial();
      s->geometry[item.first+"_Ixx_kg_m2"]=inertia->IXX();s->geometry[item.first+"_Iyy_kg_m2"]=inertia->IYY();s->geometry[item.first+"_Izz_kg_m2"]=inertia->IZZ();
      auto cog=inertia->CoG();s->geometry[item.first+"_com_x_m"]=cog.X();s->geometry[item.first+"_com_y_m"]=cog.Y();s->geometry[item.first+"_com_z_m"]=cog.Z();
    }
    for(auto item:std::map<std::string,physics::LinkPtr>{{"left",s->leftWheel},{"right",s->rightWheel}}){
      auto shape=boost::dynamic_pointer_cast<physics::CylinderShape>(item.second->GetCollision("wheel_collision")->GetShape());
      if(!shape)throw std::runtime_error("Actual wheel collision is not a cylinder");
      s->geometry[item.first+"_collision_radius_m"]=shape->GetRadius();s->geometry[item.first+"_collision_length_m"]=shape->GetLength();
    }
    s->node.reset(new transport::Node());s->node->Init(s->world->Name());s->publisher=s->node->Advertise<msgs::Pose>("~/"+model->GetName()+"/vel_cmd");
    s->csv.open(s->directory+"/drive.csv");s->csv<<"episode,phase,stage,phase_step,phase_elapsed_s,measurement_step,measurement_elapsed_s,sim_time_s,dt_s,command_v_m_s,command_yaw_rate_rad_s,legacy_pose_x,legacy_pose_yaw,left_q_before_rad,right_q_before_rad,left_w_before_rad_s,right_w_before_rad_s,left_q_after_rad,right_q_after_rad,left_w_after_rad_s,right_w_after_rad_s,gt_x_before_m,gt_y_before_m,gt_yaw_before_rad,gt_x_after_m,gt_y_after_m,gt_yaw_after_rad,gt_yaw_unwrapped_rad,gt_z_m,body_vx_m_s,body_vy_m_s,body_vz_m_s,body_yaw_rate_rad_s,body_roll_rad,body_pitch_rad,encoder_x_m,encoder_y_m,encoder_yaw_rad,config_sha256\n";s->csv.flush();s->Status();
    begin=event::Events::ConnectWorldUpdateBegin([s](const common::UpdateInfo &info){s->Begin(info);});
    before=event::Events::ConnectBeforePhysicsUpdate([s](const common::UpdateInfo &info){s->Before(info);});
    end=event::Events::ConnectWorldUpdateEnd([s](){s->End();});
  }
  void Reset() override {if(state)state->Reset();}
  ~DifferentialLesson() override {if(state){std::lock_guard<std::mutex> lock(state->mutex);state->closing=true;state->csv.flush();}begin.reset();before.reset();end.reset();}
};
GZ_REGISTER_MODEL_PLUGIN(DifferentialLesson)
}
