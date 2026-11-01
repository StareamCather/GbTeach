// Two real revolute joints: Cartesian IK sets only torque-controller references.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Events.hh>
#include <boost/weak_ptr.hpp>
#include <cmath>
#include <mutex>
#include <sstream>

namespace gazebo {
class PushArm : public ModelPlugin {
 public:
  ~PushArm() override {
    update.reset(); subscriber.reset(); publisher.reset();
    if(node) node->Fini(); node.reset();
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    auto one=model->GetJoint("shoulder_joint"),two=model->GetJoint("elbow_joint");
    if(!one||!two) { gzerr<<"L37_ARM_ERROR missing physical joints\n";return; }
    shoulder=one; elbow=two;
    node.reset(new transport::Node());node->Init(model->GetWorld()->Name());
    publisher=node->Advertise<msgs::Any>("~/lesson37/arm/state",10);
    subscriber=node->Subscribe("~/lesson37/arm/command",&PushArm::Command,this);
    update=event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &info) {
      std::lock_guard<std::mutex> guard(mutex);
      auto one=shoulder.lock(),two=elbow.lock();if(!one||!two)return;
      sim=info.simTime.Double();q1=one->Position(0);q2=two->Position(0);
      dq1=one->GetVelocity(0);dq2=two->GetVelocity(0);
      if(pending) { start=sim;pending=false; }
      if(mode=="trajectory") {
        double u=std::max(0.,std::min(1.,(sim-start-.8)/duration));
        desiredX=.4+.24*u*u*(3-2*u);
        const double physical2=std::acos((desiredX*desiredX-.245)/.245);
        const double physical1=-physical2/2.;
        target1=physical1+offset;target2=physical2-2*offset;
      }
      tau1=enabled&&mode!="zero"?clip(100*(target1-q1)-4*dq1):0;
      tau2=enabled&&mode!="zero"?clip(70*(target2-q2)-2*dq2):0;
      one->SetForce(0,tau1);two->SetForce(0,tau2);
      if(sim<last||sim-last>=.02) { last=sim;Publish(""); }
    });
    gzmsg<<"L37_ARM_LOADED real 2R SetForce PD kp100/70 kd4/2\n";
  }
  void Reset() override {
    std::lock_guard<std::mutex> guard(mutex);
    pending=false;mode="hold";enabled=true;target1=target2=0;desiredX=.4;start=-1;last=-1;sim=0;
    q1=q2=dq1=dq2=tau1=tau2=0;
  }
 private:
  double clip(double value) const { return std::max(-cap,std::min(cap,value)); }
  void Command(ConstAnyPtr &message) {
    if(message->type()!=msgs::Any::STRING)return;
    std::string action,id,extra;int value;
    std::istringstream in(message->string_value());
    if(!(in>>action>>value>>id)||(in>>extra)||id.size()>80)return;
    std::lock_guard<std::mutex> guard(mutex);
    if(action=="enable"&&(value==0||value==1))enabled=value;
    else if(action=="cap"&&(value==3||value==6||value==9))cap=value;
    else if(action=="duration"&&(value==4||value==6||value==8))duration=value;
    else if(action=="start"&&value==0) { mode="trajectory";pending=true;start=-1; }
    else if(action=="hold"&&value==0) { mode="hold";pending=false; }
    else if(action=="stop"&&value==0) { mode="zero";pending=false; }
    else if(action!="status"||value!=0)return;
    Publish(id); // Paused command acknowledgement does not need a sensor tick.
  }
  void Publish(const std::string &id) {
    std::ostringstream out;out.precision(17);
    out<<"{\"id\":\""<<id<<"\",\"sim_time\":"<<sim<<",\"start_sim\":"<<start
       <<",\"mode\":\""<<mode<<"\",\"enabled\":"<<(enabled?"true":"false")
       <<",\"torque_limit_nm\":"<<cap<<",\"push_duration_s\":"<<duration
       <<",\"q1\":"<<q1<<",\"q2\":"<<q2<<",\"dq1\":"<<dq1<<",\"dq2\":"<<dq2
       <<",\"target_q1\":"<<target1<<",\"target_q2\":"<<target2
       <<",\"tau1\":"<<tau1<<",\"tau2\":"<<tau2<<",\"desired_x\":"<<desiredX<<",\"desired_y\":0}";
    msgs::Any message;message.set_type(msgs::Any::STRING);message.set_string_value(out.str());publisher->Publish(message);
  }
  const double offset=std::acos(4./7.);
  boost::weak_ptr<physics::Joint> shoulder,elbow;
  transport::NodePtr node;transport::PublisherPtr publisher;transport::SubscriberPtr subscriber;
  event::ConnectionPtr update;std::mutex mutex;
  bool enabled=true,pending=false;std::string mode="hold";
  double q1=0,q2=0,dq1=0,dq2=0,target1=0,target2=0,tau1=0,tau2=0;
  double sim=0,start=-1,last=-1,cap=6,duration=6,desiredX=.4;
};
GZ_REGISTER_MODEL_PLUGIN(PushArm)
}
