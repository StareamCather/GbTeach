// A physical prismatic door. Commands select a target, never a pose or velocity.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Events.hh>
#include <boost/weak_ptr.hpp>
#include <atomic>
#include <cmath>
#include <mutex>
#include <sstream>

namespace gazebo {
class WarehouseDoor : public ModelPlugin {
 public:
  ~WarehouseDoor() override {
    update.reset(); subscriber.reset(); publisher.reset();
    if(node) node->Fini(); node.reset(); joint.reset();
    // The gzserver owns transport. This plugin must never shut it down.
  }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    auto actual=model->GetJoint("slide_joint");
    if(!actual) { gzerr<<"L36_DOOR_ERROR missing slide_joint\n"; return; }
    cap=sdf->Get<double>("force_cap");
    if(!std::isfinite(cap.load())||cap<12||cap>30) { gzerr<<"L36_DOOR_ERROR invalid force_cap\n"; return; }
    joint=actual; node.reset(new transport::Node()); node->Init(model->GetWorld()->Name());
    publisher=node->Advertise<msgs::Any>("~/lesson36/door/state",10);
    subscriber=node->Subscribe("~/lesson36/door/command",&WarehouseDoor::Command,this);
    update=event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &info) {
      auto actual=joint.lock(); if(!actual) return;
      const double position=actual->Position(0), velocity=actual->GetVelocity(0);
      const double target=(enabled.load()&&open.load())?1.25:0.0;
      double force=120*(target-position)-20*velocity;
      const double bound=cap.load();force=std::max(-bound,std::min(bound,force));
      actual->SetForce(0,force);
      q=position; v=velocity; f=force; sim=info.simTime.Double();
      if(sim.load()<last || sim.load()-last>=.05) { last=sim; Publish(""); }
    });
    gzmsg<<"L36_DOOR_LOADED SetForce PD kp120 kd20 cap="<<cap.load()<<" N\n";
  }
  void Reset() override { open=false; enabled=true; last=-1; }
 private:
  void Command(ConstAnyPtr &message) {
    if(message->type()!=msgs::Any::STRING) return;
    std::istringstream in(message->string_value()); std::string action,id,extra; int value=-1;
    if(!(in>>action>>value>>id)||(in>>extra)||id.size()>80) return;
    if(action=="open"&&(value==0||value==1)) open=(value==1);
    else if(action=="enable"&&(value==0||value==1)) enabled=(value==1);
    else if(action=="cap"&&(value==12||value==20||value==30)) cap=value;
    else if(action!="status"||value!=0) return;
    Publish(id); // Ack works even while paused; q is the last update measurement.
  }
  void Publish(const std::string &id) {
    std::lock_guard<std::mutex> guard(output);
    std::ostringstream out; out.precision(17);
    out<<"{\"id\":\""<<id<<"\",\"q\":"<<q.load()<<",\"velocity\":"<<v.load()
       <<",\"force\":"<<f.load()<<",\"sim_time\":"<<sim.load()<<",\"open\":"
       <<(open?"true":"false")<<",\"enabled\":"<<(enabled?"true":"false")<<",\"force_cap\":"<<cap.load()<<"}";
    msgs::Any message; message.set_type(msgs::Any::STRING); message.set_string_value(out.str()); publisher->Publish(message);
  }
  boost::weak_ptr<physics::Joint> joint;
  transport::NodePtr node; transport::PublisherPtr publisher; transport::SubscriberPtr subscriber;
  event::ConnectionPtr update; std::mutex output;
  std::atomic<bool> open{false},enabled{true};
  std::atomic<double> q{0},v{0},f{0},sim{0},cap{20}; double last=-1;
};
GZ_REGISTER_MODEL_PLUGIN(WarehouseDoor)
}
