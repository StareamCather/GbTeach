// The healthy reference applies a real Newton force, never a pose/velocity fix.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <boost/weak_ptr.hpp>
#include <cmath>
#include <iostream>

namespace gazebo {
class FaultForce : public ModelPlugin {
 public:
  ~FaultForce() override { connection.reset(); link.reset(); }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    physics::LinkPtr body=model->GetLink("body");
    if(!body) { gzerr << "L35_FORCE_LOAD_ERROR missing body link\n";return; }
    force=sdf->HasElement("force_x") ? sdf->Get<double>("force_x") : 1.0;
    if(!std::isfinite(force) || std::abs(force)>10) {
      gzerr << "L35_FORCE_LOAD_ERROR force_x must be finite within +/-10N\n";return;
    }
    link=body;
    connection=event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &) {
      physics::LinkPtr body=link.lock();
      if(body) body->AddForce(ignition::math::Vector3d(force,0,0));
    });
    gzmsg << "L35_FORCE_LOADED force_x=" << force << " N; AddForce per UpdateBegin\n";
  }
 private:
  boost::weak_ptr<physics::Link> link;
  event::ConnectionPtr connection;
  double force=1;
};
GZ_REGISTER_MODEL_PLUGIN(FaultForce)
}
