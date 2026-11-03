// A53: ODE queries one simultaneous 16-beam column per real physics step.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <locale>
#include <regex>
#include <sstream>
#include <vector>

namespace gazebo {
namespace {
constexpr int Beams=16, Columns=200, Scans=340;
constexpr int64_t StepNs=500000, SettleNs=500000000;
constexpr double Pi=3.14159265358979323846, Near=.3, Far=22.;
struct Point { ignition::math::Vector3d local; int beam, reflectivity; int64_t offset; };
}

class RollingLidar final : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr cfg) override {
    const char *token=std::getenv("GAZEBOTEACH_SESSION_TOKEN");
    if (!token || !std::regex_match(token,std::regex("[0-9a-f]{32}")))
      gzthrow("A53 sensor requires a GazeboTeach session");
    model=parent;
    body=model->GetLink("sensor_body");
    world=model->GetWorld();
    if (!body || world->Physics()->GetType()!="ode" ||
        std::abs(world->Physics()->GetMaxStepSize()-.0005)>1e-12)
      gzthrow("A53 requires sensor_body and ODE configured at 0.5 ms");
    collision=world->Physics()->CreateCollision("multiray",body->GetScopedName());
    if (!collision) gzthrow("A53 could not create a native multiray collision");
    collision->SetName("gt_rolling_ray_collision");
    collision->SetRelativePose(ignition::math::Pose3d::Zero);
    collision->SetInitialRelativePose(ignition::math::Pose3d::Zero);
    rays=boost::dynamic_pointer_cast<physics::MultiRayShape>(collision->GetShape());
    if (!rays) gzthrow("A53 did not obtain the native ODE MultiRayShape");
    rays->Load(cfg);
    rays->Init();
    if (rays->GetSampleCount()!=1 || rays->GetVerticalSampleCount()!=Beams ||
        std::abs(rays->GetMinRange()-Near)>1e-12 || std::abs(rays->GetMaxRange()-Far)>1e-12)
      gzthrow("A53 multiray configuration does not match the declared sensor");
    points.reserve(Beams*Columns);
    begin=event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &){Control();});
    end=event::Events::ConnectWorldUpdateEnd([this](){Measure();});
    std::ofstream("build/native-ready")<<token<<' '<<GT_PLANT_SOURCE_SHA<<'\n';
  }

private:
  int64_t Now() const { const auto t=world->SimTime(); return int64_t(t.sec)*1000000000+t.nsec; }
  void Control() {
    const auto pose=body->WorldPose();
    const double elapsed=active?double(Now()-startNs)*1e-9:0.;
    // Reference only: ODE integrates the actual force/torque response.
    // Smooth one circle over 24 s, revisit, then advance enough for a new keyframe.
    const double u=std::clamp((elapsed-2.)/24.,0.,1.);
    const double angle=2*Pi*u*u*(3.-2.*u);
    const double rate=2*Pi*(6.*u*(1.-u))/24.;
    constexpr double radius=2.5;
    const double tail=std::clamp((elapsed-27.)/4.,0.,1.);
    const double advance=.95*tail*tail*(3.-2.*tail);
    const double advanceRate=.95*(6.*tail*(1.-tail))/4.;
    const ignition::math::Vector3d target(radius*std::sin(angle)+advance,radius*(1.-std::cos(angle)),1.2);
    const ignition::math::Vector3d desired(radius*rate*std::cos(angle)+advanceRate+2.2*(target.X()-pose.Pos().X()),
        radius*rate*std::sin(angle)+2.2*(target.Y()-pose.Pos().Y()),0.);
    const auto v=body->WorldLinearVel();
    body->AddForce(ignition::math::Vector3d(3*(desired.X()-v.X()),3*(desired.Y()-v.Y()),
        9.81+18*(1.2-pose.Pos().Z())-7*v.Z()));
    const auto e=pose.Rot().Euler();
    const auto w=body->RelativeAngularVel();
    const double yawError=std::remainder(angle-e.Z(),2*Pi);
    const double yawRate=rate+3.*yawError;
    body->AddRelativeTorque(ignition::math::Vector3d(-2*e.X()-.6*w.X(),
        -2*e.Y()-.6*w.Y(),.45*(yawRate-w.Z())));
  }
  void Finish(bool ok, const char *reason) {
    out.flush();
    ok=ok && bool(out);
    out.close();
    std::ofstream receipt("build/capture.done.partial",std::ios::trunc);
    receipt<<"{\"ok\":"<<(ok?"true":"false")<<",\"first_ns\":"<<firstNs
           <<",\"last_ns\":"<<previousNs<<",\"scans\":"<<scanCount
           <<",\"truth_samples\":"<<truthCount<<",\"request_id\":\""<<requestId
           <<"\",\"reason\":\""<<reason<<"\"}\n";
    receipt.close();
    if (std::rename("build/capture.done.partial","build/capture.done")!=0)
      gzerr<<"A53 cannot publish capture completion: "<<std::strerror(errno)<<'\n';
    active=false; points.clear();
  }
  void Measure() {
    const int64_t now=Now();
    if (!active) {
      if (world->Iterations()%20!=0) return;
      std::ifstream request("build/capture.request");
      if (!request) return;
      std::string id; std::getline(request,id); request.close();
      if (std::remove("build/capture.request")!=0 || !std::regex_match(id,std::regex("[0-9a-f]{32}"))) return;
      requestId=id;
      model->SetWorldPose(ignition::math::Pose3d(0,0,1.2,0,0,0));
      model->SetLinearVel(ignition::math::Vector3d::Zero);
      model->SetAngularVel(ignition::math::Vector3d::Zero);
      startNs=now+SettleNs; firstNs=previousNs=0;
      scanCount=column=truthCount=0; points.clear();
      out.clear();
      out.open("build/native.jsonl",std::ios::trunc);
      out.imbue(std::locale::classic());
      out<<std::setprecision(10);
      active=bool(out);
      if (!active) Finish(false,"capture file unavailable");
      return;
    }
    if (previousNs && now<=previousNs) {
      if (now<previousNs) Finish(false,"world time regressed");
      return;
    }
    if (now<startNs) return;
    if (previousNs && now!=previousNs+StepNs) { Finish(false,"missing physical sample"); return; }
    previousNs=now;
    if (!firstNs) firstNs=now;
    if (column==0) scanNs=now;
    if ((now-firstNs)%(20*StepNs)==0) {
      const auto pose=body->WorldPose(); const auto p=pose.Pos(); const auto q=pose.Rot();
      out<<"{\"type\":\"truth\",\"stamp_ns\":"<<now<<",\"position\":["<<p.X()<<','<<p.Y()<<','<<p.Z()
         <<"],\"orientation\":["<<q.X()<<','<<q.Y()<<','<<q.Z()<<','<<q.W()<<"]}\n";
      ++truthCount;
    }
    const double yaw=2*Pi*column/Columns;
    const ignition::math::Vector3d lidarInImu(.04165,.02326,-.0284);
    ignition::math::Vector3d directions[Beams];
    for (int i=0;i<Beams;++i) {
      const double elevation=(-22.+44.*i/(Beams-1))*Pi/180.;
      auto &d=directions[i];
      d.Set(std::cos(elevation)*std::cos(yaw),std::cos(elevation)*std::sin(yaw),std::sin(elevation));
      rays->SetRay(i,lidarInImu+Near*d,lidarInImu+Far*d);
    }
    rays->Update();
    for (int i=0;i<Beams;++i) {
      const double distance=rays->GetRange(i);
      if (std::isfinite(distance) && distance>=Near && distance<Far-1e-6) {
        const double retro=rays->GetRetro(i);
        const auto intensity=std::isfinite(retro)?int(std::clamp(retro,0.,255.)):0;
        points.push_back({distance*directions[i],i,intensity,now-scanNs});
      }
    }
    if (++column==Columns) {
      out<<"{\"type\":\"scan\",\"stamp_ns\":"<<scanNs<<",\"points\":[";
      bool first=true;
      for (const auto &sample:points) {
        if (!first) out<<',';
        first=false;
        const auto &p=sample.local;
        out<<'['<<p.X()<<','<<p.Y()<<','<<p.Z()<<','<<sample.reflectivity<<','<<sample.beam<<','<<sample.offset<<']';
      }
      out<<"]}\n";
      points.clear(); column=0;
      if (++scanCount==Scans || !out) Finish(bool(out),"capture finished");
    }
  }
  physics::ModelPtr model;
  physics::LinkPtr body;
  physics::WorldPtr world;
  physics::CollisionPtr collision;
  physics::MultiRayShapePtr rays;
  std::ofstream out;
  std::vector<Point> points;
  int64_t startNs=0,firstNs=0,previousNs=0,scanNs=0;
  int scanCount=0,column=0,truthCount=0;
  bool active=false;
  std::string requestId;
  event::ConnectionPtr begin,end;
};
GZ_REGISTER_MODEL_PLUGIN(RollingLidar)
}
