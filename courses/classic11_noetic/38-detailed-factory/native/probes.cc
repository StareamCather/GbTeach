// Observe real World model poses after the physics solver; no computed answer poses.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <vector>
namespace gazebo {
class GeometryObserver final:public WorldPlugin {
 physics::WorldPtr world;event::ConnectionPtr connection;std::string dir;double next=0;unsigned long long seq=0;
 public:
 ~GeometryObserver() override {connection.reset();world.reset();}
 void Load(physics::WorldPtr w,sdf::ElementPtr) override {
  world=w;const char *p=std::getenv("GAZEBOTEACH_SESSION_DIR");if(!p)gzthrow("Missing owned session");dir=p;
  connection=event::Events::ConnectWorldUpdateEnd(std::bind(&GeometryObserver::Update,this));
 }
 void Update(){double now=world->SimTime().Double();if(now<next&&now>next-.2)return;next=now+.1;++seq;
  std::ostringstream s;s<<std::setprecision(17)<<"{\"sequence\":"<<seq<<",\"sim_time_s\":"<<now<<",\"probes\":[";bool comma=false;
  for(const std::string &name:{"probe_table","probe_pallet","probe_gap"}){
   auto m=world->ModelByName(name);if(!m)continue;auto p=m->WorldPose().Pos();auto v=m->WorldLinearVel();
   if(comma)s<<',';comma=true;s<<"{\"name\":\""<<name<<"\",\"x\":"<<p.X()<<",\"y\":"<<p.Y()<<",\"z\":"<<p.Z()<<",\"speed\":"<<v.Length()<<'}';
  }s<<"]}";std::string file=dir+"/physics.json";{std::ofstream f(file+".tmp");f<<s.str();if(!f)return;}std::rename((file+".tmp").c_str(),file.c_str());
 }
};GZ_REGISTER_WORLD_PLUGIN(GeometryObserver)
}
