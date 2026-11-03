// Bounded four-lane drag experiment; only recorded owned model IDs are reset.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstdlib>
namespace gazebo {
class DragWind final:public WorldPlugin {
 struct Cart{std::string name;unsigned int id=0;double area=0,mass=0;bool on=false;physics::ModelPtr model;physics::LinkPtr link;ignition::math::Pose3d initial;};
 physics::WorldPtr world;event::ConnectionPtr connection;std::vector<Cart> carts;std::string dir,token,sample="null";double rho,cd,wind,shear,gust,period,start=-1,next=0;long long cycle=-1;unsigned long long sequence=0;bool captured=false;
 double Wind(double age,double y){return wind+shear*y+gust*std::sin(2*3.14159265358979323846*age/period);}
 public:
 ~DragWind() override{connection.reset();carts.clear();world.reset();}
 void Load(physics::WorldPtr w,sdf::ElementPtr s)override{
  world=w;const char *p=std::getenv("GAZEBOTEACH_SESSION_DIR");const char *t=std::getenv("GAZEBOTEACH_SESSION_TOKEN");if(!p||!t)gzthrow("Missing owned session identity");dir=p;token=t;
  rho=s->Get<double>("density");cd=s->Get<double>("cd");wind=s->Get<double>("wind");shear=s->Get<double>("shear");gust=s->Get<double>("gust");period=s->Get<double>("period");
  if(!std::isfinite(rho+cd+wind+shear+gust+period)||rho<=0||cd<=0||period<=0)gzthrow("Invalid finite wind configuration");
  for(auto e=s->GetElement("cart");e;e=e->GetNextElement("cart")){
   Cart c;c.name=e->Get<std::string>("name");c.area=e->Get<double>("area");c.on=e->Get<bool>("wind_on");c.model=world->ModelByName(c.name);if(!c.model||c.area<=0)gzthrow("Missing actual wind cart");
   c.id=c.model->GetId();c.link=c.model->GetLink("body");if(!c.link)gzthrow("Missing actual cart body");c.mass=c.link->GetInertial()->Mass();c.initial=c.model->WorldPose();carts.push_back(c);
  }
  connection=event::Events::ConnectWorldUpdateBegin(std::bind(&DragWind::Update,this,std::placeholders::_1));
 }
 void Update(const common::UpdateInfo &info){
  const double now=info.simTime.Double();if(start<0||now<start){start=now;cycle=-1;sample="null";}
  const auto current=static_cast<long long>((now-start)/4.0);const double age=now-start-current*4.0;
  if(current!=cycle){cycle=current;captured=false;for(auto &c:carts){auto m=world->ModelByName(c.name);if(!m||m->GetId()!=c.id)continue;c.model->SetWorldPose(c.initial);c.link->SetLinearVel(ignition::math::Vector3d::Zero);c.link->SetAngularVel(ignition::math::Vector3d::Zero);}}
  bool capturing=!captured&&age>=1.0;std::ostringstream data;data<<std::setprecision(17)<<"{\"cycle\":"<<cycle<<",\"elapsed_s\":"<<age<<",\"carts\":[";bool comma=false;
  for(auto &c:carts){auto m=world->ModelByName(c.name);if(!m||m->GetId()!=c.id)continue;
   const auto p=c.link->WorldCoGPose().Pos();const auto v=c.link->WorldLinearVel();const double w=c.on?Wind(age,c.initial.Pos().Y()):0;const double u=w-v.X();double force=.5*rho*cd*c.area*u*std::abs(u);force=std::max(-20.0,std::min(20.0,force));
   c.link->AddForce(ignition::math::Vector3d(force,0,0));
   if(comma)data<<',';comma=true;data<<"{\"name\":\""<<c.name<<"\",\"model_id\":"<<c.id<<",\"mass_kg\":"<<c.mass<<",\"area_m2\":"<<c.area<<",\"wind_on\":"<<(c.on?"true":"false")<<",\"y_m\":"<<c.initial.Pos().Y()<<",\"displacement_m\":"<<p.X()-c.initial.Pos().X()<<",\"velocity_m_s\":"<<v.X()<<",\"wind_m_s\":"<<w<<",\"force_n\":"<<force<<'}';
  }data<<"]}";if(capturing){sample=data.str();captured=true;}
  if(now<next&&now>next-.2)return;next=now+.05;++sequence;
  std::string filename=dir+"/wind.json";{std::ofstream out(filename+".tmp");out<<std::setprecision(17)<<"{\"sequence\":"<<sequence<<",\"session_token\":\""<<token<<"\",\"sim_time_s\":"<<now<<",\"current\":"<<data.str()<<",\"sample_at_one_second\":"<<sample<<"}";if(!out)return;}std::rename((filename+".tmp").c_str(),filename.c_str());
 }
};GZ_REGISTER_WORLD_PLUGIN(DragWind)
}
