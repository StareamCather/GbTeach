// Read-only SDK observer. Native Model.Reset/Actor animation and ODE alone move
// the scene. This helper never writes a body's pose, velocity, force or height.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/HeightmapShape.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/common/Skeleton.hh>
#include <gazebo/common/SkeletonAnimation.hh>
#include <gazebo/common/Mesh.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace gazebo { namespace {
using J=google::protobuf::Value;
J Obj(){J j;j.mutable_struct_value();return j;}
J Arr(){J j;j.mutable_list_value();return j;}
J Num(double n){if(!std::isfinite(n))throw std::runtime_error("Non-finite actual SDK number");J j;j.set_number_value(n);return j;}
J Str(const std::string &s){J j;j.set_string_value(s);return j;}
J Bool(bool b){J j;j.set_bool_value(b);return j;}
void Put(J &o,const std::string &k,const J &v){(*o.mutable_struct_value()->mutable_fields())[k]=v;}
void Add(J &a,const J &v){*a.mutable_list_value()->add_values()=v;}
const J &Get(const J &o,const std::string &k){return o.struct_value().fields().at(k);}
J Vec(const ignition::math::Vector3d &v){J a=Arr();Add(a,Num(v.X()));Add(a,Num(v.Y()));Add(a,Num(v.Z()));return a;}
J Pose(const ignition::math::Pose3d &p){J a=Vec(p.Pos());for(double n:{p.Rot().W(),p.Rot().X(),p.Rot().Y(),p.Rot().Z()})Add(a,Num(n));return a;}
std::string Text(const J &j){std::string s;auto r=google::protobuf::util::MessageToJsonString(j,&s);if(!r.ok())throw std::runtime_error(r.ToString());return s;}
J Decode(const std::string &s){J j;auto r=google::protobuf::util::JsonStringToMessage(s,&j);if(!r.ok())throw std::runtime_error("Invalid build metadata: "+r.ToString());return j;}
J Message(const google::protobuf::Message &m){std::string s;google::protobuf::util::JsonPrintOptions o;o.preserve_proto_field_names=true;auto r=google::protobuf::util::MessageToJsonString(m,&s,o);if(!r.ok())throw std::runtime_error(r.ToString());return Decode(s);}
std::string Env(const char *k){const char *v=std::getenv(k);if(!v)throw std::runtime_error(std::string("Missing ")+k);return v;}
const char *ids[]={"probe_nw","probe_ne","probe_sw","probe_se","probe_matte","probe_emissive"};
struct Observer {
  std::mutex mutex;physics::WorldPtr world;std::map<std::string,physics::LinkPtr> probes;
  physics::HeightmapShapePtr terrain;physics::RayShapePtr ray;
  boost::shared_ptr<physics::Actor> walker,marker;
  std::string directory,run,error;J build,model,last=Obj(),resetBefore=Obj();
  unsigned long long epoch=0,resets=0,frame=0;double start=0,lastSample=-1;
  std::map<std::string,double> lastScript;std::map<std::string,bool> hasFrame;std::map<std::string,unsigned long long> updates;
  bool closing=false,resetPending=false;std::ofstream frames,tail,resetLog;

  J ActorMeta(const boost::shared_ptr<physics::Actor> &a){
    J o=Obj();Put(o,"name",Str(a->GetName()));Put(o,"is_static",Bool(a->IsStatic()));Put(o,"self_collide",Bool(a->GetSelfCollide()));
    Put(o,"wind_mode",Bool(a->WindMode()));J links=Arr();unsigned collisions=0;
    for(auto &l:a->GetLinks()){J v=Obj();Put(v,"name",Str(l->GetName()));Put(v,"is_static",Bool(l->IsStatic()));Put(v,"gravity",Bool(l->GetGravityMode()));Put(v,"kinematic",Bool(l->GetKinematic()));Put(v,"collision_count",Num(l->GetCollisions().size()));collisions+=l->GetCollisions().size();Add(links,v);}
    Put(o,"links",links);Put(o,"collision_count",Num(collisions));auto mesh=a->Mesh();Put(o,"mesh_node_count",Num(mesh&&mesh->HasSkeleton()?mesh->GetSkeleton()->GetNumNodes():0));
    J animations=Arr();for(auto &x:a->SkeletonAnimations()){if(!x.second)continue;J n=Obj();Put(n,"name",Str(x.first));Put(n,"length_s",Num(x.second->GetLength()));Put(n,"node_count",Num(x.second->GetNodeCount()));Add(animations,n);}Put(o,"animations",animations);return o;
  }
  void Init(){
    auto hm=world->ModelByName("terrain");if(!hm||!hm->GetLink("terrain_link"))throw std::runtime_error("Missing terrain model/link");
    auto collision=hm->GetLink("terrain_link")->GetCollision("terrain_collision");if(!collision)throw std::runtime_error("Missing terrain collision");
    terrain=boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());if(!terrain)throw std::runtime_error("Actual collision is not a native HeightmapShape");
    model=Obj();J h=Obj();Put(h,"uri",Str(terrain->GetURI()));Put(h,"size_m",Vec(terrain->Size()));Put(h,"pos_m",Vec(terrain->Pos()));Put(h,"sampling",Num(terrain->GetSubSampling()));
    auto count=terrain->VertexCount();J vc=Arr();Add(vc,Num(count.X()));Add(vc,Num(count.Y()));Put(h,"vertex_count",vc);Put(h,"collision_name",Str(collision->GetScopedName()));
    J heights=Arr();const int xx[]={16,48,16,48},yy[]={16,16,48,48};for(int i=0;i<4;++i){J e=Obj();Put(e,"id",Str(ids[i]));Put(e,"ix",Num(xx[i]));Put(e,"iy",Num(yy[i]));Put(e,"height_m",Num(terrain->GetHeight(xx[i],yy[i])));Add(heights,e);}Put(h,"indexed_heights",heights);Put(model,"heightmap",h);
    Put(model,"physics_step_s",Num(world->Physics()->GetMaxStepSize()));Put(model,"gravity_m_s2",Vec(world->Gravity()));
    J ps=Arr();for(auto id:ids){auto m=world->ModelByName(id);if(!m)throw std::runtime_error(std::string("Missing probe ")+id);auto l=m->GetLink("ball");if(!l)throw std::runtime_error("Missing probe ball");probes[id]=l;auto c=l->GetCollision("ball_collision");auto shape=boost::dynamic_pointer_cast<physics::SphereShape>(c->GetShape());if(!shape)throw std::runtime_error("Probe collision is not a sphere");J p=Obj();Put(p,"id",Str(id));Put(p,"initial_position_m",Vec(l->WorldPose().Pos()));Put(p,"radius_m",Num(shape->GetRadius()));Put(p,"mass_kg",Num(l->GetInertial()->Mass()));Put(p,"is_static",Bool(l->IsStatic()));Put(p,"gravity",Bool(l->GetGravityMode()));Put(p,"kinematic",Bool(l->GetKinematic()));Add(ps,p);}Put(model,"probes",ps);
    walker=boost::dynamic_pointer_cast<physics::Actor>(world->ModelByName("walker"));marker=boost::dynamic_pointer_cast<physics::Actor>(world->ModelByName("marker"));
    if(!walker||!marker||!walker->GetLink("Hips")||!walker->GetLink("LeftFoot")||!walker->GetLink("RightFoot"))throw std::runtime_error("Native walker/marker or actual skeleton links missing");
    J aa=Obj();Put(aa,"walker",ActorMeta(walker));Put(aa,"marker",ActorMeta(marker));Put(model,"actors",aa);
    J support=Arr();for(const char *n:{"matte_support","emissive_support"}){auto m=world->ModelByName(n);auto l=m->GetLink("support");auto c=l->GetCollision("support_collision");auto b=boost::dynamic_pointer_cast<physics::BoxShape>(c->GetShape());if(!b)throw std::runtime_error("Support collision not BoxShape");J s=Obj();Put(s,"name",Str(n));Put(s,"size_m",Vec(b->Size()));Put(s,"world_pose",Pose(c->WorldPose()));Add(support,s);}Put(model,"supports",support);
    J lights=Arr();for(auto &l:world->Lights()){msgs::Light m;l->FillMsg(m);Add(lights,Message(m));}Put(model,"lights",lights);
    J visuals=Arr();for(const auto &p:std::vector<std::pair<std::string,std::string>>{{"matte_support","support"},{"emissive_support","support"},{"material_exhibits","display"},{"light_exhibits","display"}}){auto l=world->ModelByName(p.first)->GetLink(p.second);auto sdf=l->GetSDF();for(auto v=sdf->GetElement("visual");v;v=v->GetNextElement("visual")){auto name=v->Get<std::string>("name");auto m=l->GetVisualMessage(name);if(!m.has_name())m=l->GetVisualMessage(l->GetScopedName()+"::"+name);J e=Obj();Put(e,"model",Str(p.first));Put(e,"visual",Str(name));Put(e,"message",Message(m));Add(visuals,e);}}Put(model,"visuals",visuals);Put(model,"scene",Message(world->SceneMsg()));
    ray=boost::dynamic_pointer_cast<physics::RayShape>(world->Physics()->CreateShape("ray",physics::CollisionPtr()));if(!ray)throw std::runtime_error("Native physics ray unavailable");
    start=world->SimTime().Double();for(auto n:{"walker","marker"}){lastScript[n]=0;hasFrame[n]=false;updates[n]=0;}
  }
  J ActorFrame(const boost::shared_ptr<physics::Actor> &a){
    auto name=a->GetName();double t=a->ScriptTime();bool changed=std::abs(t-lastScript[name])>1e-10;
    if(changed){hasFrame[name]=true;++updates[name];lastScript[name]=t;}
    J o=Obj();Put(o,"script_time_s",Num(t));Put(o,"active",Bool(a->IsActive()));Put(o,"world_pose",Pose(a->WorldPose()));Put(o,"distinct_update",Bool(changed));Put(o,"has_animation_frame",Bool(hasFrame[name]));Put(o,"distinct_update_count",Num(updates[name]));
    if(name=="walker"){auto hip=a->GetLink("Hips")->WorldPose();Put(o,"left_foot_relative_hips",Pose(hip.Inverse()*a->GetLink("LeftFoot")->WorldPose()));Put(o,"right_foot_relative_hips",Pose(hip.Inverse()*a->GetLink("RightFoot")->WorldPose()));}return o;
  }
  J ProbeFrame(){J pp=Arr();for(auto id:ids){auto l=probes.at(id);J p=Obj();Put(p,"id",Str(id));Put(p,"position_m",Vec(l->WorldPose().Pos()));Put(p,"velocity_m_s",Vec(l->WorldLinearVel()));Add(pp,p);}return pp;}
  J Header(){J row=Obj();Put(row,"run_id",Str(run));Put(row,"epoch",Num(epoch));Put(row,"iteration",Num(world->Iterations()));Put(row,"sim_time_s",Num(world->SimTime().Double()));Put(row,"elapsed_s",Num(world->SimTime().Double()-start));return row;}
  J Sample(const J &pp){
    J row=Header();Put(row,"frame",Num(frame));Put(row,"probes",pp);
    J actors=Obj();Put(actors,"walker",ActorFrame(walker));Put(actors,"marker",ActorFrame(marker));Put(row,"actors",actors);
    J rays=Arr();const double xx[]={-1.,1.,-1.,1.}, yy[]={1.,1.,-1.,-1.};for(int i=0;i<4;++i){double x=xx[i]+.35,y=yy[i],dist=0;std::string entity;ray->SetPoints({x,y,3.},{x,y,-1.});ray->GetIntersection(dist,entity);J r=Obj();Put(r,"id",Str(ids[i]));Put(r,"x_m",Num(x));Put(r,"y_m",Num(y));Put(r,"z_m",Num(3.-dist));Put(r,"entity",Str(entity));Add(rays,r);}Put(row,"rays",rays);return row;
  }
  void Status(){J s=build;Put(s,"schema",Num(1));Put(s,"run_id",Str(run));Put(s,"world",Str(world->Name()));Put(s,"epoch",Num(epoch));Put(s,"reset_count",Num(resets));Put(s,"ready",Bool(error.empty()));Put(s,"error",Str(error));Put(s,"sim_time_s",Num(world->SimTime().Double()));Put(s,"iteration",Num(world->Iterations()));Put(s,"epoch_start_sim_time_s",Num(start));Put(s,"model",model);Put(s,"snapshot",last);Put(s,"reset_pending",Bool(resetPending));std::ofstream out(directory+"/status.json.tmp");out<<Text(s)<<'\n';out.close();std::rename((directory+"/status.json.tmp").c_str(),(directory+"/status.json").c_str());}
  void End(){
    std::lock_guard<std::mutex> lock(mutex);if(closing)return;
    try{
      double now=world->SimTime().Double(),elapsed=now-start;
      bool sample=lastSample<0||now-lastSample>=.02-1e-9,tailWindow=elapsed>=3.-1e-9&&elapsed<=3.5+1e-9;
      if(sample||tailWindow){J pp=ProbeFrame();if(tailWindow){J t=Header();Put(t,"probes",pp);tail<<Text(t)<<'\n';tail.flush();}
        if(sample){++frame;last=Sample(pp);lastSample=now;frames<<Text(last)<<'\n';frames.flush();
          if(resetPending){J e=Obj();Put(e,"run_id",Str(run));Put(e,"epoch",Num(epoch));Put(e,"reset_count",Num(resets));Put(e,"before",resetBefore);Put(e,"after_first_physics_step",last);resetLog<<Text(e)<<'\n';resetLog.flush();resetPending=false;}Status();}
      }
    }catch(const std::exception &e){error=e.what();frames.flush();tail.flush();Status();gzerr<<"Gallery observer: "<<error<<'\n';}
  }
  void Reset(){std::lock_guard<std::mutex> lock(mutex);++epoch;++resets;resetBefore=last;last=Obj();frame=0;start=world->SimTime().Double();lastSample=-1;resetPending=true;error.clear();for(auto n:{"walker","marker"}){lastScript[n]=0;hasFrame[n]=false;updates[n]=0;}frames.flush();tail.flush();Status();}
};
}
class GalleryObserver : public ModelPlugin {
  std::shared_ptr<Observer> s;event::ConnectionPtr end;
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {s=std::make_shared<Observer>();s->world=model->GetWorld();s->run=Env("GAZEBOTEACH_SESSION_TOKEN");s->directory=Env("GAZEBOTEACH_SESSION_DIR")+"/control";mkdir(s->directory.c_str(),0755);std::ifstream in(Env("GAZEBOTEACH_WORKSPACE")+"/build/gallery.json");std::string text((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());s->build=Decode(text);s->Init();s->frames.open(s->directory+"/gallery-frames.jsonl");s->tail.open(s->directory+"/probe-tail.jsonl");s->resetLog.open(s->directory+"/resets.jsonl");s->Status();auto state=s;end=event::Events::ConnectWorldUpdateEnd([state](){state->End();});}
  void Reset() override {if(s)s->Reset();}
  ~GalleryObserver() override {end.reset();if(s){std::lock_guard<std::mutex> lock(s->mutex);s->closing=true;s->frames.flush();s->tail.flush();s->resetLog.flush();}}
};
GZ_REGISTER_MODEL_PLUGIN(GalleryObserver)
}
