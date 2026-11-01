// Actual GUI input ~/gui; ~/user_camera/pose is output/readback, never a command.
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/IOManager.hh>
#include <gazebo/transport/Connection.hh>
#include <gazebo/gazebo_config.h>
#include <dlfcn.h>
#include <climits>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <thread>
#include <gazebo/msgs/msgs.hh>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <iomanip>
#include <map>
#include <mutex>
#include <string>
#include <stdexcept>

namespace {
std::mutex output;
void Print(const std::string &s){std::lock_guard<std::mutex> lock(output);std::cout<<s<<std::endl;}
std::string Pose(const ignition::math::Pose3d &p){std::ostringstream o;o<<std::setprecision(17)<<'['<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<','<<p.Rot().W()<<','<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z()<<']';return o.str();}
struct Receiver {
  std::mutex mutex;std::condition_variable cv;ignition::math::Pose3d target;bool armed=false,done=false,ready=false;unsigned count=0;ignition::math::Pose3d actual,firstActual;
  void OnPose(ConstPosePtr &m){
    if(!m->IsInitialized())return;auto p=gazebo::msgs::ConvertIgn(*m);if(!p.IsFinite())return;
    std::lock_guard<std::mutex> lock(mutex);++count;
    double dot=p.Rot().W()*target.Rot().W()+p.Rot().X()*target.Rot().X()+p.Rot().Y()*target.Rot().Y()+p.Rot().Z()*target.Rot().Z();
    double pn=std::sqrt(p.Rot().W()*p.Rot().W()+p.Rot().X()*p.Rot().X()+p.Rot().Y()*p.Rot().Y()+p.Rot().Z()*p.Rot().Z());
    double tn=std::sqrt(target.Rot().W()*target.Rot().W()+target.Rot().X()*target.Rot().X()+target.Rot().Y()*target.Rot().Y()+target.Rot().Z()*target.Rot().Z());
    bool unit=std::isfinite(pn)&&std::isfinite(tn)&&std::abs(pn-1)<=1e-6&&std::abs(tn-1)<=1e-6;
    if(unit&&!ready){ready=true;firstActual=p;}
    double normalizedDot=unit?dot/(pn*tn):0;
    bool match=unit&&p.Pos().Distance(target.Pos())<=1e-4&&std::abs(std::abs(normalizedDot)-1)<=1e-8;
    Print(std::string("{\"event\":\"camera_pose\",\"after_publish\":")+(armed?"true":"false")+",\"matches_target\":"+(match?"true":"false")+",\"world_pose\":"+Pose(p)+"}");
    if(armed&&!done&&match){done=true;actual=p;cv.notify_all();}
    if(unit)cv.notify_all();
  }
};
}
using namespace gazebo;
std::string Q(const std::string &v) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:v) {
    if(c=='"'||c=='\\')out<<'\\';
    if(c=='\n')out<<"\\n";else if(c=='\r')out<<"\\r";
    else if(c=='\t')out<<"\\t";else if(c>=32)out<<c;
  }
  return out.str()+'"';
}
// Compatibility for this exact Classic11 SDK, isolated from teaching plugins.
// The exported data symbol is a PRIVATE ABI: fail before transport init if its
// DSO/version cannot be identified. No process exit or timed sleep replaces join.
class MainThreadIOOwner {
 public:
  MainThreadIOOwner() : thread(std::this_thread::get_id()) {
    if (std::string(GAZEBO_VERSION_FULL)!="11.15.1") throw std::runtime_error("IO guard private ABI is validated only on Classic11.15.1 SDK");
    dlerror();
    void *address=dlsym(RTLD_DEFAULT,"_ZN6gazebo9transport10Connection9iomanagerE");
    const char *error=dlerror();
    if(error || !address) throw std::runtime_error("Unsupported Classic11 IOManager ABI symbol");
    Dl_info information{};char canonical[PATH_MAX];
    if(!dladdr(address,&information) || !information.dli_fname ||
       !realpath(information.dli_fname,canonical))
      throw std::runtime_error("Cannot verify Classic11 IOManager DSO");
    runtime=canonical;
    const auto separator=runtime.rfind('/');
    const std::string filename=runtime.substr(separator==std::string::npos?0:separator+1);
    if(filename!=std::string("libgazebo_transport.so.")+GAZEBO_VERSION_FULL)
      throw std::runtime_error(std::string("IOManager runtime DSO differs from compiled SDK: version=")+GAZEBO_VERSION+", full="+GAZEBO_VERSION_FULL+", runtime="+runtime);
    slot=reinterpret_cast<transport::IOManager **>(address);
    // This unconnected main-owned Connection contributes a real shared-manager
    // reference before any asynchronous callbacks exist.
    owner.reset(new transport::Connection());
    io=*slot;
    if(!io || io->GetCount()<1) throw std::runtime_error("IOManager ownership was not established");
  }
  void Stop() {
    if(stopped)return;
    if(std::this_thread::get_id()!=thread || !owner || !io || *slot!=io)
      throw std::runtime_error("IOManager changed or cleanup moved off its owning main thread");
    // Public Stop stops io_service and JOINS its worker. The sentinel prevents
    // a last weakBind temporary on that worker from deleting/joining itself.
    io->Stop();stopped=true;
    if(std::getenv("GAZEBOTEACH_IO_DIAGNOSTICS"))
      std::cerr<<"{\"io_guard_joined\":true,\"runtime_dso\":"<<Q(runtime)<<",\"compiled_version\":"<<Q(GAZEBO_VERSION)<<",\"compiled_version_full\":"<<Q(GAZEBO_VERSION_FULL)<<"}"<<std::endl;
  }
 private:
  std::thread::id thread;transport::ConnectionPtr owner;
  transport::IOManager **slot=nullptr,*io=nullptr;
  bool stopped=false;std::string runtime;
};

int main(int argc,char **argv){
  bool initialized=false;int result=0;
  std::unique_ptr<MainThreadIOOwner> ioOwner;Receiver receiver;transport::NodePtr node;
  transport::SubscriberPtr sub;transport::PublisherPtr pub;
  try{
    std::string station=argc>1?argv[1]:"overview";double timeout=argc>2?std::stod(argv[2]):4;
    if(!std::isfinite(timeout)||timeout<=0||timeout>8)throw std::runtime_error("Camera timeout must be finite within (0,8]");
    const std::map<std::string,ignition::math::Pose3d> views={{"overview",{8,-10,9,0,.62,2.18}},{"materials",{6.8,-3.5,3.1,0,.4,2.35}},{"specular",{5.8,4.3,2.5,0,.6,-2.25}},{"lights",{-6.8,-5,4,0,.5,.98}},{"terrain",{5,-6,7,0,.72,2.26}},{"actors",{3,-7,2.4,0,.3,2.25}}};
    auto found=views.find(station);if(found==views.end())throw std::runtime_error("Unknown gallery station");
    ioOwner.reset(new MainThreadIOOwner());initialized=true;
    if(!gazebo::transport::init("",0,3))throw std::runtime_error("Bounded Classic master setup failed");gazebo::transport::run();
    node.reset(new gazebo::transport::Node());node->Init("default");receiver.target=found->second;
    sub=node->Subscribe("~/user_camera/pose",&Receiver::OnPose,&receiver,false);
    pub=node->Advertise<gazebo::msgs::GUI>("~/gui");
    auto readyDeadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    if(!pub->WaitForConnection(gazebo::common::Time(timeout))||std::chrono::steady_clock::now()>readyDeadline)throw std::runtime_error("GUI not connected: no ~/gui subscriber within readiness budget");
    ignition::math::Pose3d firstActual;
    {std::unique_lock<std::mutex> lock(receiver.mutex);if(!receiver.cv.wait_until(lock,readyDeadline,[&](){return receiver.ready;}))throw std::runtime_error("GUI not connected: no valid actual user_camera/pose within readiness budget");firstActual=receiver.firstActual;}
    Print("{\"event\":\"gui_ready\",\"input_connected\":true,\"actual_pose_seen\":true,\"first_actual_world_pose\":"+Pose(firstActual)+"}");
    gazebo::msgs::GUI message;auto cam=message.mutable_camera();cam->set_name("user_camera");gazebo::msgs::Set(cam->mutable_pose(),receiver.target);
    {std::lock_guard<std::mutex> lock(receiver.mutex);
      pub->Publish(message);
      Print("{\"event\":\"published_once\",\"input_topic\":\"/gazebo/default/gui\",\"type\":\"gazebo.msgs.GUI\",\"station\":\""+station+"\",\"requested_world_pose\":"+Pose(receiver.target)+"}");
      receiver.armed=true;
    }
    ignition::math::Pose3d confirmed;
    {std::unique_lock<std::mutex> lock(receiver.mutex);if(!receiver.cv.wait_for(lock,std::chrono::duration<double>(timeout),[&](){return receiver.done;}))throw std::runtime_error("GUI not connected or actual camera pose readback did not match within bounded timeout");confirmed=receiver.actual;receiver.armed=false;}
    Print("{\"event\":\"view_confirmed\",\"passed\":true,\"station\":\""+station+"\",\"actual_world_pose\":"+Pose(confirmed)+",\"output_topic\":\"/gazebo/default/user_camera/pose\",\"published_count\":1}");
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;result=1;}
  // Receiver stays alive until both native worker groups have joined.
  try { if(initialized)transport::fini(); }
  catch(const std::exception &e){std::cerr<<e.what()<<std::endl;result=1;}
  try { if(ioOwner)ioOwner->Stop(); }
  catch(const std::exception &e){std::cerr<<e.what()<<std::endl;result=1;}
  if(node)node->Fini();sub.reset();pub.reset();node.reset();ioOwner.reset();
  return result;
}
