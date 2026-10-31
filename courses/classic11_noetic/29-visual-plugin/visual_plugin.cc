// A client-only VisualPlugin. Ogre/Camera APIs run only on render events.
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <boost/weak_ptr.hpp>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif
namespace gazebo {
class AppearancePlugin final : public VisualPlugin {
  struct Command { unsigned long long sequence=0;std::string action,color,path;double scale=1; };
  struct State {
    std::mutex mutex;bool active=true,applied=false,camera=false,material=false;
    std::string initialColor="red",color="red",configSHA,visual,client,cache,error,capture;
    double initialScale=1,scale=1;unsigned long long loads=0,inits=0,resets=0,renders=0,queued=0,appliedSeq=0,capturedSeq=0;
    std::deque<Command> commands;Command pendingCapture;
    bool captureArmed=false,previousCaptureData=false;
    unsigned long long imageEvents=0;unsigned int captureWidth=0,captureHeight=0;
    boost::weak_ptr<rendering::UserCamera> captureCamera;event::ConnectionPtr imageConnection;
    ignition::math::Color diffuse;ignition::math::Vector3d actualScale;ignition::math::Pose3d cameraPose;
    transport::PublisherPtr responses,publisher;
  };
  std::shared_ptr<State> state;transport::NodePtr node;transport::SubscriberPtr requests;
  event::ConnectionPtr before;
  static std::string Q(const std::string &text){std::ostringstream out;out<<'"';for(unsigned char c:text){if(c=='"'||c=='\\')out<<'\\';if(c=='\n')out<<"\\n";else if(c>=32)out<<c;}return out.str()+'"';}
  static ignition::math::Color Color(const std::string &name){
    if(name=="red")return {1,0,0,1};if(name=="blue")return {0,0,1,1};if(name=="green")return {0,1,0,1};
    throw std::runtime_error("Use red, blue or green");
  }
  static void Capture(State &s){
    std::ostringstream out;out<<std::setprecision(17)<<"{\"world\":\"lesson29\",\"visual_name\":"<<Q(s.visual)
      <<",\"client_id\":"<<Q(s.client)<<",\"source_sha256\":"<<Q(GZTEACH_SOURCE_SHA)<<",\"config_sha256\":"<<Q(s.configSHA)
      <<",\"load_count\":"<<s.loads<<",\"init_count\":"<<s.inits<<",\"reset_count\":"<<s.resets<<",\"render_count\":"<<s.renders
      <<",\"queued_command_seq\":"<<s.queued<<",\"applied_command_seq\":"<<s.appliedSeq<<",\"captured_command_seq\":"<<s.capturedSeq
      <<",\"appearance_applied\":"<<(s.applied?"true":"false")<<",\"configured_color\":"<<Q(s.initialColor)<<",\"configured_visual_scale\":"<<s.initialScale
      <<",\"color\":"<<Q(s.color)<<",\"visual_scale\":"<<s.scale
      <<",\"actual_diffuse_rgba\":["<<s.diffuse.R()<<','<<s.diffuse.G()<<','<<s.diffuse.B()<<','<<s.diffuse.A()<<']'
      <<",\"actual_scale_xyz\":["<<s.actualScale.X()<<','<<s.actualScale.Y()<<','<<s.actualScale.Z()<<']'
      <<",\"camera_initialized\":"<<(s.camera?"true":"false")<<",\"camera_pose\":["<<s.cameraPose.Pos().X()<<','<<s.cameraPose.Pos().Y()<<','<<s.cameraPose.Pos().Z()
      <<','<<s.cameraPose.Rot().Roll()<<','<<s.cameraPose.Rot().Pitch()<<','<<s.cameraPose.Rot().Yaw()<<']'
      <<",\"capture_path\":"<<Q(s.capture)<<",\"capture_image_events\":"<<s.imageEvents<<",\"capture_width\":"<<s.captureWidth
      <<",\"capture_height\":"<<s.captureHeight<<",\"capture_armed\":"<<(s.captureArmed?"true":"false")<<",\"last_error\":"<<Q(s.error)<<'}';s.cache=out.str();
  }
 public:
  ~AppearancePlugin()override{if(state){std::lock_guard<std::mutex> lock(state->mutex);state->active=false;auto camera=state->captureCamera.lock();if(camera&&state->captureArmed)camera->SetCaptureData(state->previousCaptureData);}before.reset();requests.reset();if(node)node->Fini();node.reset();state.reset();}
  void Load(rendering::VisualPtr parent,sdf::ElementPtr sdf)override{
    if(!parent)gzthrow("L29 needs an actual client Visual");state=std::make_shared<State>();auto shared=state;
    shared->initialColor=shared->color=sdf->Get<std::string>("color");Color(shared->color);
    shared->initialScale=shared->scale=sdf->Get<double>("visual_scale");
    if(!std::isfinite(shared->scale)||shared->scale<.7||shared->scale>1.6)gzthrow("Scale must be finite within .7..1.6");
    shared->configSHA=sdf->Get<std::string>("config_sha256");shared->visual=parent->Name();shared->loads=1;
    shared->client=std::to_string(getpid())+"-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());Capture(*shared);
    node.reset(new transport::Node());node->Init("lesson29");shared->responses=node->Advertise<msgs::Response>("~/response",100);shared->publisher=node->Advertise<msgs::Any>("~/visual_plugin/state",100);
    requests=node->Subscribe<msgs::Request>("~/request",[shared](const boost::shared_ptr<const msgs::Request> &request){
      if(request->request()!="gazeboteach_visual_snapshot"&&request->request()!="gazeboteach_visual_command")return;
      std::string copy,nonce=request->data();bool accepted=true;unsigned long long sequence=0;
      {std::lock_guard<std::mutex> lock(shared->mutex);if(!shared->active)return;
        if(request->request()=="gazeboteach_visual_command"){
          google::protobuf::Struct data;auto parsed=google::protobuf::util::JsonStringToMessage(request->data(),&data);
          try{
            if(!parsed.ok())throw std::runtime_error("Command is not JSON");const auto &f=data.fields();
            nonce=f.at("nonce").string_value();Command command;command.action=f.at("action").string_value();
            if(command.action=="color"){command.color=f.at("color").string_value();Color(command.color);}
            else if(command.action=="scale"){command.scale=f.at("scale").number_value();if(!std::isfinite(command.scale)||command.scale<.7||command.scale>1.6)throw std::runtime_error("Invalid scale");}
            else if(command.action=="capture"){command.path=f.at("path").string_value();if(command.path.empty()||command.path[0]!='/')throw std::runtime_error("Capture path must be absolute");}
            else if(command.action!="reset"&&command.action!="camera")throw std::runtime_error("Unknown render command");
            command.sequence=++shared->queued;sequence=command.sequence;shared->commands.push_back(command);
          }catch(const std::exception &){accepted=false;}
        }
        copy=shared->cache;
      }
      auto close=copy.rfind('}');if(close==std::string::npos)return;
      copy.insert(close,",\"query_nonce\":"+Q(nonce)+",\"accepted\":"+(accepted?"true":"false")+",\"command_seq\":"+std::to_string(sequence));
      msgs::Any payload;payload.set_type(msgs::Any::STRING);payload.set_string_value(copy);
      msgs::Response reply;reply.set_id(request->id());reply.set_request(request->request());reply.set_response(accepted?"success":"error");reply.set_type("gazebo.msgs.Any");reply.set_serialized_data(payload.SerializeAsString());shared->responses->Publish(reply);
    });
    std::weak_ptr<rendering::Visual> weak(parent);
    before=event::Events::ConnectPreRender([shared,weak]{
      auto visual=weak.lock();if(!visual)return;std::lock_guard<std::mutex> lock(shared->mutex);auto &s=*shared;if(!s.active)return;
      ++s.renders;bool appearance=!s.applied;
      while(!s.commands.empty()){
        auto command=s.commands.front();s.commands.pop_front();
        if(command.action=="color"){s.color=command.color;appearance=true;}
        else if(command.action=="scale"){s.scale=command.scale;appearance=true;}
        else if(command.action=="reset"){s.color=s.initialColor;s.scale=s.initialScale;appearance=true;}
        else if(command.action=="camera")s.camera=false;
        else if(command.action=="capture")s.pendingCapture=command;
        s.appliedSeq=command.sequence;
      }
      try{
        if(appearance){if(!s.material){visual->SetMaterial("Gazebo/White",true,false);s.material=true;}auto color=Color(s.color);visual->SetAmbient(color,false);visual->SetDiffuse(color,false);visual->SetEmissive(color,false);visual->SetLighting(true);visual->SetScale(ignition::math::Vector3d(s.scale,s.scale,s.scale));s.applied=true;}
        auto camera=gui::get_active_camera();
        if(camera&&!s.camera){camera->SetWorldPose(ignition::math::Pose3d(4,-6,3.5,0,std::atan2(2.7,std::sqrt(52.0)),std::atan2(6.0,-4.0)));s.camera=true;}
        if(camera)s.cameraPose=camera->WorldPose();
        if(camera&&s.pendingCapture.sequence&&!s.captureArmed){
          // SaveFrame alone does not capture pixels: the user camera normally
          // keeps no ImageData buffer. Arm a real new-image event, then enable
          // readback. Completion is proven by that event, not a render counter.
          s.previousCaptureData=camera->CaptureData();s.captureCamera=camera;
          std::weak_ptr<State> weakState(shared);
          s.imageConnection=camera->ConnectNewImageFrame([weakState](const unsigned char *buffer,unsigned int width,unsigned int height,unsigned int depth,const std::string &format){
            auto shared=weakState.lock();if(!shared)return;std::lock_guard<std::mutex> lock(shared->mutex);auto &s=*shared;
            if(!s.active||!s.captureArmed||!s.pendingCapture.sequence)return;
            ++s.imageEvents;s.captureWidth=width;s.captureHeight=height;
            try{
              if(!buffer||!width||!height||!depth)s.error="Actual new-image event has no pixel buffer";
              else if(rendering::Camera::SaveFrame(buffer,width,height,depth,format,s.pendingCapture.path)){s.capture=s.pendingCapture.path;s.capturedSeq=s.pendingCapture.sequence;}
              else s.error="Actual new-image pixel buffer could not be saved";
            }catch(const std::exception &error){s.error=error.what();}
            auto camera=s.captureCamera.lock();if(camera)camera->SetCaptureData(s.previousCaptureData);
            s.captureArmed=false;s.pendingCapture=Command();Capture(s);
          });
          s.captureArmed=true;camera->SetCaptureData(true);
        }
        s.diffuse=visual->Diffuse();s.actualScale=visual->Scale();
      }catch(const std::exception &error){s.error=error.what();}
      Capture(s);if(s.renders%6==0){msgs::Any message;message.set_type(msgs::Any::STRING);message.set_string_value(s.cache);s.publisher->Publish(message);}
    });
  }
  void Init()override{std::lock_guard<std::mutex> lock(state->mutex);++state->inits;Capture(*state);}
  void Reset()override{std::lock_guard<std::mutex> lock(state->mutex);++state->resets;Command command;command.action="reset";command.sequence=++state->queued;state->commands.push_back(command);Capture(*state);}
};
GZ_REGISTER_VISUAL_PLUGIN(AppearancePlugin)
}
