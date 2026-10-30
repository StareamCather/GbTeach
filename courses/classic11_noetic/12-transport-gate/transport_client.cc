// Bounded Classic SDK wire client. JSON is parsed with the already installed
// protobuf Struct utility; mutations are published once, never blindly retried.
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using Json=google::protobuf::Struct;
std::mutex outputMutex;
std::string Q(const std::string &v){std::ostringstream o;o<<'"';for(unsigned char c:v){if(c=='"'||c=='\\')o<<'\\';if(c=='\n')o<<"\\n";else if(c=='\r')o<<"\\r";else if(c=='\t')o<<"\\t";else if(c>=32)o<<c;}o<<'"';return o.str();}
void Print(const std::string &v){std::lock_guard<std::mutex> l(outputMutex);std::cout<<v<<std::endl;}
const google::protobuf::Value &Field(const Json &j,const std::string &key){auto it=j.fields().find(key);if(it==j.fields().end())throw std::runtime_error("Missing network JSON field "+key);return it->second;}
std::string Str(const Json &j,const std::string &key){auto &v=Field(j,key);if(v.kind_case()!=google::protobuf::Value::kStringValue)throw std::runtime_error("Non-string network field "+key);return v.string_value();}
double Num(const Json &j,const std::string &key){auto &v=Field(j,key);if(v.kind_case()!=google::protobuf::Value::kNumberValue||!std::isfinite(v.number_value()))throw std::runtime_error("Non-finite/non-numeric network field "+key);return v.number_value();}
long long Integer(const Json &j,const std::string &key){double n=Num(j,key);if(n!=std::floor(n)||std::abs(n)>9007199254740991.)throw std::runtime_error("Non-integer network correlation field "+key);return static_cast<long long>(n);}
Json Decode(const gazebo::msgs::Any &m){
  if(!m.IsInitialized()||m.type()!=gazebo::msgs::Any::STRING||!m.has_string_value()||m.has_double_value()||m.has_int_value()||m.has_bool_value()||m.has_vector3d_value()||m.has_color_value()||m.has_pose3d_value()||m.has_quaternion_value()||m.has_time_value())throw std::runtime_error("Network Any must contain exactly its STRING value");
  Json j;auto result=google::protobuf::util::JsonStringToMessage(m.string_value(),&j);if(!result.ok())throw std::runtime_error("Invalid network JSON in Any.STRING");return j;
}
struct Options {
  std::map<std::string,std::string> v;
  std::string Get(const std::string &k,const std::string &fallback="")const{auto i=v.find(k);return i==v.end()?fallback:i->second;}
  double Number(const std::string &k,double fallback)const{auto x=Get(k);return x.empty()?fallback:std::stod(x);}
};
struct Channels {
  std::mutex mutex;std::condition_variable cv;Options options;
  bool ackReady=false,stateReady=false,done=false,rejected=false,rpcDone=false,armed=false;
  int rpcID=0;std::string rpcName;gazebo::msgs::Response rpcResponse;
  void Any(const gazebo::msgs::Any &m,const std::string &kind){
    try{
      Json j=Decode(m);std::string run=Str(j,"run_id"),world=Str(j,"world"),stage=Str(j,"stage"),client=Str(j,"client_id"),nonce=Str(j,"nonce"),attempt=Str(j,"attempt_nonce");
      auto epoch=Integer(j,"epoch"),seq=Integer(j,"seq");
      auto &snapshotValue=Field(j,"snapshot");if(snapshotValue.kind_case()!=google::protobuf::Value::kStructValue)throw std::runtime_error("Missing actual network snapshot");
      const auto &snapshot=snapshotValue.struct_value();for(const char *key:{"sim_time_s","q_m","qd_m_s","force_N","pid_command_N"})Num(snapshot,key);Integer(snapshot,"iteration");
      bool context=run==options.Get("expected-run")&&world==options.Get("world","default")&&epoch==static_cast<long long>(options.Number("expected-epoch",0));
      bool identity=context&&client==options.Get("client")&&seq==static_cast<long long>(options.Number("seq",0))&&nonce==options.Get("nonce")&&attempt==options.Get("attempt");
      bool targetMatches=true;
      if(identity&&(stage=="received"||stage=="applied"||stage=="completed")){
        double wanted=options.Number("target",NAN);targetMatches=std::isfinite(wanted)&&std::abs(Num(j,"target_requested_m")-wanted)<1e-12;
        if(stage!="received")targetMatches=targetMatches&&std::abs(Num(j,"target_applied_m")-wanted)<1e-12;
      }
      auto &counterValue=Field(j,"counters");if(counterValue.kind_case()!=google::protobuf::Value::kStructValue)throw std::runtime_error("Missing actual ACK counters");
      bool causal=options.Get("mode")!="send"||(stage=="rejected"?Integer(counterValue.struct_value(),"rejected")>options.Number("baseline-rejected",0):Integer(counterValue.struct_value(),"received")>options.Number("baseline-received",0));
      std::unique_lock<std::mutex> lock(mutex);
      bool matched=identity&&causal&&targetMatches&&(armed||options.Get("mode")!="send");
      Print("{\"event\":"+Q(kind)+",\"matched\":"+(matched?"true":"false")+",\"target_matches\":"+(targetMatches?"true":"false")+",\"raw_payload_json\":"+Q(m.string_value())+",\"payload\":"+m.string_value()+"}");
      if(context){if(kind=="ack")ackReady=true;else stateReady=true;}
      if(kind=="ack"&&matched){
        if(stage=="rejected"||stage=="failed"||stage=="cancelled"){rejected=true;done=true;}
        else if(stage==options.Get("wait","completed"))done=true;
      }
      cv.notify_all();
    }catch(const std::exception &e){Print("{\"event\":\"invalid_network_message\",\"reason\":"+Q(e.what())+"}");}
  }
  void Ack(ConstAnyPtr &m){Any(*m,"ack");}
  void State(ConstAnyPtr &m){Any(*m,"state");}
  void Response(ConstResponsePtr &m){
    std::lock_guard<std::mutex> lock(mutex);
    if(m->id()==rpcID&&m->request()==rpcName){rpcResponse.CopyFrom(*m);rpcDone=true;cv.notify_all();}
  }
};
}

int main(int argc,char **argv){
  bool initialized=false;int code=0;
  try{
    if(argc<2)throw std::runtime_error("Expected send/wait/watch/query/entity mode");
    std::string mode=argv[1];Options options;
    for(int i=2;i<argc;++i){std::string k=argv[i];if(k.substr(0,2)!="--")throw std::runtime_error("Expected option");k=k.substr(2);if(k=="omit-target")options.v[k]="1";else{if(++i>=argc)throw std::runtime_error("Missing option value");options.v[k]=argv[i];}}
    if(mode=="query"&&options.Get("request","gazeboteach_transport_snapshot")!="gazeboteach_transport_snapshot"&&options.Get("request")!="gazeboteach_transport_missing")throw std::runtime_error("Query request is outside the read-only allowlist; rejected before transport initialization");
    double timeout=options.Number("timeout",6);if(!std::isfinite(timeout)||timeout<=0||timeout>12)throw std::runtime_error("Client timeout must be finite within (0,12] seconds");
    if(mode=="send"&&options.Get("observed-type")!=options.Get("expect-type","gazebo.msgs.Request"))throw std::runtime_error("wire type mismatch before publish");
    if(mode=="send"&&options.Get("observed-type")!="gazebo.msgs.Request")throw std::runtime_error("SDK sender only permits gazebo.msgs.Request");
    if(!gazebo::transport::init("",0,3))throw std::runtime_error("Bounded Classic master setup failed");initialized=true;gazebo::transport::run();
    Channels channels;channels.options=options;channels.options.v["mode"]=mode;
    gazebo::transport::NodePtr node(new gazebo::transport::Node());node->Init(options.Get("world","default"));
    if(mode=="query"||mode=="entity"){
      channels.rpcName=mode=="entity"?"entity_info":options.Get("request","gazeboteach_transport_snapshot");
      channels.rpcID=static_cast<int>((std::chrono::steady_clock::now().time_since_epoch().count()&0x3fffffff)+1);
      auto sub=node->Subscribe("~/response",&Channels::Response,&channels,true);
      auto pub=node->Advertise<gazebo::msgs::Request>("~/request");
      if(!pub->WaitForConnection(gazebo::common::Time(1,0)))throw std::runtime_error("No native RPC request subscriber");
      gazebo::msgs::Request request;request.set_id(channels.rpcID);request.set_request(channels.rpcName);request.set_data(mode=="entity"?"transport_gate":options.Get("nonce"));pub->Publish(request);
      {std::unique_lock<std::mutex> lock(channels.mutex);if(!channels.cv.wait_for(lock,std::chrono::duration<double>(timeout),[&](){return channels.rpcDone;}))throw std::runtime_error("Bounded native RPC timeout: no matching Response.id/request");}
      gazebo::msgs::Response response;{std::lock_guard<std::mutex> lock(channels.mutex);response.CopyFrom(channels.rpcResponse);}
      if(!response.IsInitialized()||response.id()!=channels.rpcID||response.request()!=channels.rpcName||response.response()!="success")throw std::runtime_error("Native RPC response metadata failed correlation/success check");
      std::string payload;
      if(mode=="entity"){
        if(response.type()!="gazebo.msgs.Model")throw std::runtime_error("entity_info response type is not gazebo.msgs.Model");
        gazebo::msgs::Model m;if(!m.ParseFromString(response.serialized_data())||!m.IsInitialized())throw std::runtime_error("Invalid native Model response bytes");
        google::protobuf::util::JsonPrintOptions print;print.preserve_proto_field_names=true;
        auto result=google::protobuf::util::MessageToJsonString(m,&payload,print);if(!result.ok())throw std::runtime_error("Unable to decode native Model JSON");
      }else{
        if(response.type()!="gazebo.msgs.Any")throw std::runtime_error("Snapshot response type is not gazebo.msgs.Any");
        gazebo::msgs::Any m;if(!m.ParseFromString(response.serialized_data()))throw std::runtime_error("Invalid snapshot response bytes");Json j=Decode(m);
        if(Str(j,"query_nonce")!=options.Get("nonce")||Str(j,"run_id")!=options.Get("expected-run")||Str(j,"world")!=options.Get("world","default"))throw std::runtime_error("Snapshot query nonce/boot/world mismatch");Integer(j,"epoch");payload=m.string_value();
      }
      Print("{\"event\":\"rpc_response\",\"request_id\":"+std::to_string(channels.rpcID)+",\"response_id\":"+std::to_string(response.id())+",\"request\":"+Q(response.request())+",\"response\":"+Q(response.response())+",\"response_type\":"+Q(response.type())+",\"serialized_data_size\":"+std::to_string(response.serialized_data().size())+",\"response_debug\":"+Q(response.DebugString())+",\"payload\":"+payload+"}");
    }else{
      auto ackSub=node->Subscribe("~/transport_lab/ack",&Channels::Ack,&channels,true);
      auto stateSub=node->Subscribe("~/transport_lab/state",&Channels::State,&channels,true);
      if(mode=="send"){
        // Seeing an actual current-boot cached ACK confirms the reverse path.
        // It is recorded as unmatched, never accepted for this fresh nonce.
        {std::unique_lock<std::mutex> lock(channels.mutex);if(!channels.cv.wait_for(lock,std::chrono::seconds(2),[&](){return channels.ackReady;}))throw std::runtime_error("No actual current-boot ACK subscription handshake");}
        auto pub=node->Advertise<gazebo::msgs::Request>(options.Get("topic","~/transport_lab/command"));
        if(!pub->WaitForConnection(gazebo::common::Time(1,0)))throw std::runtime_error("No command subscriber within bounded connection wait");
        gazebo::msgs::Request m;m.set_id(std::stoi(options.Get("seq")));m.set_request(options.Get("verb","set_target"));
        m.set_data(options.Get("run")+" "+options.Get("client")+" "+options.Get("nonce")+" "+options.Get("epoch")+" "+options.Get("attempt"));
        if(options.Get("omit-target")!="1")m.set_dbl_data(std::stod(options.Get("target")));
        {std::lock_guard<std::mutex> lock(channels.mutex);channels.armed=true;}
        pub->Publish(m);Print("{\"event\":\"published_once\",\"application_success\":false,\"request_debug\":"+Q(m.DebugString())+"}");
        std::unique_lock<std::mutex> lock(channels.mutex);if(!channels.cv.wait_for(lock,std::chrono::duration<double>(timeout),[&](){return channels.done;}))throw std::runtime_error("No matching application ACK within bounded wait");
        if(channels.rejected)code=3;
      }else if(mode=="wait"){
        std::unique_lock<std::mutex> lock(channels.mutex);if(!channels.cv.wait_for(lock,std::chrono::duration<double>(timeout),[&](){return channels.done;}))throw std::runtime_error("Continuation wait timed out without resending");if(channels.rejected)code=3;
      }else if(mode=="watch"){
        std::unique_lock<std::mutex> lock(channels.mutex);channels.cv.wait_for(lock,std::chrono::duration<double>(timeout),[](){return false;});if(!channels.stateReady)throw std::runtime_error("No valid current-boot network state during bounded watch");
      }else throw std::runtime_error("Unknown SDK client mode");
    }
    node->Fini();node.reset();
  }catch(const std::exception &e){Print("{\"event\":\"client_error\",\"transport_initialized\":"+std::string(initialized?"true":"false")+",\"reason\":"+Q(e.what())+"}");code=2;}
  if(initialized)gazebo::transport::fini();return code;
}
