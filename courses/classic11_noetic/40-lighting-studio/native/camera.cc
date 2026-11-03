// Capture actual server-side rendered camera frames; never synthesize pixels.
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/common/Events.hh>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
namespace gazebo {
class SceneFrame final : public SensorPlugin {
 sensors::CameraSensorPtr sensor; event::ConnectionPtr connection; std::string dir; unsigned long long sequence=0;
 public:
 ~SceneFrame() override {connection.reset();sensor.reset();}
 void Load(sensors::SensorPtr parent,sdf::ElementPtr) override {
  sensor=std::dynamic_pointer_cast<sensors::CameraSensor>(parent);
  if(!sensor)gzthrow("SceneFrame requires actual CameraSensor");
  const char *value=std::getenv("GAZEBOTEACH_SESSION_DIR");if(!value)gzthrow("Missing owned session directory");dir=value;
  connection=sensor->Camera()->ConnectNewImageFrame(std::bind(&SceneFrame::Frame,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
  sensor->SetActive(true);
 }
 void Frame(const unsigned char *image,unsigned int w,unsigned int h,unsigned int depth,const std::string &format) {
  ++sequence;if(sequence%5||!image||depth!=3||format!="R8G8B8")return;
  const size_t count=static_cast<size_t>(w)*h*depth;uint64_t sum=14695981039346656037ULL;
  for(size_t i=0;i<count;++i){sum^=image[i];sum*=1099511628211ULL;}
  std::string filename=dir+"/camera.ppm";
  {std::ofstream out(filename+".tmp",std::ios::binary);out<<"P6\n"<<w<<" "<<h<<"\n255\n";out.write(reinterpret_cast<const char*>(image),count);if(!out) return;}
  std::rename((filename+".tmp").c_str(),filename.c_str());
  std::string report=dir+"/camera.json";
  {std::ofstream out(report+".tmp");out<<std::setprecision(17)<<"{\"sequence\":"<<sequence<<",\"sim_time_s\":"<<sensor->LastMeasurementTime().Double()<<",\"width\":"<<w<<",\"height\":"<<h<<",\"format\":\"R8G8B8\",\"fnv64\":\""<<sum<<"\"}";if(!out)return;}
  std::rename((report+".tmp").c_str(),report.c_str());
 }
};
GZ_REGISTER_SENSOR_PLUGIN(SceneFrame)
}
