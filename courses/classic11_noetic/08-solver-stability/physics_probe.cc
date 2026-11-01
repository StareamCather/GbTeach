// Read-only native Classic transport request; no server plugin or control input.
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
#include <iostream>
#include <stdexcept>

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

int main(int argc, char **argv)
{
  if (argc != 2) { std::cerr << "Expected world name\n"; return 2; }
  int status = 1; bool initialized = false;
  std::unique_ptr<MainThreadIOOwner> ioOwner;
  try
  {
    ioOwner.reset(new MainThreadIOOwner());initialized=true;
    if (!transport::init("",0,3)) throw std::runtime_error("Classic master setup failed");
    transport::run();
    // TransportIface takes seconds/nanoseconds, not an integer millisecond count.
    auto reply = gazebo::transport::request(argv[1], "physics_info", "",
        gazebo::common::Time(1, 500000000));
    gazebo::msgs::Physics physics;
    if (!reply || reply->response() != "success" ||
        reply->request() != "physics_info" || reply->type() != physics.GetTypeName() ||
        !physics.ParseFromString(reply->serialized_data()))
      throw std::runtime_error("Missing or invalid physics_info response");
    std::cout << physics.DebugString() << std::flush;
    status = 0;
  }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; }
  try { if(initialized) transport::fini(); }
  catch(const std::exception &error) {std::cerr<<error.what()<<std::endl;status=1;}
  try { if(ioOwner) ioOwner->Stop(); }
  catch(const std::exception &error) {std::cerr<<error.what()<<std::endl;status=1;}
  ioOwner.reset();
  return status;
}
