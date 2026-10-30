// Gazebo owns the plant. Python supplies only a scalar cart force every 10 ms.
#include <Python.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <cerrno>

namespace gazebo {
namespace {
struct GIL { PyGILState_STATE state; GIL():state(PyGILState_Ensure()) {} ~GIL(){PyGILState_Release(state);} };
std::mutex interpreterMutex;
void Interpreter() {
  std::lock_guard<std::mutex> lock(interpreterMutex);
  if (!Py_IsInitialized()) {
    Py_InitializeEx(0);
    PyObject *numpy=PyImport_ImportModule("numpy");
    if (!numpy) { PyErr_Print(); PyEval_SaveThread(); throw std::runtime_error("Embedded Python cannot import NumPy"); }
    Py_DECREF(numpy);
    PyEval_SaveThread();  // The update thread will explicitly acquire the GIL.
  }
}
std::string Env(const char *key) { const char *value=std::getenv(key); if(!value) throw std::runtime_error(std::string("Missing ")+key); return value; }
std::string Quote(const std::string &text) {
  std::ostringstream out; out << '"';
  for(unsigned char c:text) { if(c=='"'||c=='\\') out << '\\' << c; else if(c=='\n') out << "\\n"; else if(c=='\r') out << "\\r"; else if(c=='\t') out << "\\t"; else if(c<32) out << '?'; else out << c; }
  out << '"'; return out.str();
}
std::string PythonError() {
  PyObject *type=nullptr,*value=nullptr,*trace=nullptr;
  PyErr_Fetch(&type,&value,&trace); PyErr_NormalizeException(&type,&value,&trace);
  std::string result="Python callback error";
  PyObject *traceback=PyImport_ImportModule("traceback");
  PyObject *lines=traceback?PyObject_CallMethod(traceback,"format_exception","OOO",type?type:Py_None,value?value:Py_None,trace?trace:Py_None):nullptr;
  PyObject *separator=PyUnicode_FromString("");
  PyObject *joined=lines?PyUnicode_Join(separator,lines):nullptr;
  if(joined && PyUnicode_AsUTF8(joined)) result=PyUnicode_AsUTF8(joined);
  else if(value) { PyObject *str=PyObject_Str(value); if(str && PyUnicode_AsUTF8(str)) result=PyUnicode_AsUTF8(str); Py_XDECREF(str); }
  Py_XDECREF(joined);Py_XDECREF(separator);Py_XDECREF(lines);Py_XDECREF(traceback);
  Py_XDECREF(type);Py_XDECREF(value);Py_XDECREF(trace);PyErr_Clear();return result;
}
struct ControllerState {
  std::mutex mutex;
  std::atomic<bool> closing{false};
  physics::ModelPtr model; physics::WorldPtr world; physics::JointPtr cart,pole;
  PyObject *callback=nullptr,*module=nullptr;
  std::string directory,mode,sha,source,worldSha,numpy,error,loadError,episode="startup",lastRequest;
  std::ofstream csv;
  double start=0,lastTime=-1,nextControl=0,lastStatus=-1,force=0,pythonForce=0,duration=12;
  double disturbance=0;
  unsigned long long calls=0,episodeCalls=0,demoNumber=0,physicsTicks=0,resets=0;
  bool manual=false,initialized=false;
  ~ControllerState() {
    if(Py_IsInitialized()) { GIL gil;
      if(module) { PyObject *name=PyObject_GetAttrString(module,"__name__");
        if(name && PyDict_DelItem(PyImport_GetModuleDict(),name)<0) PyErr_Clear(); Py_XDECREF(name); }
      Py_XDECREF(callback);Py_XDECREF(module);
    }
  }
  void Status(double now) {
    std::ofstream out(directory+"/status.json.tmp");
    auto inertia=model->GetLink("pole")->GetInertial();
    out << std::setprecision(17) << "{\"schema\":1,\"mode\":" << Quote(mode) << ",\"source_sha256\":" << Quote(sha)
        << ",\"source_path\":" << Quote(source) << ",\"world_sha256\":" << Quote(worldSha) << ",\"numpy_version\":" << Quote(numpy)
        << ",\"error\":" << Quote(error) << ",\"episode\":" << Quote(episode)
        << ",\"callback_count\":" << calls << ",\"episode_callback_count\":" << episodeCalls
        << ",\"physics_tick_count\":" << physicsTicks << ",\"reset_count\":" << resets << ",\"sim_time_s\":" << now
        << ",\"episode_time_s\":" << now-start << ",\"manual_episode\":" << (manual?"true":"false")
        << ",\"initialization\":\"Joint.SetPosition/SetVelocity once per episode; force only thereafter\""
        << ",\"model\":{\"cart_mass_kg\":" << model->GetLink("cart")->GetInertial()->Mass()
        << ",\"pole_mass_kg\":" << inertia->Mass() << ",\"pole_Iyy_kg_m2\":" << inertia->IYY()
        << ",\"cart_damping\":" << cart->GetDamping(0) << ",\"pole_damping\":" << pole->GetDamping(0)
        << ",\"physics_step_s\":" << world->Physics()->GetMaxStepSize() << ",\"gravity_z\":" << world->Gravity().Z() << "}}\n";
    out.close(); std::rename((directory+"/status.json.tmp").c_str(),(directory+"/status.json").c_str()); lastStatus=now;
  }
  void Begin(const std::string &id,double x,double xd,double q,double qd,double seconds,double external,double now,bool requested) {
    // This is the ONLY trajectory initialization path, once at the episode boundary.
    cart->SetForce(0,0); pole->SetForce(0,0);
    cart->SetPosition(0,x,true); pole->SetPosition(0,q,true);
    cart->SetVelocity(0,xd); pole->SetVelocity(0,qd);
    force=0;pythonForce=0;episodeCalls=0;error=loadError;episode=id;start=now;nextControl=now;
    duration=seconds;disturbance=external;manual=requested;initialized=true;Status(now);
  }
  void Request(double now) {
    std::ifstream in(directory+"/episode.request");
    std::string id;double x,xd,q,qd,seconds,external;
    if(!(in>>id>>x>>xd>>q>>qd>>seconds>>external)||id==lastRequest) return;
    lastRequest=id;
    if(id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos ||
       !std::isfinite(x)||!std::isfinite(xd)||!std::isfinite(q)||!std::isfinite(qd)||!std::isfinite(seconds)||!std::isfinite(external)||
       std::abs(x)>.3||std::abs(xd)>1||std::abs(q)>.25||std::abs(qd)>1||seconds<1||seconds>20||std::abs(external)>3) {
      error="Rejected invalid episode request";force=0;Status(now);return;
    }
    Begin(id,x,xd,q,qd,seconds,external,now,true);
  }
  void ResetController() {
    std::lock_guard<std::mutex> lock(mutex);
    cart->SetForce(0,0);pole->SetForce(0,0);
    force=0;pythonForce=0;calls=0;episodeCalls=0;physicsTicks=0;demoNumber=0;
    lastTime=-1;nextControl=0;lastStatus=-1;initialized=false;manual=false;error=loadError;
    ++resets;episode="reset-pending-"+std::to_string(resets);start=world->SimTime().Double();Status(start);
  }
  void Update(const common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex);
    if(closing) return;
    double now=info.simTime.Double(); ++physicsTicks;
    if(!initialized || now<lastTime) Begin("startup-"+std::to_string(resets),.12,0,.08,0,12,2,now,false);
    lastTime=now;
    // File commands and controller callbacks are bounded to the 100 Hz control boundary.
    if(now+1e-9>=nextControl) {
      Request(now);
      if(mode=="reference_demo" && !manual && now-start>=duration) {
        ++demoNumber; double sign=(demoNumber%2)?-1:1;
        Begin("demo-"+std::to_string(demoNumber),sign*.12,0,sign*.08,0,12,2,now,false);
      }
      double x=cart->Position(0),xd=cart->GetVelocity(0),q=pole->Position(0),qd=pole->GetVelocity(0);
      double angle=std::remainder(q,2*M_PI);
      if(std::abs(x)>1.49||std::abs(angle)>.65) error="Local stabilization boundary exceeded (track/angle); force set to zero";
      pythonForce=0;force=0;
      if(error.empty() && callback) {
        GIL gil;
        PyObject *observation=Py_BuildValue("{s:d,s:d,s:d,s:d}","x",x,"xdot",xd,"q1",q,"q1dot",qd);
        ++calls;++episodeCalls;
        PyObject *answer=PyObject_CallFunctionObjArgs(callback,observation,nullptr);Py_DECREF(observation);
        if(!answer) error=PythonError();
        else {
          pythonForce=PyFloat_AsDouble(answer);Py_DECREF(answer);
          if(PyErr_Occurred()) error=PythonError();
          else if(!std::isfinite(pythonForce)) error="Python control returned non-finite force";
          else force=std::max(-15.0,std::min(15.0,pythonForce));
        }
        if(!error.empty()) {force=0;pythonForce=0;gzerr << "[GazeboTeach Python control] " << error << "\n";}
      }
      double elapsed=now-start;
      double external=(error.empty() && elapsed>=4 && elapsed<4.25)?disturbance:0;
      auto direction=model->GetLink("pole")->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitZ);
      csv << std::setprecision(17) << episode << ',' << now << ',' << elapsed << ',' << x << ',' << xd << ',' << q << ',' << qd
          << ',' << angle << ',' << direction.X() << ',' << direction.Z() << ',' << pythonForce << ',' << force << ',' << external
          << ',' << force+external << ',' << mode << ',' << sha << ',' << calls << ',' << episodeCalls << ',' << (error.empty()?1:0) << '\n';
      csv.flush();
      nextControl=now+.01;
      if(!error.empty() || now-lastStatus>=.1) Status(now);
    }
    double elapsed=now-start;
    double external=(error.empty() && elapsed>=4 && elapsed<4.25)?disturbance:0;
    cart->SetForce(0,error.empty()?force+external:0);  // ZOH force refreshed every physics step.
  }
};
}

class PythonControl : public ModelPlugin {
  std::shared_ptr<ControllerState> state;
  event::ConnectionPtr connection;
 public:
  void Reset() override { if(state) state->ResetController(); }
  ~PythonControl() override {
    if(state) state->closing=true;
    connection.reset();
    // No GIL while waiting for an in-flight callback to finish.
    if(state) { std::lock_guard<std::mutex> lock(state->mutex); }
    state.reset();
  }
  void Load(physics::ModelPtr model,sdf::ElementPtr) override {
    state=std::make_shared<ControllerState>();auto s=state;
    s->model=model;s->world=model->GetWorld();s->cart=model->GetJoint("cart_slide");s->pole=model->GetJoint("pole_hinge");
    if(!s->cart||!s->pole) throw std::runtime_error("Missing cart_slide/pole_hinge actual joints");
    s->directory=Env("GAZEBOTEACH_SESSION_DIR")+"/control";
    if(::mkdir(s->directory.c_str(),0700)!=0 && errno!=EEXIST) throw std::runtime_error("Cannot create scoped telemetry directory");
    s->csv.open(s->directory+"/control.csv");
    if(!s->csv) throw std::runtime_error("Cannot create actual-state CSV");
    s->csv << "episode,sim_time_s,episode_time_s,x_m,xdot_m_s,q1_rad,q1dot_rad_s,theta_up_rad,pole_axis_x,pole_axis_z,python_force_N,command_force_N,disturbance_N,applied_force_N,mode,source_sha256,callback_count,episode_callback_count,controller_ok\n";
    try {
      Interpreter();GIL gil;
      auto lesson=Env("GAZEBOTEACH_LESSON_SOURCE");auto slash=lesson.find_last_of('/');
      std::string loader=lesson.substr(0,slash)+"/_control_support/runtime_loader.py";
      std::ifstream input(loader);std::stringstream text;text<<input.rdbuf();
      if(!input) throw std::runtime_error("Cannot read trusted runtime loader");
      PyObject *globals=PyDict_New();PyDict_SetItemString(globals,"__builtins__",PyEval_GetBuiltins());
      PyObject *loaded=PyRun_String(text.str().c_str(),Py_file_input,globals,globals);
      if(!loaded) {auto error=PythonError();Py_DECREF(globals);throw std::runtime_error(error);}Py_DECREF(loaded);
      PyObject *function=PyDict_GetItemString(globals,"load_controller");
      PyObject *metadata=PyObject_CallObject(function,nullptr);
      if(!metadata) {auto error=PythonError();Py_DECREF(globals);throw std::runtime_error(error);}
      s->callback=PyTuple_GetItem(metadata,0);Py_INCREF(s->callback);
      s->mode=PyUnicode_AsUTF8(PyTuple_GetItem(metadata,1));s->sha=PyUnicode_AsUTF8(PyTuple_GetItem(metadata,2));
      s->source=PyUnicode_AsUTF8(PyTuple_GetItem(metadata,3));s->module=PyTuple_GetItem(metadata,4);Py_INCREF(s->module);
      s->numpy=PyUnicode_AsUTF8(PyTuple_GetItem(metadata,5));s->worldSha=PyUnicode_AsUTF8(PyTuple_GetItem(metadata,6));Py_DECREF(metadata);Py_DECREF(globals);
      gzmsg << "[GazeboTeach] actual-state Python control mode=" << s->mode << " source_sha256=" << s->sha << " numpy=" << s->numpy << "\n";
    } catch(const std::exception &error) {s->loadError=s->error=error.what();gzerr << "[GazeboTeach bridge load failure] " << s->error << "\n";}
    s->Status(s->world->SimTime().Double());
    connection=event::Events::ConnectWorldUpdateBegin([s](const common::UpdateInfo &info){s->Update(info);});
  }
};
GZ_REGISTER_MODEL_PLUGIN(PythonControl)
}
