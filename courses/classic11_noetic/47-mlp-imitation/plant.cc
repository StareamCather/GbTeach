// A47: owned paused-world transactions. Only the socket worker calls Step(N).
// Physics callbacks apply commands and cache the actual completed ODE state.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#ifndef GT_PLANT_SOURCE_SHA
#define GT_PLANT_SOURCE_SHA ""
#endif

namespace gazebo {
class LearningSlider : public ModelPlugin {
  using Clock = std::chrono::steady_clock;
  struct Snapshot {
    uint64_t iterations=0, callbacks=0;
    double time=0, x=0, v=0;
    bool paused=true;
  };
 public:
  ~LearningSlider() override {
    stopping.store(true); force.store(0);
    const int fd=client.load();
    if (fd>=0) shutdown(fd,SHUT_RDWR);
    // World::Step(0) only cancels its counter, without a physics iteration.
    // Model deletion can run on the physics thread: cancel repeatedly until
    // the worker leaves Step(N), rather than join a worker waiting on us.
    // Repetition covers the race just before the worker enters Step(N).
    if (worker.joinable()) {
      while (!workerDone.load()) {
        if (inStep.load() && world) { try { world->Step(0); } catch (...) { } }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      worker.join();
    }
    beginConnection.reset(); endConnection.reset();
    if (listener>=0) close(listener);
    RemoveOwnedEndpoint();
  }
  void Load(physics::ModelPtr parent,sdf::ElementPtr) override {
    model=parent; world=model->GetWorld(); joint=model->GetJoint("slide");
    const char *value=getenv("GAZEBOTEACH_SESSION_TOKEN");
    const std::string token=value?value:"", source=GT_PLANT_SOURCE_SHA;
    auto hex=[](unsigned char c) { return (c>='0' && c<='9') || (c>='a' && c<='f'); };
    if (!joint || token.size()!=32 || !std::all_of(token.begin(),token.end(),hex) ||
        source.size()!=64 || !std::all_of(source.begin(),source.end(),hex))
      gzthrow("A47 needs an owned session, slider joint and compiled source SHA256");
    dt=world->Physics()->GetMaxStepSize();
    if (!std::isfinite(dt) || dt<=0 || dt>.01) gzthrow("A47 physics step size is outside its bounded range");
    path="/tmp/gt-learning-"+token+".sock";
    listener=socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    if (path.size()>=sizeof(address.sun_path)) gzthrow("A47 endpoint path is too long");
    std::copy(path.begin(),path.end(),address.sun_path);
    if (listener<0 || bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0)
      gzthrow("Cannot bind unique A47 endpoint; existing endpoints are never removed");
    struct stat identity{};
    if (lstat(path.c_str(),&identity)!=0 || !S_ISSOCK(identity.st_mode) || identity.st_uid!=getuid())
      gzthrow("A47 endpoint ownership could not be established");
    endpointDevice=identity.st_dev; endpointInode=identity.st_ino; ownsEndpoint=true;
    if (chmod(path.c_str(),0600)!=0 || listen(listener,2)!=0) gzthrow("Cannot secure/listen on A47 endpoint");
    cached=ReadPhysics();
    beginConnection=event::Events::ConnectWorldUpdateBegin(std::bind(&LearningSlider::OnBegin,this));
    endConnection=event::Events::ConnectWorldUpdateEnd(std::bind(&LearningSlider::OnEnd,this));
    workerDone.store(false);
    worker=std::thread(&LearningSlider::Serve,this);
  }
 private:
  static std::string Quoted(const std::string &value) {
    std::ostringstream out; out << '"';
    for (unsigned char c:value) {
      if (c=='"' || c=='\\') out << '\\' << c;
      else if (c=='\n') out << "\\n";
      else if (c=='\r') out << "\\r";
      else if (c=='\t') out << "\\t";
      else if (c<32) out << '?';
      else out << c;
    }
    out << '"'; return out.str();
  }
  static void Number(std::ostream &out,double value) {
    if (std::isfinite(value)) out << std::setprecision(17) << value;
    else out << "null";
  }
  static void WriteSnapshot(std::ostream &out,const Snapshot &s) {
    out << "{\"iterations\":" << s.iterations << ",\"sim_time_s\":"; Number(out,s.time);
    out << ",\"x_m\":"; Number(out,s.x); out << ",\"v_m_s\":"; Number(out,s.v);
    out << ",\"paused\":" << (s.paused?"true":"false") << ",\"callbacks\":" << s.callbacks << '}';
  }
  Snapshot ReadPhysics() const {
    Snapshot s; s.iterations=world->Iterations(); s.callbacks=callbacks.load();
    s.time=world->SimTime().Double(); s.x=joint->Position(0); s.v=joint->GetVelocity(0);
    s.paused=world->IsPaused(); return s;
  }
  Snapshot ReadCache() {
    std::lock_guard<std::mutex> lock(cacheMutex);
    Snapshot result=cached; result.paused=world->IsPaused(); return result;
  }
  bool Finite(const Snapshot &s) const {
    return std::isfinite(s.time) && std::isfinite(s.x) && std::isfinite(s.v);
  }
  void Poison(const std::string &why) {
    force.store(0); poisoned.store(true);
    { std::lock_guard<std::mutex> lock(errorMutex); if (error.empty()) error=why; }
    if (!stopping.load() && world && world->Running() && !world->IsPaused()) world->SetPaused(true);
  }
  std::string Error() {
    std::lock_guard<std::mutex> lock(errorMutex);
    return error.empty()?"A47 transaction is poisoned; restart this experiment":error;
  }
  void OnBegin() {
    callbacks.fetch_add(1);
    if (stopping.load()) { joint->SetForce(0,0); return; }
    if (leased.load()) {
      if (!world->IsPaused()) Poison("External resume violated the paused-world learning lease");
      if (!inStep.load()) Poison("An external physics step occurred outside an A47 transaction");
    }
    if (resetPending.exchange(false) && !poisoned.load()) {
      joint->SetPosition(0,resetX.load(),true); joint->SetVelocity(0,resetV.load());
    }
    joint->SetForce(0,leased.load() && !poisoned.load()?force.load():0);
  }
  void OnEnd() {
    const Snapshot state=ReadPhysics();
    std::lock_guard<std::mutex> lock(cacheMutex); cached=state;
  }
  bool Stable(Snapshot &state) {
    if (!leased.load() || poisoned.load()) return false;
    if (!world->IsPaused()) Poison("World was resumed outside the A47 lease");
    state=ReadCache();
    if (!world->Running() || !Finite(state) || state.iterations!=world->Iterations() ||
        state.callbacks!=callbacks.load() || state.iterations!=expected.iterations ||
        state.time!=expected.time || state.x!=expected.x || state.v!=expected.v)
      Poison("Physical state changed between A47 transactions; no action was replayed");
    return !poisoned.load();
  }
  bool EndpointOwned() const {
    struct stat s{};
    return ownsEndpoint && lstat(path.c_str(),&s)==0 && S_ISSOCK(s.st_mode) &&
      s.st_uid==getuid() && s.st_dev==endpointDevice && s.st_ino==endpointInode;
  }
  void RemoveOwnedEndpoint() {
    if (EndpointOwned()) unlink(path.c_str());
    ownsEndpoint=false;
  }
  bool PeerAllowed(int fd) const {
    struct ucred peer{}; socklen_t size=sizeof(peer);
    return getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&peer,&size)==0 && size==sizeof(peer) && peer.uid==getuid();
  }
  void RejectQueuedClients() {
    for (;;) {
      int extra=accept4(listener,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
      if (extra<0) return;
      const std::string busy="{\"protocol\":2,\"id\":0,\"kind\":\"busy\",\"ok\":false,\"error\":\"Another learning client owns this endpoint\"}\n";
      if (PeerAllowed(extra)) send(extra,busy.data(),busy.size(),MSG_NOSIGNAL|MSG_DONTWAIT);
      close(extra);
    }
  }
  bool Send(int fd,const std::string &text) {
    size_t sent=0; const auto deadline=Clock::now()+std::chrono::seconds(2);
    while (sent<text.size() && !stopping.load() && Clock::now()<deadline) {
      ssize_t n=send(fd,text.data()+sent,text.size()-sent,MSG_NOSIGNAL|MSG_DONTWAIT);
      if (n>0) { sent+=static_cast<size_t>(n); continue; }
      if (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) return false;
      pollfd p{fd,POLLOUT,0}; poll(&p,1,50);
    }
    return sent==text.size();
  }
  bool Reply(int fd,uint64_t id,const std::string &kind,bool ok,const Snapshot &before,
             const Snapshot &after,double action,unsigned count,bool mutation,const std::string &message="",
             bool restore=false,bool restoredPaused=true) {
    std::ostringstream out; out << "{\"protocol\":2,\"id\":" << id << ",\"kind\":" << Quoted(kind)
      << ",\"ok\":" << (ok?"true":"false") << ",\"source_sha256\":" << Quoted(GT_PLANT_SOURCE_SHA)
      << ",\"before\":"; WriteSnapshot(out,before); out << ",\"after\":"; WriteSnapshot(out,after);
    out << ",\"force_n\":"; Number(out,action); out << ",\"steps\":" << count << ",\"physics_dt_s\":";
    Number(out,dt); out << ",\"original_paused\":" << (originalPaused?"true":"false")
      << ",\"mutation_started\":" << (mutation?"true":"false");
    if (!message.empty()) out << ",\"error\":" << Quoted(message);
    if (restore) out << ",\"restored_paused\":" << (restoredPaused?"true":"false");
    out << "}\n"; return Send(fd,out.str());
  }
  bool ReadLine(int fd,std::string &line) {
    line.clear(); auto deadline=Clock::now()+std::chrono::seconds(120);
    while (!stopping.load() && Clock::now()<deadline) {
      pollfd fds[2]={{fd,POLLIN,0},{listener,POLLIN,0}};
      int result=poll(fds,2,100);
      if (result<0) { if (errno==EINTR) continue; return false; }
      if (fds[1].revents&POLLIN) RejectQueuedClients();
      if (fds[0].revents&(POLLIN|POLLHUP|POLLERR)) {
        char value; ssize_t n=recv(fd,&value,1,MSG_DONTWAIT);
        if (n==0) return false;
        if (n<0) { if (errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR) continue; return false; }
        if (line.empty()) deadline=Clock::now()+std::chrono::seconds(2);
        if (value=='\n') return true;
        if (value=='\0' || line.size()>=255) { Poison("Malformed or overlong A47 request"); return false; }
        line+=value;
      }
    }
    if (!line.empty()) Poison("Partial A47 request exceeded its wall deadline");
    return false;
  }
  bool RestoreSafeLease() {
    Snapshot state;
    if (!leased.load()) return true;
    if (awaitingAck || !Stable(state)) { Poison("Client disconnected with an unconfirmed or changed transaction"); return false; }
    force.store(0); leased.store(false);
    if (!stopping.load() && world->Running()) world->SetPaused(originalPaused);
    return true;
  }
  void HandleClient(int fd) {
    uint64_t nextId=1; std::string line;
    while (!stopping.load() && ReadLine(fd,line)) {
      uint64_t id=0; std::string verb,extra; double a=0,b=0;
      std::istringstream command(line);
      bool parsed=static_cast<bool>(command>>id>>verb>>a>>b) && !(command>>extra) &&
        std::isfinite(a) && std::isfinite(b) && id==nextId;
      Snapshot before=ReadCache(),after=before;
      if (!parsed) {
        Poison("Malformed request or non-monotonic transaction id; no replay is allowed");
        Reply(fd,id,verb.empty()?"invalid":verb,false,before,after,0,0,false,Error()); break;
      }
      ++nextId;
      bool valid=((verb=="begin" || verb=="state" || verb=="end") && a==0 && b==0) ||
        (verb=="reset" && std::abs(a)<=.95 && std::abs(b)<=2) ||
        (verb=="step" && std::abs(a)<=8 && b>=1 && b<=200 && b==std::floor(b));
      if (!valid || poisoned.load() || !EndpointOwned()) {
        Poison("Invalid command, poisoned lease or changed endpoint identity");
        Reply(fd,id,verb,false,before,after,0,0,false,Error()); break;
      }
      // A subsequent ordered request acknowledges the preceding response.
      // EOF immediately after an unacknowledged mutation keeps the world paused.
      awaitingAck=false;
      if (verb=="begin") {
        if (leased.load()) { Poison("Only one begin is allowed per learning lease"); Reply(fd,id,verb,false,before,after,0,0,false,Error()); break; }
        originalPaused=world->IsPaused(); world->SetPaused(true); leased.store(true);
        after=ReadCache(); expected=after;
        if (!world->Running() || !after.paused || !Finite(after)) Poison("Cannot enter a stable paused learning lease");
        if (!Reply(fd,id,verb,!poisoned.load(),before,after,0,0,true,poisoned.load()?Error():"")) { Poison("Begin response delivery failed"); break; }
        if (poisoned.load()) break;
        continue;
      }
      if (!Stable(before)) { Reply(fd,id,verb,false,before,ReadCache(),0,0,false,Error()); break; }
      if (verb=="state") {
        if (!Reply(fd,id,verb,true,before,before,0,0,false)) { Poison("State response delivery failed"); break; }
        continue;
      }
      if (verb=="end") {
        force.store(0); leased.store(false); world->SetPaused(originalPaused);
        bool restored=world->IsPaused()==originalPaused;
        Reply(fd,id,verb,restored,before,before,0,0,true,restored?"":"Original pause state was not restored",true,world->IsPaused());
        if (!restored) Poison("Original pause state was not restored");
        break;
      }
      const unsigned steps=verb=="reset"?1u:static_cast<unsigned>(b);
      force.store(verb=="step"?a:0);
      if (verb=="reset") { resetX.store(a); resetV.store(b); resetPending.store(true); }
      // Neither physics callback needs an application lock held across Step.
      inStep.store(true); world->Step(steps); force.store(0); inStep.store(false);
      if (stopping.load()) break;
      after=ReadCache();
      if (!world->Running() || !after.paused || !Finite(after) || poisoned.load() ||
          after.iterations!=before.iterations+steps || after.callbacks!=before.callbacks+steps ||
          std::abs((after.time-before.time)-dt*steps)>1e-8 || resetPending.load())
        Poison("World::Step did not finish the exact requested paused physics/callback/time transition");
      if (poisoned.load()) { Reply(fd,id,verb,false,before,after,a,steps,true,Error()); break; }
      expected=after; awaitingAck=true;
      if (!Reply(fd,id,verb,true,before,after,verb=="step"?a:0,steps,true)) { Poison("Mutation response delivery failed"); break; }
    }
    force.store(0);
    if (!stopping.load() && leased.load()) RestoreSafeLease();
  }
  void Serve() noexcept {
    try {
      while (!stopping.load()) {
        pollfd descriptor{listener,POLLIN,0};
        if (poll(&descriptor,1,100)<=0) continue;
        int fd=accept4(listener,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
        if (fd<0) continue;
        if (!PeerAllowed(fd) || poisoned.load() || !EndpointOwned()) { close(fd); continue; }
        client.store(fd); HandleClient(fd); client.store(-1); close(fd);
      }
    } catch (const std::exception &exception) { Poison(std::string("A47 worker failed: ")+exception.what()); }
      catch (...) { Poison("A47 worker failed with an unknown exception"); }
    const int fd=client.exchange(-1); if (fd>=0) close(fd);
    workerDone.store(true);
  }
  physics::ModelPtr model; physics::WorldPtr world; physics::JointPtr joint;
  event::ConnectionPtr beginConnection,endConnection;
  std::thread worker; std::mutex cacheMutex,errorMutex;
  Snapshot cached,expected;
  std::atomic<bool> stopping{false},workerDone{true},leased{false},poisoned{false},inStep{false},resetPending{false};
  std::atomic<double> force{0},resetX{0},resetV{0};
  std::atomic<uint64_t> callbacks{0}; std::atomic<int> client{-1};
  int listener=-1; double dt=.001; bool originalPaused=true,awaitingAck=false,ownsEndpoint=false;
  std::string path,error; dev_t endpointDevice{}; ino_t endpointInode{};
};
GZ_REGISTER_MODEL_PLUGIN(LearningSlider)
}
