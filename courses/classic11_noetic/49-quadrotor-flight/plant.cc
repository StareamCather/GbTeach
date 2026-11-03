// A49: owned paused-world transactions for a real six-DOF rigid body.
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
#include <array>
#include <atomic>
#include <cstdlib>
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
class QuadrotorPlant : public ModelPlugin {
  using Clock = std::chrono::steady_clock;
  struct Snapshot {
    uint64_t iterations=0, callbacks=0;
    double time=0;
    std::array<double,12> state{};
    bool paused=true;
  };
 public:
  ~QuadrotorPlant() override {
    stopping.store(true); ClearEffort();
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
    model=parent; world=model->GetWorld(); body=model->GetLink("body");
    const char *value=getenv("GAZEBOTEACH_SESSION_TOKEN");
    const std::string token=value?value:"", source=GT_PLANT_SOURCE_SHA;
    auto hex=[](unsigned char c) { return (c>='0' && c<='9') || (c>='a' && c<='f'); };
    if (!body || token.size()!=32 || !std::all_of(token.begin(),token.end(),hex) ||
        source.size()!=64 || !std::all_of(source.begin(),source.end(),hex))
      gzthrow("A49 needs an owned session, quadrotor body and compiled source SHA256");
    dt=world->Physics()->GetMaxStepSize();
    if (!std::isfinite(dt) || dt<=0 || dt>.01) gzthrow("A49 physics step size is outside its bounded range");
    path="/tmp/gt-quad-"+token+".sock";
    listener=socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    if (path.size()>=sizeof(address.sun_path)) gzthrow("A49 endpoint path is too long");
    std::copy(path.begin(),path.end(),address.sun_path);
    if (listener<0 || bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0)
      gzthrow("Cannot bind unique A49 endpoint; existing endpoints are never removed");
    struct stat identity{};
    if (lstat(path.c_str(),&identity)!=0 || !S_ISSOCK(identity.st_mode) || identity.st_uid!=getuid())
      gzthrow("A49 endpoint ownership could not be established");
    endpointDevice=identity.st_dev; endpointInode=identity.st_ino; ownsEndpoint=true;
    if (chmod(path.c_str(),0600)!=0 || listen(listener,2)!=0) gzthrow("Cannot secure/listen on A49 endpoint");
    cached=ReadPhysics();
    beginConnection=event::Events::ConnectWorldUpdateBegin(std::bind(&QuadrotorPlant::OnBegin,this));
    endConnection=event::Events::ConnectWorldUpdateEnd(std::bind(&QuadrotorPlant::OnEnd,this));
    workerDone.store(false);
    worker=std::thread(&QuadrotorPlant::Serve,this);
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
    out << ",\"state\":"; WriteArray(out,s.state);
    out << ",\"paused\":" << (s.paused?"true":"false") << ",\"callbacks\":" << s.callbacks << '}';
  }
  template<size_t N>
  static void WriteArray(std::ostream &out,const std::array<double,N> &values) {
    out << '[';
    for (size_t i=0; i<N; ++i) {
      if (i) out << ',';
      Number(out,values[i]);
    }
    out << ']';
  }
  Snapshot ReadPhysics() const {
    Snapshot s; s.iterations=world->Iterations(); s.callbacks=callbacks.load();
    s.time=world->SimTime().Double(); s.paused=world->IsPaused();
    const auto pose=body->WorldPose();
    const auto euler=pose.Rot().Euler();
    const auto velocity=body->WorldLinearVel();
    const auto rates=body->RelativeAngularVel();
    s.state={pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z(),euler.X(),euler.Y(),euler.Z(),
             velocity.X(),velocity.Y(),velocity.Z(),rates.X(),rates.Y(),rates.Z()};
    return s;
  }
  Snapshot ReadCache() {
    std::lock_guard<std::mutex> lock(cacheMutex);
    Snapshot result=cached; result.paused=world->IsPaused(); return result;
  }
  static bool Finite(const Snapshot &s) {
    return std::isfinite(s.time) && std::all_of(s.state.begin(),s.state.end(),
      [](double value) { return std::isfinite(value); });
  }
  static int64_t WallNanos() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  }
  void ClearEffort() {
    for (auto &value: rotor) value.store(0);
    for (auto &value: wind) value.store(0);
  }
  void Poison(const std::string &why) {
    poisoned.store(true);
    ClearEffort();
    { std::lock_guard<std::mutex> lock(errorMutex); if (error.empty()) error=why; }
    if (!stopping.load() && world && world->Running() && !world->IsPaused()) world->SetPaused(true);
  }
  std::string Error() {
    std::lock_guard<std::mutex> lock(errorMutex);
    return error.empty()?"A49 transaction is poisoned; restart this experiment":error;
  }
  void OnBegin() {
    callbacks.fetch_add(1);
    if (stopping.load()) return;
    if (leased.load()) {
      if (!world->IsPaused()) Poison("External resume violated the paused-world flight lease");
      if (!inStep.load()) Poison("An external physics step occurred outside an A49 transaction");
      else if (!poisoned.load()) {
        // The socket worker still owns this fd while Step is active. Poll only
        // for hangup/error and keep all socket reads on the worker.
        pollfd peer{client.load(),POLLRDHUP,0};
        const int ready=poll(&peer,1,0);
        if (peer.fd<0 || (ready>0 && (peer.revents&(POLLRDHUP|POLLHUP|POLLERR|POLLNVAL))))
          Poison("Flight client disconnected during an action");
        else if (WallNanos()>=stepDeadline.load())
          Poison("Flight action exceeded its wall-clock deadline");
      }
    }
    if (resetPending.exchange(false) && !poisoned.load()) {
      model->SetWorldPose(ignition::math::Pose3d(0,0,resetZ.load(),resetRoll.load(),0,resetYaw.load()));
      model->SetLinearVel(ignition::math::Vector3d::Zero);
      model->SetAngularVel(ignition::math::Vector3d::Zero);
    }
    ignition::math::Vector3d windForce=ignition::math::Vector3d::Zero;
    if (leased.load() && inStep.load() && !poisoned.load()) {
      const ignition::math::Vector3d positions[4]={{.22,0,0},{0,.22,0},{-.22,0,0},{0,-.22,0}};
      const auto rotation=body->WorldPose().Rot();
      std::array<double,4> thrust{};
      for (size_t i=0; i<thrust.size(); ++i) {
        thrust[i]=rotor[i].load();
        body->AddForceAtRelativePosition(
          rotation.RotateVector(ignition::math::Vector3d(0,0,thrust[i])),positions[i]);
      }
      body->AddRelativeTorque(ignition::math::Vector3d(0,0,
        .018*(thrust[0]-thrust[1]+thrust[2]-thrust[3])));
      windForce.Set(wind[0].load(),wind[1].load(),0);
    }
    // Passive drag persists on ordinary manual simulation steps; rotor
    // thrust and the commanded gust exist only inside the checked action.
    body->AddForce(windForce-.12*body->WorldLinearVel());
    body->AddRelativeTorque(-.008*body->RelativeAngularVel());
  }
  void OnEnd() {
    const Snapshot state=ReadPhysics();
    std::lock_guard<std::mutex> lock(cacheMutex); cached=state;
  }
  bool Stable(Snapshot &state) {
    if (!leased.load() || poisoned.load()) return false;
    if (!world->IsPaused()) Poison("World was resumed outside the A49 lease");
    state=ReadCache();
    if (!world->Running() || !Finite(state) || state.iterations!=world->Iterations() ||
        state.callbacks!=callbacks.load() || state.iterations!=expected.iterations ||
        state.time!=expected.time || state.state!=expected.state)
      Poison("Physical state changed between A49 transactions; no action was replayed");
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
    for (int i=0; i<4; ++i) {
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
             const Snapshot &after,unsigned count,bool mutation,const std::string &message="",
             bool restore=false,bool restoredPaused=true,const std::array<double,4> &thrust={},
             const std::array<double,2> &gust={}) {
    std::ostringstream out; out << "{\"protocol\":2,\"id\":" << id << ",\"kind\":" << Quoted(kind)
      << ",\"ok\":" << (ok?"true":"false") << ",\"source_sha256\":" << Quoted(GT_PLANT_SOURCE_SHA)
      << ",\"before\":"; WriteSnapshot(out,before); out << ",\"after\":"; WriteSnapshot(out,after);
    out << ",\"applied_rotors_n\":"; WriteArray(out,thrust);
    out << ",\"wind_force_n\":"; WriteArray(out,gust);
    out << ",\"steps\":" << count << ",\"physics_dt_s\":";
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
        if (value=='\0' || line.size()>=255) { Poison("Malformed or overlong A49 request"); return false; }
        line+=value;
      }
    }
    if (!line.empty()) Poison("Partial A49 request exceeded its wall deadline");
    return false;
  }
  bool RestoreSafeLease() {
    Snapshot state;
    if (!leased.load()) return true;
    if (awaitingAck || !Stable(state)) { Poison("Client disconnected with an unconfirmed or changed transaction"); return false; }
    ClearEffort(); leased.store(false);
    if (!stopping.load() && world->Running()) world->SetPaused(originalPaused);
    return true;
  }
  void HandleClient(int fd) {
    uint64_t nextId=1; std::string line;
    while (!stopping.load() && ReadLine(fd,line)) {
      uint64_t id=0; std::string verb,word;
      std::istringstream command(line);
      std::array<double,7> values{};
      size_t count=0;
      bool parsed=static_cast<bool>(command>>id>>verb) && id==nextId;
      while (parsed && command>>word) {
        if (count==values.size()) { parsed=false; break; }
        std::istringstream number(word);
        double value;
        if (!(number>>value) || number.peek()!=std::char_traits<char>::eof() || !std::isfinite(value)) {
          parsed=false; break;
        }
        values[count++]=value;
      }
      Snapshot before=ReadCache(),after=before;
      if (!parsed) {
        Poison("Malformed request or non-monotonic transaction id; no replay is allowed");
        Reply(fd,id,verb.empty()?"invalid":verb,false,before,after,0,false,Error()); break;
      }
      ++nextId;
      const bool valid=((verb=="begin" || verb=="state" || verb=="end") && count==2 &&
                        values[0]==0 && values[1]==0) ||
        (verb=="reset" && count==3 && values[0]>=.3 && values[0]<=2 &&
         std::abs(values[1])<=.3 && std::abs(values[2])<=.8) ||
        (verb=="step" && count==7 && values[6]==20 &&
         std::abs(values[4])<=1 && std::abs(values[5])<=1 &&
         std::all_of(values.begin(),values.begin()+4,[](double value) { return value>=0 && value<=6; }));
      if (!valid || poisoned.load() || !EndpointOwned()) {
        Poison("Invalid command, poisoned lease or changed endpoint identity");
        Reply(fd,id,verb,false,before,after,0,false,Error()); break;
      }
      // A subsequent ordered request acknowledges the preceding response.
      // EOF immediately after an unacknowledged mutation keeps the world paused.
      awaitingAck=false;
      if (verb=="begin") {
        if (leased.load()) { Poison("Only one begin is allowed per learning lease"); Reply(fd,id,verb,false,before,after,0,false,Error()); break; }
        originalPaused=world->IsPaused(); world->SetPaused(true); leased.store(true);
        after=ReadCache(); expected=after;
        if (!world->Running() || !after.paused || !Finite(after)) Poison("Cannot enter a stable paused learning lease");
        if (!Reply(fd,id,verb,!poisoned.load(),before,after,0,true,poisoned.load()?Error():"")) { Poison("Begin response delivery failed"); break; }
        if (poisoned.load()) break;
        continue;
      }
      if (!Stable(before)) { Reply(fd,id,verb,false,before,ReadCache(),0,false,Error()); break; }
      if (verb=="state") {
        if (!Reply(fd,id,verb,true,before,before,0,false)) { Poison("State response delivery failed"); break; }
        continue;
      }
      if (verb=="end") {
        ClearEffort(); leased.store(false); world->SetPaused(originalPaused);
        bool restored=world->IsPaused()==originalPaused;
        Reply(fd,id,verb,restored,before,before,0,true,restored?"":"Original pause state was not restored",true,world->IsPaused());
        if (!restored) Poison("Original pause state was not restored");
        break;
      }
      const unsigned steps=verb=="reset"?1u:20u;
      std::array<double,4> thrust{};
      std::array<double,2> gust{};
      if (verb=="reset") {
        resetZ.store(values[0]); resetRoll.store(values[1]); resetYaw.store(values[2]);
        resetPending.store(true);
      } else {
        std::copy_n(values.begin(),4,thrust.begin());
        std::copy_n(values.begin()+4,2,gust.begin());
      }
      for (size_t i=0; i<thrust.size(); ++i) rotor[i].store(thrust[i]);
      for (size_t i=0; i<gust.size(); ++i) wind[i].store(gust[i]);
      // The ODE callback applies forces only while this Step is active.
      // A failed or disconnected command does not retain rotor or wind effort.
      stepDeadline.store(WallNanos()+2000000000LL);
      inStep.store(true); world->Step(steps); ClearEffort(); inStep.store(false);
      if (stopping.load()) break;
      after=ReadCache();
      if (!world->Running() || !after.paused || !Finite(after) || poisoned.load() ||
          after.iterations!=before.iterations+steps || after.callbacks!=before.callbacks+steps ||
          after.iterations!=world->Iterations() || after.callbacks!=callbacks.load() ||
          std::abs((after.time-before.time)-dt*steps)>1e-8 || resetPending.load())
        Poison("World::Step did not finish the exact requested paused physics/callback/time transition");
      if (poisoned.load()) { Reply(fd,id,verb,false,before,after,steps,true,Error()); break; }
      expected=after; awaitingAck=true;
      if (!Reply(fd,id,verb,true,before,after,steps,true,"",false,true,thrust,gust)) {
        Poison("Mutation response delivery failed"); break;
      }
    }
    ClearEffort();
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
    } catch (const std::exception &exception) { Poison(std::string("A49 worker failed: ")+exception.what()); }
      catch (...) { Poison("A49 worker failed with an unknown exception"); }
    const int fd=client.exchange(-1); if (fd>=0) close(fd);
    workerDone.store(true);
  }
  physics::ModelPtr model; physics::WorldPtr world; physics::LinkPtr body;
  event::ConnectionPtr beginConnection,endConnection;
  std::thread worker; std::mutex cacheMutex,errorMutex;
  Snapshot cached,expected;
  std::atomic<bool> stopping{false},workerDone{true},leased{false},poisoned{false},inStep{false},resetPending{false};
  std::atomic<double> rotor[4]{{0},{0},{0},{0}}, wind[2]{{0},{0}};
  std::atomic<double> resetZ{0},resetRoll{0},resetYaw{0};
  std::atomic<int64_t> stepDeadline{0};
  std::atomic<uint64_t> callbacks{0}; std::atomic<int> client{-1};
  int listener=-1; double dt=.001; bool originalPaused=true,awaitingAck=false,ownsEndpoint=false;
  std::string path,error; dev_t endpointDevice{}; ino_t endpointInode{};
};
GZ_REGISTER_MODEL_PLUGIN(QuadrotorPlant)
}
