// GUI owns QWidget access. Transport callbacks only copy data under a mutex.
#include "gui_plugin.hh"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QThread>
#include <gazebo/common/Exception.hh>
#include <unistd.h>
#include <chrono>
#include <deque>
#include <iomanip>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#ifndef GZTEACH_SOURCE_SHA
#define GZTEACH_SOURCE_SHA "unrecorded"
#endif
namespace gazebo {
namespace {
long long Wall() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
long long Ns(const msgs::Time &t){return static_cast<long long>(t.sec())*1000000000LL+t.nsec();}
std::string Q(const std::string &s){std::ostringstream out;out<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')out<<'\\';if(c=='\n')out<<"\\n";else if(c=='\r')out<<"\\r";else if(c=='\t')out<<"\\t";else if(c>=32)out<<c;}return out.str()+'"';}
}
struct TeachingPanel::Shared {
  std::mutex mutex;
  bool active=true,haveStats=false,busy=false;
  msgs::WorldStatistics latest;
  unsigned long long statsReceived=0,ticks=0,loads=0,published=0,completed=0;
  unsigned long long pauseClicks=0,resumeClicks=0,stepClicks=0,pendingStats=0,startIterations=0;
  int count=100,decimals=3;
  long long startNs=0,targetNs=0,deadline=0;
  std::string configSHA,client,cache,pending,lastCompleted,error;
  std::deque<std::string> clicks;
  std::set<std::string> receipts;
  transport::PublisherPtr response;
};
TeachingPanel::TeachingPanel():GUIPlugin(){setObjectName("gazeboteach_l30_panel");}
TeachingPanel::~TeachingPanel(){
  if(timer)timer->stop();
  if(shared){std::lock_guard<std::mutex> lock(shared->mutex);shared->active=false;}
  statistics.reset();requests.reset();if(node)node->Fini();node.reset();shared.reset();
}
void TeachingPanel::Load(sdf::ElementPtr element){
  if(!element||!element->HasElement("step_count"))gzthrow("Load L30 through world/gui/plugin, not an empty -g configuration");
  shared=std::make_shared<Shared>();auto state=shared;
  state->count=element->Get<int>("step_count");state->decimals=element->Get<int>("display_decimals");
  state->configSHA=element->Get<std::string>("config_sha256");
  if(state->count<10||state->count>500||state->decimals<1||state->decimals>6)gzthrow("L30 finite integer settings are outside the exercise range");
  state->client=std::to_string(getpid())+"-"+std::to_string(Wall());state->loads=1;
  setStyleSheet("QWidget#gazeboteach_l30_panel { background: #16324f; border: 2px solid #77b7e5; border-radius: 5px; } QLabel { color: white; background: transparent; } QPushButton { color: #16324f; background: #edf5ff; min-height: 28px; padding: 3px; } QPushButton:disabled { color: #76899b; background: #b8c6d2; }");
  auto layout=new QVBoxLayout(this);layout->setContentsMargins(12,10,12,10);
  auto title=new QLabel(QString::fromUtf8("L30 · GazeboTeach 控制面板"),this);layout->addWidget(title);
  timeLabel=new QLabel("waiting for native world statistics",this);layout->addWidget(timeLabel);
  stateLabel=new QLabel("unknown",this);layout->addWidget(stateLabel);
  iterationLabel=new QLabel("iterations = unknown",this);layout->addWidget(iterationLabel);
  auto row=new QHBoxLayout;
  pauseButton=new QPushButton("Pause",this);pauseButton->setObjectName("pause");
  resumeButton=new QPushButton("Resume",this);resumeButton->setObjectName("resume");
  stepButton=new QPushButton(QString("Step %1").arg(state->count),this);stepButton->setObjectName("step");
  row->addWidget(pauseButton);row->addWidget(resumeButton);row->addWidget(stepButton);layout->addLayout(row);
  resultLabel=new QLabel("ready: no command published",this);resultLabel->setWordWrap(true);layout->addWidget(resultLabel);
  connect(pauseButton,&QPushButton::clicked,this,&TeachingPanel::OnPause);
  connect(resumeButton,&QPushButton::clicked,this,&TeachingPanel::OnResume);
  connect(stepButton,&QPushButton::clicked,this,&TeachingPanel::OnStep);
  move(16,16);resize(420,230);setMinimumSize(420,230);
  node.reset(new transport::Node());node->Init("lesson30");
  controls=node->Advertise<msgs::WorldControl>("~/world_control",20);
  state->response=node->Advertise<msgs::Response>("~/response",100);
  statistics=node->Subscribe<msgs::WorldStatistics>("~/world_stats",[state](const boost::shared_ptr<const msgs::WorldStatistics>& message){
    std::lock_guard<std::mutex> lock(state->mutex);if(!state->active)return;
    state->latest.CopyFrom(*message);state->haveStats=true;++state->statsReceived;
  });
  requests=node->Subscribe<msgs::Request>("~/request",[state](const boost::shared_ptr<const msgs::Request>& request){
    if(request->request()!="gazeboteach_panel_snapshot"&&request->request()!="gazeboteach_panel_click")return;
    bool accepted=true;std::string copy,nonce=request->data();
    // Command data is action:nonce. Both values are created by the SDK client.
    {std::lock_guard<std::mutex> lock(state->mutex);if(!state->active)return;
      if(request->request()=="gazeboteach_panel_click"){
        auto colon=nonce.find(':');std::string action=nonce.substr(0,colon);
        if(colon==std::string::npos||!(action=="pause"||action=="resume"||action=="step"))accepted=false;
        else{nonce=nonce.substr(colon+1);if(state->receipts.insert(request->data()).second){state->clicks.push_back(action);if(state->receipts.size()>1024)state->receipts.erase(state->receipts.begin());}}
      }
      copy=state->cache;
    }
    auto close=copy.rfind('}');if(close==std::string::npos)return;
    copy.insert(close,",\"accepted\":"+std::string(accepted?"true":"false")+",\"query_nonce\":"+Q(nonce));
    msgs::Any data;data.set_type(msgs::Any::STRING);data.set_string_value(copy);
    msgs::Response reply;reply.set_id(request->id());reply.set_request(request->request());reply.set_response(accepted?"success":"error");
    reply.set_type("gazebo.msgs.Any");reply.set_serialized_data(data.SerializeAsString());state->response->Publish(reply);
  });
  timer=new QTimer(this);connect(timer,&QTimer::timeout,this,&TeachingPanel::OnTick);timer->start(50);Cache();
}
void TeachingPanel::OnPause(){Send("pause");}
void TeachingPanel::OnResume(){Send("resume");}
void TeachingPanel::OnStep(){Send("step");}
void TeachingPanel::Send(const std::string &action){
  auto &s=*shared;std::lock_guard<std::mutex> lock(s.mutex);
  if(action=="pause")++s.pauseClicks;else if(action=="resume")++s.resumeClicks;else ++s.stepClicks;
  if(s.busy||!s.haveStats){s.error="Wait for the pending command and actual native statistics";return;}
  if(action=="step"&&!s.latest.paused()){s.error="Pause before requesting physics ticks";return;}
  if(!controls->HasConnections()){s.error="No actual world_control subscriber; no command sent";return;}
  msgs::WorldControl command;
  command.set_pause(action!="resume");if(action=="step")command.set_multi_step(s.count);
  s.startNs=Ns(s.latest.sim_time());s.startIterations=s.latest.iterations();s.targetNs=s.startNs+(action=="step"?s.count*1000000LL:0);
  s.pendingStats=s.statsReceived;s.pending=action;s.busy=true;s.error.clear();s.deadline=Wall()+6000000000LL;++s.published;
  controls->Publish(command); // exactly one publication; completion comes from stats
}
void TeachingPanel::OnTick(){
  std::deque<std::string> clicks;
  {std::lock_guard<std::mutex> lock(shared->mutex);clicks.swap(shared->clicks);}
  for(const auto &action:clicks){if(action=="pause")pauseButton->click();else if(action=="resume")resumeButton->click();else stepButton->click();}
  {
    auto &s=*shared;std::lock_guard<std::mutex> lock(s.mutex);++s.ticks;
    if(s.busy&&s.haveStats&&s.statsReceived>s.pendingStats){
      bool done=false;
      if(s.pending=="step"){
        const auto now=Ns(s.latest.sim_time());
        if(now>=s.targetNs){
          if(now==s.targetNs&&s.latest.iterations()==s.startIterations+s.count&&s.latest.paused())done=true;
          else{s.error="Native step completion differs from the single requested pulse";s.busy=false;s.pending.clear();}
        }
      }else done=s.latest.paused()==(s.pending=="pause");
      if(done){++s.completed;s.lastCompleted=s.pending;s.busy=false;s.pending.clear();}
    }
    if(s.busy&&Wall()>s.deadline){s.error="Native completion timed out; pulse was NOT replayed. Inspect world state before another click.";s.busy=false;s.pending.clear();}
    if(s.haveStats){
      timeLabel->setText(QString("t = %1 s").arg(Ns(s.latest.sim_time())*1e-9,0,'f',s.decimals));
      stateLabel->setText(s.latest.paused()?"paused":"running");iterationLabel->setText(QString("iterations = %1").arg(static_cast<qulonglong>(s.latest.iterations())));
    }
    pauseButton->setEnabled(s.haveStats&&!s.busy);resumeButton->setEnabled(s.haveStats&&!s.busy);
    stepButton->setEnabled(s.haveStats&&!s.busy&&s.latest.paused());
    resultLabel->setText(QString::fromStdString(!s.error.empty()?s.error:s.busy?"waiting for actual "+s.pending:"completed = "+std::to_string(s.completed)+"; sent = "+std::to_string(s.published)));
  }
  Cache();
}
void TeachingPanel::Cache(){
  auto rect=[](QPushButton *button){QPoint p=button->mapToGlobal(QPoint(0,0));std::ostringstream out;out<<"{\"x\":"<<p.x()<<",\"y\":"<<p.y()<<",\"width\":"<<button->width()<<",\"height\":"<<button->height()<<",\"enabled\":"<<(button->isEnabled()?"true":"false")<<",\"text\":"<<Q(button->text().toUtf8().constData())<<'}';return out.str();};
  const auto pause=rect(pauseButton),resume=rect(resumeButton),step=rect(stepButton);
  auto &s=*shared;std::lock_guard<std::mutex> lock(s.mutex);std::ostringstream out;
  out<<"{\"world\":\"lesson30\",\"client_id\":"<<Q(s.client)<<",\"source_sha256\":"<<Q(GZTEACH_SOURCE_SHA)<<",\"config_sha256\":"<<Q(s.configSHA)
    <<",\"step_count\":"<<s.count<<",\"display_decimals\":"<<s.decimals<<",\"load_count\":"<<s.loads
    <<",\"init_hook_supported\":false,\"reset_hook_supported\":false,\"init_count\":0,\"reset_count\":0"
    <<",\"stats_received\":"<<s.statsReceived<<",\"timer_ticks\":"<<s.ticks<<",\"stats_available\":"<<(s.haveStats?"true":"false")
    <<",\"paused\":"<<(s.latest.paused()?"true":"false")<<",\"sim_time_ns\":"<<Q(std::to_string(Ns(s.latest.sim_time())))<<",\"iterations\":"<<Q(std::to_string(s.latest.iterations()))
    <<",\"label_time\":"<<Q(timeLabel->text().toUtf8().constData())<<",\"label_state\":"<<Q(stateLabel->text().toUtf8().constData())<<",\"label_iterations\":"<<Q(iterationLabel->text().toUtf8().constData())
    <<",\"buttons\":{\"pause\":"<<pause<<",\"resume\":"<<resume<<",\"step\":"<<step<<"},\"click_counts\":{\"pause\":"<<s.pauseClicks<<",\"resume\":"<<s.resumeClicks<<",\"step\":"<<s.stepClicks<<'}'
    <<",\"published_controls\":"<<s.published<<",\"completed_controls\":"<<s.completed<<",\"busy\":"<<(s.busy?"true":"false")<<",\"pending_action\":"<<Q(s.pending)
    <<",\"last_completed_action\":"<<Q(s.lastCompleted)<<",\"last_error\":"<<Q(s.error)<<",\"step_start_ns\":"<<Q(std::to_string(s.startNs))<<",\"step_target_ns\":"<<Q(std::to_string(s.targetNs))<<'}';
  s.cache=out.str();
}
GZ_REGISTER_GUI_PLUGIN(TeachingPanel)
}
