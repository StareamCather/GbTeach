#ifndef GAZEBOTEACH_PANEL_HH
#define GAZEBOTEACH_PANEL_HH
#ifndef Q_MOC_RUN
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <memory>
#include <string>
#endif
namespace gazebo {
class TeachingPanel final : public GUIPlugin {
  Q_OBJECT
 public:
  TeachingPanel();
  ~TeachingPanel() override;
  void Load(sdf::ElementPtr element) override;
 private slots:
  void OnPause();
  void OnResume();
  void OnStep();
  void OnTick();
 private:
  struct Shared;
  std::shared_ptr<Shared> shared;
  transport::NodePtr node;
  transport::PublisherPtr controls;
  transport::SubscriberPtr statistics,requests;
  QTimer *timer=nullptr;
  QLabel *timeLabel=nullptr,*stateLabel=nullptr,*iterationLabel=nullptr,*resultLabel=nullptr;
  QPushButton *pauseButton=nullptr,*resumeButton=nullptr,*stepButton=nullptr;
  void Send(const std::string &action);
  void Cache();
};
}
#endif
