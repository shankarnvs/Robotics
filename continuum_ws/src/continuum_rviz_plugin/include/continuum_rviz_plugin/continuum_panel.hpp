#ifndef CONTINUUM_PANEL_HPP
#define CONTINUUM_PANEL_HPP

#include <QObject>
#include <rviz_common/panel.hpp>

#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <continuum_msgs/msg/robot_state.hpp>

struct ServoWidgets {
  QSlider* angle_slider;
  QLineEdit* angle_text;
  QLineEdit* radius;
  QComboBox* affects;
};

struct LinkWidgets {
  QLineEdit* length;
  QCheckBox* enable_servo1;
  QCheckBox* enable_servo2;
  ServoWidgets servo1;
  ServoWidgets servo2;
};

class ContinuumPanel : public rviz_common::Panel
{
Q_OBJECT

public:
  ContinuumPanel(QWidget * parent = nullptr);
  virtual ~ContinuumPanel();

private Q_SLOTS:
  void generateLinks();
  void publishState();

private:
  QSpinBox* link_count_;
  QTabWidget* tabs_;

  std::vector<LinkWidgets> links_ui_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<continuum_msgs::msg::RobotState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif