#ifndef CONTINUUM_PANEL_HPP
#define CONTINUUM_PANEL_HPP

#include <QObject>
#include <rviz_common/panel.hpp>

// Qt
#include <QSpinBox>
#include <QTabWidget>
#include <QSlider>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <continuum_msgs/msg/robot_state.hpp>

// ================= Servo UI =================
struct ServoWidgets
{
  QSlider* angle_slider;
  QLineEdit* angle_text;
  QLineEdit* radius;
  QComboBox* target_link;   // ✅ NEW
};

// ================= Link UI =================
struct LinkWidgets
{
  QLineEdit* length;

  QCheckBox* enable_servo1;
  QCheckBox* enable_servo2;

  ServoWidgets servo1;
  ServoWidgets servo2;
};

// ================= Panel =================
class ContinuumPanel : public rviz_common::Panel
{
Q_OBJECT

public:
  explicit ContinuumPanel(QWidget * parent = nullptr);
  virtual ~ContinuumPanel();
  
protected:   
  void onInitialize() override;

private Q_SLOTS:
  void generateLinks();
  void publishState();

private:
  // UI
  QSpinBox* link_count_;
  QTabWidget* tabs_;
  QTextEdit* matrix_display_;

  std::vector<LinkWidgets> links_ui_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<continuum_msgs::msg::RobotState>::SharedPtr pub_;
  //rclcpp::TimerBase::SharedPtr timer_;
};

#endif
