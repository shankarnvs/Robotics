#include "continuum_rviz_plugin/continuum_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Dense>
#include <cmath>

namespace
{
Eigen::Matrix4d computeTransform(double length, double angle_rad)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  if (std::abs(angle_rad) < 1e-6)
  {
    T(2, 3) = length;
    return T;
  }

  double R = length / angle_rad;

  T(0, 0) = std::cos(angle_rad);
  T(0, 2) = std::sin(angle_rad);
  T(2, 0) = -std::sin(angle_rad);
  T(2, 2) = std::cos(angle_rad);

  T(0, 3) = R * (1.0 - std::cos(angle_rad));
  T(2, 3) = R * std::sin(angle_rad);

  return T;
}
}

ContinuumPanel::ContinuumPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  link_count_ = new QSpinBox;
  link_count_->setMinimum(1);
  link_count_->setMaximum(20);
  link_count_->setValue(2);

  QPushButton* btn = new QPushButton("Generate Links");
  connect(btn, SIGNAL(clicked()), this, SLOT(generateLinks()));

  tabs_ = new QTabWidget;

  matrix_display_ = new QTextEdit;
  matrix_display_->setReadOnly(true);

  layout->addWidget(link_count_);
  layout->addWidget(btn);
  layout->addWidget(tabs_);
  layout->addWidget(new QLabel("End Effector Transform"));
  layout->addWidget(matrix_display_);

  setLayout(layout);
}

ContinuumPanel::~ContinuumPanel() {}

void ContinuumPanel::onInitialize()
{
  auto context = getDisplayContext();
  if (!context)
  {
    RCLCPP_ERROR(rclcpp::get_logger("continuum_panel"), "No display context!");
    return;
  }

  auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RCLCPP_ERROR(rclcpp::get_logger("continuum_panel"), "Failed to get ROS node!");
    return;
  }

  node_ = ros_node_abstraction->get_raw_node();

  pub_ = node_->create_publisher<continuum_msgs::msg::RobotState>(
    "/continuum/state", 10);
}

void ContinuumPanel::generateLinks()
{
  tabs_->clear();
  links_ui_.clear();

  int n = link_count_->value();

  for (int i = 0; i < n; i++)
  {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout;

    // 🔥 Create and store FIRST
    links_ui_.emplace_back();
    auto& lw = links_ui_.back();

    // ===== Link =====
    lw.length = new QLineEdit("1.0");
    layout->addWidget(new QLabel("Length"));
    layout->addWidget(lw.length);

    // ===== Servo 1 =====
    lw.enable_servo1 = new QCheckBox("Enable Servo 1");
    layout->addWidget(lw.enable_servo1);

    connect(lw.enable_servo1, &QCheckBox::stateChanged,
            this, &ContinuumPanel::publishState);

    lw.servo1.angle_slider = new QSlider(Qt::Horizontal);
    lw.servo1.angle_slider->setRange(-180, 180);

    lw.servo1.angle_text = new QLineEdit("0");
    lw.servo1.radius = new QLineEdit("0.02");

    lw.servo1.target_link = new QComboBox;
    for (int j = i + 1; j < n; j++)
      lw.servo1.target_link->addItem(QString::number(j));

    QHBoxLayout* s1 = new QHBoxLayout;
    s1->addWidget(lw.servo1.angle_slider);
    s1->addWidget(lw.servo1.angle_text);

    layout->addWidget(new QLabel("Servo 1 Angle"));
    layout->addLayout(s1);
    layout->addWidget(new QLabel("Servo 1 Radius"));
    layout->addWidget(lw.servo1.radius);
    layout->addWidget(new QLabel("Servo 1 Target Link"));
    layout->addWidget(lw.servo1.target_link);

    connect(lw.servo1.angle_slider, &QSlider::valueChanged,
            [this, i](int val){
              auto& lw = links_ui_[i];
              lw.servo1.angle_text->setText(QString::number(val));
              publishState();
            });

    connect(lw.servo1.angle_text, &QLineEdit::editingFinished,
            [this, i]() {
              auto& lw = links_ui_[i];
              lw.servo1.angle_slider->setValue(
                lw.servo1.angle_text->text().toInt());
              publishState();
            });

    // ===== Servo 2 =====
    lw.enable_servo2 = new QCheckBox("Enable Servo 2");
    layout->addWidget(lw.enable_servo2);

    connect(lw.enable_servo2, &QCheckBox::stateChanged,
            this, &ContinuumPanel::publishState);

    lw.servo2.angle_slider = new QSlider(Qt::Horizontal);
    lw.servo2.angle_slider->setRange(-180, 180);

    lw.servo2.angle_text = new QLineEdit("0");
    lw.servo2.radius = new QLineEdit("0.02");

    lw.servo2.target_link = new QComboBox;
    for (int j = i + 1; j < n; j++)
      lw.servo2.target_link->addItem(QString::number(j));

    QHBoxLayout* s2 = new QHBoxLayout;
    s2->addWidget(lw.servo2.angle_slider);
    s2->addWidget(lw.servo2.angle_text);

    layout->addWidget(new QLabel("Servo 2 Angle"));
    layout->addLayout(s2);
    layout->addWidget(new QLabel("Servo 2 Radius"));
    layout->addWidget(lw.servo2.radius);
    layout->addWidget(new QLabel("Servo 2 Target Link"));
    layout->addWidget(lw.servo2.target_link);

    connect(lw.servo2.angle_slider, &QSlider::valueChanged,
            [this, i](int val){
              auto& lw = links_ui_[i];
              lw.servo2.angle_text->setText(QString::number(val));
              publishState();
            });

    connect(lw.servo2.angle_text, &QLineEdit::editingFinished,
            [this, i]() {
              auto& lw = links_ui_[i];
              lw.servo2.angle_slider->setValue(
                lw.servo2.angle_text->text().toInt());
              publishState();
            });

    tab->setLayout(layout);
    tabs_->addTab(tab, QString("Link %1").arg(i));
  }

  publishState();
}

void ContinuumPanel::publishState()
{
  if (!pub_)
  {
    RCLCPP_WARN(rclcpp::get_logger("continuum_panel"), "Publisher not ready yet");
    return;
  }

  continuum_msgs::msg::RobotState msg;
  Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

  for (size_t i = 0; i < links_ui_.size(); i++)
  {
    auto& lw = links_ui_[i];

    continuum_msgs::msg::Link link;
    link.id = i;
    link.length = lw.length->text().toDouble();
    msg.links.push_back(link);

    double angle = 0.0;

    if (lw.enable_servo1->isChecked())
    {
      continuum_msgs::msg::Servo s;
      s.id = i * 2;
      s.parent_link = i;
      s.angle = lw.servo1.angle_slider->value() * M_PI / 180.0;

      if (lw.servo1.target_link->count() > 0)
        s.affects_links.push_back(
          lw.servo1.target_link->currentText().toInt());

      msg.servos.push_back(s);
      angle += s.angle;
    }

    if (lw.enable_servo2->isChecked())
    {
      continuum_msgs::msg::Servo s;
      s.id = i * 2 + 1;
      s.parent_link = i;
      s.angle = lw.servo2.angle_slider->value() * M_PI / 180.0;

      if (lw.servo2.target_link->count() > 0)
        s.affects_links.push_back(
          lw.servo2.target_link->currentText().toInt());

      msg.servos.push_back(s);
      angle += s.angle;
    }

    T_total = T_total * computeTransform(link.length, angle);
  }

  QString mat;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
      mat += QString::number(T_total(i,j),'f',3)+" ";
    mat += "\n";
  }

  matrix_display_->setPlainText(mat);

  pub_->publish(msg);
}

PLUGINLIB_EXPORT_CLASS(ContinuumPanel, rviz_common::Panel)