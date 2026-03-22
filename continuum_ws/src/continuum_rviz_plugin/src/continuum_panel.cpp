#include "continuum_rviz_plugin/continuum_panel.hpp"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>

#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>

ContinuumPanel::ContinuumPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  // 🔥 Initialize ROS (safe inside RViz plugin)
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("continuum_panel_node");

  pub_ = node_->create_publisher<continuum_msgs::msg::RobotState>(
    "/continuum/state", 10);

  // 🔥 Spin node periodically
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      rclcpp::spin_some(node_);
    });

  link_count_ = new QSpinBox;
  link_count_->setMinimum(1);
  link_count_->setMaximum(20);

  QPushButton* gen_btn = new QPushButton("Generate Links");
  connect(gen_btn, SIGNAL(clicked()), this, SLOT(generateLinks()));

  tabs_ = new QTabWidget;

  layout->addWidget(new QLabel("Number of Links"));
  layout->addWidget(link_count_);
  layout->addWidget(gen_btn);
  layout->addWidget(tabs_);

  setLayout(layout);
}

ContinuumPanel::~ContinuumPanel() {}

void ContinuumPanel::generateLinks()
{
  tabs_->clear();
  links_ui_.clear();

  int n = link_count_->value();

  for (int i = 0; i < n; i++)
  {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout;

    LinkWidgets lw;

    // Link length
    lw.length = new QLineEdit("10.0");
    layout->addWidget(new QLabel("Length"));
    layout->addWidget(lw.length);

    // ===== Servo 1 =====
    lw.enable_servo1 = new QCheckBox("Enable Servo 1");
    layout->addWidget(lw.enable_servo1);

    lw.servo1.angle_slider = new QSlider(Qt::Horizontal);
    lw.servo1.angle->setRange(-90, 90);
	lw.servo1.angle_text = new QLineEdit("0");

    lw.servo1.radius = new QLineEdit("5");

    lw.servo1.affects = new QComboBox;
    for (int j = i + 1; j < n; j++)
      lw.servo1.affects->addItem(QString::number(j));

    layout->addWidget(new QLabel("Servo1 Angle"));
    layout->addWidget(lw.servo1.angle_slider);
	layout->addWidget(lw.servo1.angle_text);
    layout->addWidget(new QLabel("Radius"));
    layout->addWidget(lw.servo1.radius);
    layout->addWidget(new QLabel("Affects Link"));
    layout->addWidget(lw.servo1.affects);

    // ===== Servo 2 =====
    lw.enable_servo2 = new QCheckBox("Enable Servo 2");
    layout->addWidget(lw.enable_servo2);

    lw.servo2.angle_slider = new QSlider(Qt::Horizontal);
    lw.servo2.angle->setRange(-180, 180);
	lw.servo2.angle_text = new QLineEdit("0");

    lw.servo2.radius = new QLineEdit("5");

    lw.servo2.affects = new QComboBox;
    for (int j = i + 1; j < n; j++)
      lw.servo2.affects->addItem(QString::number(j));

    layout->addWidget(new QLabel("Servo2 Angle"));
    layout->addWidget(lw.servo2.angle_slider);
	layout->addWidget(lw.servo2.angle_text);
    layout->addWidget(new QLabel("Radius"));
    layout->addWidget(lw.servo2.radius);
    layout->addWidget(new QLabel("Affects Link"));
    layout->addWidget(lw.servo2.affects);

    // 🔥 REAL-TIME PUBLISH CONNECTIONS
	connect(lw.servo1.angle_slider, &QSlider::valueChanged,
        [=](int val){
            lw.servo1.angle_text->setText(QString::number(val));
            publishState();
        });
	
	connect(lw.servo1.angle_text, &QLineEdit::editingFinished,
        [=](){
            int val = lw.servo1.angle_text->text().toInt();
            lw.servo1.angle_slider->setValue(val);
            publishState();
        });
		
	connect(lw.servo2.angle_slider, &QSlider::valueChanged,
        [=](int val){
            lw.servo1.angle_text->setText(QString::number(val));
            publishState();
        });
	
	connect(lw.servo2.angle_text, &QLineEdit::editingFinished,
        [=](){
            int val = lw.servo1.angle_text->text().toInt();
            lw.servo1.angle_slider->setValue(val);
            publishState();
        });
		
    tab->setLayout(layout);
    tabs_->addTab(tab, QString("Link %1").arg(i));

    links_ui_.push_back(lw);
  }

  // 🔥 Publish initial state
  publishState();
}

void ContinuumPanel::publishState()
{
  continuum_msgs::msg::RobotState msg;

  for (size_t i = 0; i < links_ui_.size(); i++)
  {
    auto& lw = links_ui_[i];

    continuum_msgs::msg::Link link;
    link.id = i;
    link.length = lw.length->text().toDouble();
    msg.links.push_back(link);

    if (lw.enable_servo1->isChecked())
    {
      continuum_msgs::msg::Servo s;
      s.id = i * 2;
      s.parent_link = i;
      s.angle = lw.servo1.angle->value() * M_PI / 180.0;
      s.horn_radius = lw.servo1.radius->text().toDouble();

      if (lw.servo1.affects->count() > 0)
        s.affects_links.push_back(lw.servo1.affects->currentText().toInt());

      msg.servos.push_back(s);
    }

    if (lw.enable_servo2->isChecked())
    {
      continuum_msgs::msg::Servo s;
      s.id = i * 2 + 1;
      s.parent_link = i;
      s.angle = lw.servo2.angle->value() * M_PI / 180.0;
      s.horn_radius = lw.servo2.radius->text().toDouble();

      if (lw.servo2.affects->count() > 0)
        s.affects_links.push_back(lw.servo2.affects->currentText().toInt());

      msg.servos.push_back(s);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Publishing state...");
  pub_->publish(msg);
}

PLUGINLIB_EXPORT_CLASS(ContinuumPanel, rviz_common::Panel)