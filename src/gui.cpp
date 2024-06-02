#include <vector>
#include <time.h>
#include <chrono>
#include <thread>
#include <string>
#include <string.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/core.hpp>
// #include <opencv2/imgcodes.hpp>
#include "hyper_drive/GUI.h"


namespace hyper_drive
{
GUI::GUI( QWidget* parent )
  : rviz::Panel( parent )
{
  // CONSTANT VALUES
  ximea_lambda = {662.74414484, 678.44427179, 691.47995842, 702.79743883,
       718.98022991, 730.09886825, 743.11429966, 757.61797177,
       770.56671122, 779.24140246, 794.57439591, 805.38484958,
       817.52315993, 831.92643985, 842.41924582, 853.91129525,
       867.76937301, 878.09883114, 887.16460955, 901.40985581,
       909.37019374, 919.05507754, 928.17011884, 932.90958447};
  imec_lambda = {1122.48954896, 1140.51012341, 1162.45096103, 1192.69076274,
       1210.67247958, 1298.52642212, 1382.2788887 , 1465.39984657,
       1654.60278798};
  vimba_lambda = {480, 600, 700};
  spinner_ptr = new ros::AsyncSpinner(2);
  spinner_ptr->start();
  channel_listener_ = nh_.subscribe("/hsi_gui/channel_img", 1000, &GUI::updateChannel, this);
  histogram_listener_ = nh_.subscribe("/hsi_gui/hist_img", 1000, &GUI::updateHistogram, this);
  // Initiate publishers
  channel_publisher_ = nh_.advertise<std_msgs::Int8>("/hsi_gui/channel", 10);
  camera_publisher_ = nh_.advertise<std_msgs::String>("/hsi_gui/camera", 10);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &GUI::timerCallback);
  timer_->start(10); // 1 second interval
  layout = new QGridLayout;

  title = new QLabel("<P><b><FONT SIZE = 5>Hyperspectral Cube Viewer</b></P></br>");
  layout->addWidget(title, 0, 0, 1, -1, Qt::AlignCenter);
  // Create hbox for images
  images = new QWidget();
  // Create placeholder for channel image
  image_layout = new QHBoxLayout(images);
  QImage image_channel(300, 300, QImage::Format_RGB888);
  image_channel.fill(Qt::white);
  label_channel = new QLabel();
  label_channel->setPixmap(QPixmap::fromImage(image_channel));
  image_layout->addWidget(label_channel);

  // Create placeholder for histogram image
  QImage image_hist(300, 300, QImage::Format_RGB888);
  image_hist.fill(Qt::black);
  label_hist = new QLabel();
  label_hist->setPixmap(QPixmap::fromImage(image_hist));
  image_layout->addWidget(label_hist);

  layout->addWidget(images);
  // Create hbox for controls
  controls = new QWidget();
  controls_layout = new QHBoxLayout(controls);
  /* 
  ========================================
  COMBOBOX FOR CAMERA SELECTION
  ========================================
  */
  topicSelector = new QComboBox();
  QStringList topics;
  topics << "[Camera]" << "ximea" << "imec" << "vimba";
  topicSelector->addItems(topics);
  topicSelector->setCurrentIndex(1);
  currentCube = new QLabel(tr("Current Cube:"));
  controls_layout->addWidget(currentCube);
  controls_layout->addWidget(topicSelector);
  /* 
  ========================================
  SLIDER FOR HSI CUBE CHANNEL SELECTION
  ========================================
  */
  channel_slider = new QSlider(Qt::Horizontal);
  channel_slider->setMinimum(0);
  channel_slider->setMaximum(24);
  channel_slider->setTickInterval(1);
  // Add Handler for Slider Change
  currentChannelValue = new QLabel(tr("Current \u03BB:"));
  controls_layout->addWidget(currentChannelValue);
  controls_layout->addWidget(channel_slider);
  // Add the horizontal box
  std::string initial_label = "Current Lambda: " + std::to_string(ximea_lambda[0]);
  current_lambda = new QLabel(QString(initial_label.c_str()));
  layout->addWidget(controls);
  layout->addWidget(current_lambda);
  setLayout(layout);
  // Initiate event listeners
  QObject::connect(channel_slider, &QSlider::sliderMoved, this, &GUI::handleSliderEvent);
  QObject::connect(topicSelector, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                    this, &GUI::handleTopicChange);
}

void GUI::handleSliderEvent(int value) {
  // Publish the numeric index to ROS
  std::string camera = topicSelector->currentText().toStdString();
  std::string current_label;
  if (camera == "imec") {
      channel_slider->setToolTip(QString(std::to_string(imec_lambda[value]).c_str()));
      current_label = "Current Lambda: " + std::to_string(imec_lambda[value]);
      current_lambda->setText(QString(current_label.c_str()));
      current_lambda->repaint();
      QCoreApplication::processEvents();
  } else if (camera == "ximea") {
      channel_slider->setToolTip(QString(std::to_string(ximea_lambda[value]).c_str()));
      current_label = "Current Lambda: " + std::to_string(ximea_lambda[value]);
      current_lambda->setText(QString(current_label.c_str()));
      QCoreApplication::processEvents();
  } else if (camera == "vimba") {
      channel_slider->setToolTip(QString(std::to_string(vimba_lambda[value]).c_str()));
      current_label = "Current Lambda: " + std::to_string(vimba_lambda[value]);
      current_lambda->setText(QString(current_label.c_str()));
      QCoreApplication::processEvents();
  }
  std_msgs::Int8 msg;
  msg.data = value;
  channel_publisher_.publish(msg);
}

void GUI::handleTopicChange(int index) {
  // Publish the name of the currently selected camera to ROS
  std_msgs::String msg;
  // TODO make these values not hardcoded
  std::string camera = topicSelector->currentText().toStdString();
  // Always reset the slider to 0
  channel_slider->setValue(0);
  channel_slider->setMaximum(0);
  std::string current_label;
  if (camera == "imec") {
      channel_slider->setMaximum(8);
  } else if (camera == "ximea") {
      channel_slider->setMaximum(23);
  } else if (camera == "vimba") {
      channel_slider->setMaximum(2);
  }
  msg.data = topicSelector->currentText().toStdString();
  camera_publisher_.publish(msg);
}

void GUI::timerCallback()
{
  ros::spinOnce();
}

void GUI::updateHistogram(sensor_msgs::Image msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC4);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Update the image in the UI
  label_hist->setPixmap(QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGBA8888)));
}

void GUI::updateChannel(sensor_msgs::Image msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  label_channel->setPixmap(QPixmap::fromImage(QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888)));
}

void GUI::save( rviz::Config config ) const
{
  rviz::Panel::save(config);
}


void GUI::load( const rviz::Config& config )
{
  rviz::Panel::load(config);
}


} // end namespace hyper_drive

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hyper_drive::GUI, rviz::Panel)