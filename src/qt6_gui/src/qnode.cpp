#include "qnode.h"

qnode::qnode()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("img_sub_node");

    mode_pub_ = node_->create_publisher<result_msgs::msg::Mode>(
                "/mode_info", 10);

    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/yolo_image", 10, std::bind(&qnode::imageCallback, this, std::placeholders::_1));

    force_sub_ = node_->create_subscription<result_msgs::msg::Force>(
            "/force_info", 10, std::bind(&qnode::forceCallback, this, std::placeholders::_1));

    timer_ = node_->create_wall_timer(
                 std::chrono::milliseconds(10),
                 std::bind(&qnode::publish_message, this));

    mode_info.mode = 127;
}

void qnode::publish_message() 
{
    mode_pub_->publish(mode_info);
}

void qnode::set_image_label(QLabel* image_label)
{
    image_label_ = image_label;
}

void qnode::set_current_mode_label(QLabel* current_mode_label)
{
    current_mode_label_ = current_mode_label;
    current_mode_label_->setText("Current Mode: \nInit System");
}

void qnode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (mode_info.mode == 5 && !msg->data.empty())
    {
        QImage image(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
        if(!image.isNull()) {
          QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, image), Q_ARG(quint32, mode_info.mode));
        }
    }
}


void qnode::forceCallback(const result_msgs::msg::Force msg)
{
    if(mode_info.mode == 4 || mode_info.mode == 6)
    {
        // X_scaled = (X - X_min) / (X_max - X_min) * (new_max - new_min) + new_min
        int x_scaled = msg.x / 20000 * 300;
        int y_scaled = msg.y / 20000 * 200;
        //RCLCPP_INFO(node_->get_logger(), "x_scaled: %d, y_scaled: %d", x_scaled, y_scaled);
        cv::Mat img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
        cv::Point start(320, 240);
        
        int end_x = 320+x_scaled*4;
        int end_y = 240-y_scaled*4;
        
        if (end_x <= 0) end_x = 0;
        if (end_x >= 640) end_x = 640;
        if (end_y <= 0) end_y = 0;
        if (end_y >= 480) end_y = 480;
        
        cv::Point end(end_x, end_y);
        cv::arrowedLine(img, start, end, cv::Scalar(255, 0, 0), 20, cv::LINE_AA, 0, 0.15);

        QImage image = matToQImage(img);
        QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, image), Q_ARG(quint32, mode_info.mode));
    }
}


void qnode::updateImage(const QImage& image, const quint32 mode)
{
    if(!image.isNull()) {
    // QLabel에 이미지 출력
      image_label_->setPixmap(QPixmap::fromImage(image));
      
      if (mode == 1)
        current_mode_label_->setText("Current Mode:\nInit Load cell");
      else if (mode == 3)
        current_mode_label_->setText("Current Mode:\nStop");
      else if (mode == 4)
        current_mode_label_->setText("Current Mode:\nPush/Pull");
      else if (mode == 5)
        current_mode_label_->setText("Current Mode:\nHuman Following");
      else
        current_mode_label_->setText("Current Mode:\nWaiting");
    }
}

QImage qnode::matToQImage(const cv::Mat& mat)
{
    // Mat이 비어있는지 확인
    if (mat.empty())
        return QImage();

    // Mat의 차원 확인
    switch (mat.type())
    {
        case CV_8UC1:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_Grayscale8);
        case CV_8UC3:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_RGB888);
        case CV_8UC4:
            return QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_ARGB32);
        default:
            break;
    }

    return QImage();
}

void qnode::run()
{
    rclcpp::spin(node_);
    rclcpp::shutdown();
    RCLCPP_INFO(node_->get_logger(), "qnode shutdown!");
    //qDebug() << "qnode shutdown!";
}

qnode::~qnode()
{
    if (image_label_) {
        delete image_label_;
    }
}
