#include "qr_detector/qr_detector_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"

PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

namespace qr_detector
{

    QrDetectorNodelet::QrDetectorNodelet() : it_(nh_)
    {
    }

    QrDetectorNodelet::~QrDetectorNodelet()
    {
        img_subscriber_.shutdown();
    }

    void QrDetectorNodelet::onInit()
    {
        nh_ = getNodeHandle();

        tags_publisher_ =
            nh_.advertise<std_msgs::String>("qr_codes", 1, std::bind(&QrDetectorNodelet::connectCallback, this),
                                            std::bind(&QrDetectorNodelet::disconnectCallback, this));
        img_subscriber_ = it_.subscribe("image", 1, &QrDetectorNodelet::imageCallback, this);

        ROS_WARN("Initializing nodelet... [%s]", nh_.getNamespace().c_str());
    }

    void QrDetectorNodelet::connectCallback()
    {
        // ROS_INFO("tags_publisher_.getNumSubscribers() = %d", tags_publisher_.getNumSubscribers());
        // ROS_INFO("img_subscriber_ is null? %d", img_subscriber_ == nullptr);
        if (!img_subscriber_ && tags_publisher_.getNumSubscribers() > 0)
        {
            ROS_WARN("Connecting to image topic.");
            img_subscriber_ = it_.subscribe("image", 1, &QrDetectorNodelet::imageCallback, this);
        }
    }

    void QrDetectorNodelet::disconnectCallback()
    {
        if (tags_publisher_.getNumSubscribers() == 0)
        {
            ROS_WARN("Unsubscribing from image topic.");
            img_subscriber_.shutdown();
        }
    }

    void QrDetectorNodelet::imageCallback(const sensor_msgs::ImageConstPtr &image)
    {
        cv_bridge::CvImageConstPtr cv_image;
        std_msgs::String qr_msg;
        ROS_INFO("Image received.");
        try
        {
            cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        auto tags = detector_.detect(cv_image->image, 10);
        for (auto &tag : tags)
        {
            ROS_INFO("QR code detected: %s", tag.message.c_str());
            qr_msg.data = tag.message;
            tags_publisher_.publish(qr_msg);
        }
    }

}  // namespace qr_detector
