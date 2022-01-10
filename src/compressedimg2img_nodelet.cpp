/*
 * @Author: Jianheng Liu
 * @Date: 2021-12-09 21:40:57
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-09 20:21:49
 * @Description: Description
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>      // 基类Nodelet所在的头文件

using namespace std;

namespace compressedimg2img_nodelet_ns {
    class Compressedimg2ImgNodelet : public nodelet::Nodelet {
    public:
        Compressedimg2ImgNodelet() {}

    private:
        virtual void onInit() {
            ros::NodeHandle &nh = getMTNodeHandle();

            sub_img = nh.subscribe("/camera/color/image_raw/compressed", 10, &Compressedimg2ImgNodelet::img_callback,
                                   this);
            sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw/compressedDepth", 10,
                                     &Compressedimg2ImgNodelet::depth_callback, this);
            pub_img = nh.advertise<sensor_msgs::Image>("/decompressed_img", 10);
            pub_depth = nh.advertise<sensor_msgs::Image>("/decompressed_depth_img", 10);
        }

        ros::Subscriber sub_img, sub_depth;
        ros::Publisher pub_img, pub_depth;

        void img_callback(const sensor_msgs::CompressedImageConstPtr &img_msg) {
            pub_img.publish(cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->toImageMsg());
        }

        void depth_callback(const sensor_msgs::CompressedImageConstPtr &depth_msg) {
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

            // Copy message header
            cv_ptr->header = depth_msg->header;

            // Assign image encoding
            const size_t split_pos = depth_msg->format.find(';');
            const std::string image_encoding = depth_msg->format.substr(0, split_pos);
            cv_ptr->encoding = image_encoding;

            // Get compressed image data
            // https://answers.ros.org/question/51490/sensor_msgscompressedimage-decompression/
            // https://github.com/heleidsn/heleidsn.github.io/blob/c02ed13cb4ffe85ee8f03a9ad93fa55336f84f7c/source/_posts/realsense-depth-image.md
            // https://sourcegraph.com/github.com/ros-perception/image_transport_plugins/-/blob/compressed_depth_image_transport/include/compressed_depth_image_transport/codec.h
            const std::vector<uint8_t> imageData(depth_msg->data.begin() + 12, depth_msg->data.end());
            cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
            pub_depth.publish(cv_ptr->toImageMsg());
        }

    };
    PLUGINLIB_EXPORT_CLASS(compressedimg2img_nodelet_ns::Compressedimg2ImgNodelet, nodelet::Nodelet)

}