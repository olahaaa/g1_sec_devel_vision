#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>

class ImgSubNode: public rclcpp::Node
{
public:
    ImgSubNode(): Node("img_sub_node")
    {
        img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw",10,
            std::bind(&ImgSubNode::img_callback,this,std::placeholders::_1)
        );
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
    
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;

        cv::imshow("realsense_image", cv_ptr->image);
        cv::waitKey(1);

        static int count = 0;
        if (count++ % 60 == 0){
            RCLCPP_INFO(this->get_logger(),"width:%d, height:%d, frame_id:%s", width, height, msg->header.frame_id.c_str());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgSubNode>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}