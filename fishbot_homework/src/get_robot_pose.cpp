#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>

using namespace std::chrono_literals;

class TFListener:public rclcpp::Node{
    public:
        TFListener():Node("tf_listener"){
            buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);
            timer_ = this->create_wall_timer(5s,std::bind(&TFListener::getTransform,this));
        }

        void getTransform(){
            try
            {
                const auto transform = buffer_->lookupTransform(
                    "base_footprint" , "map" ,rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(1.0f)
                );
                const auto &translation = transform.transform.translation;
                const auto &rotation = transform.transform.rotation;
                double yaw,pitch,roll;
                tf2::getEulerYPR(rotation,yaw,pitch,roll);
                RCLCPP_INFO(get_logger(),"平移：{x= %f, y= %f, z=%f}",translation.x,translation.y,translation.z);
                RCLCPP_INFO(get_logger(),"旋转四元数：{x= %f, y= %f, z=%f, w=%f}",rotation.x,rotation.y,rotation.z,rotation.w);
                RCLCPP_INFO(get_logger(),"旋转欧拉角：{ %f,  %f, %f}",roll,pitch,yaw);
            }
            catch(tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(),"不能够获取坐标变换,原因:%s",ex.what());
            }
            
        }
    private:
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
