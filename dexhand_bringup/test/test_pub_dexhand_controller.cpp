//
// Created by ubuntu on 1/24/25.
//

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <chrono>
#include <thread>

class JointJogPublisher : public rclcpp::Node
{
public:
    JointJogPublisher() : Node("dexhand_controller_publisher")
    {
        // 创建一个发布者，发布到 "~/reference" 话题
        publisher_ = this->create_publisher<control_msgs::msg::JointJog>("reference", 10);
        // 设置定时器，每秒发布一次消息
        timer_ = this->create_wall_timer(std::chrono::microseconds(100), [this]() { publish_joint_jog(); });
    }

private:
    int offerset = 0;

    std::vector<std::string> all_dof_masks{
            "THJ4", "THJ3", "THJ2", "THJ1", "FFJ4", "FFJ3", "FFJ2", "FFJ1",
            "MFJ4", "MFJ3", "MFJ2", "MFJ1", "RFJ4", "RFJ3", "RFJ2", "RFJ1",
            "LFJ5", "LFJ4", "LFJ3", "LFJ2", "LFJ1"
    };
    void publish_joint_jog()
    {
        // 创建 JointJog 消息
        control_msgs::msg::JointJog joint_jog;
        // 填充消息内容
        joint_jog.header.stamp = rclcpp::Time();
        joint_jog.header.frame_id = "joint_frame";
        // 示例：添加一个关节的运动指令
        if (offerset % 2 == 0)
        {
            for (int i = 0; i < all_dof_masks.size(); ++i)
            {
                joint_jog.joint_names.push_back(all_dof_masks[i]);
                joint_jog.displacements.push_back(0.1);
                joint_jog.velocities.push_back(0.1);
            }
        }
        else
        {
            for (int i = 0; i < all_dof_masks.size(); ++i)
            {
                joint_jog.joint_names.push_back(all_dof_masks[i]);
                joint_jog.displacements.push_back(0.5);
                joint_jog.velocities.push_back(0.5);
            }
        }
        // 发布消息
        publisher_->publish(joint_jog);
        offerset++;
        RCLCPP_INFO(this->get_logger(), "Published JointJog message");
    }

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointJogPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
