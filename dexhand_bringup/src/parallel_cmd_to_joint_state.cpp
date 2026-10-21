#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ParallelCmdToJointState : public rclcpp::Node {
public:
    ParallelCmdToJointState() : Node("parallel_cmd_to_joint_state") {
        // 声明并获取参数
        declare_parameter("cmd_topic", "/dexhand_position_controller/commands");
        declare_parameter("joint_state_topic", "/parallel/dexhand/joint_states");
        declare_parameter("joints", std::vector<std::string>{});
        // 等待joints参数可用
        while (!this->has_parameter("joints")) {
            RCLCPP_INFO(this->get_logger(), "Waiting for parameters to be loaded...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        cmd_topic_ = get_parameter("cmd_topic").as_string();
        joint_state_topic_ = get_parameter("joint_state_topic").as_string();
        joints_ = get_parameter("joints").as_string_array();

        // 创建订阅者
        cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            cmd_topic_,
            10,
            std::bind(&ParallelCmdToJointState::cmdCallback, this, std::placeholders::_1)
        );

        // 创建发布者
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            joint_state_topic_, 
            10
        );

        RCLCPP_INFO(this->get_logger(), "ParallelCmdToJointState node initialized");
        RCLCPP_INFO(this->get_logger(), "Node namespace: %s", this->get_namespace());
        RCLCPP_INFO(this->get_logger(), "Number of joints: %ld", joints_.size());
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", joint_state_topic_.c_str());
    }

private:
    void cmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.name = joints_;
        joint_state.position = msg->data;
        
        // 确保关节数量匹配
        if (joint_state.name.size() != joint_state.position.size()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Number of joints (%ld) does not match number of positions (%ld)",
                joint_state.name.size(),
                joint_state.position.size()
            );
            return;
        }

        joint_state_pub_->publish(joint_state);
    }

    std::string cmd_topic_;
    std::string joint_state_topic_;
    std::vector<std::string> joints_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParallelCmdToJointState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
