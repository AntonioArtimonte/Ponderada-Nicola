#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TonguinhoNode : public rclcpp::Node
{
public:
    TonguinhoNode() : Node("tonguinho_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("tonguinho_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TonguinhoNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Olá do Tonguinho!";
        RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TonguinhoNode>());
    rclcpp::shutdown();
    return 0;
}
