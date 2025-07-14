#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class DualShock4Node : public rclcpp::Node
{
public:
    DualShock4Node() : Node("dualshock4_joystick_controller")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&DualShock4Node::joy_callback, this, _1));
        
        this->declare_parameter<std::string>("rpyt_topic", "joystick_dualshock4/rpyt");
        this->declare_parameter<std::string>("button_X_topic", "joystick/button_X");
        this->declare_parameter<std::string>("button_Circle_topic", "joystick/button_Circle");
        this->declare_parameter<std::string>("button_Triangle_topic", "joystick/button_Triangle");
        this->declare_parameter<std::string>("button_Square_topic", "joystick/button_Square");
        this->declare_parameter<std::string>("button_L1_topic", "joystick/button_L1");
        this->declare_parameter<std::string>("button_R1_topic", "joystick/button_R1");
        this->declare_parameter<std::string>("button_L2_topic", "joystick/button_L2");
        this->declare_parameter<std::string>("button_R2_topic", "joystick/button_R2");
        this->declare_parameter<std::string>("button_Share_topic", "joystick/button_Share");
        this->declare_parameter<std::string>("button_Options_topic", "joystick/button_Options");
        this->declare_parameter<std::string>("button_L3_topic", "joystick/button_L3");
        this->declare_parameter<std::string>("button_R3_topic", "joystick/button_R3");
        
        std::string rpyt_topic = this->get_parameter("rpyt_topic").as_string();
        std::string button_X_topic = this->get_parameter("button_X_topic").as_string();
        std::string button_Circle_topic = this->get_parameter("button_Circle_topic").as_string();
        std::string button_Triangle_topic = this->get_parameter("button_Triangle_topic").as_string();
        std::string button_Square_topic = this->get_parameter("button_Square_topic").as_string();
        std::string button_L1_topic = this->get_parameter("button_L1_topic").as_string();
        std::string button_R1_topic = this->get_parameter("button_R1_topic").as_string();
        std::string button_L2_topic = this->get_parameter("button_L2_topic").as_string();
        std::string button_R2_topic = this->get_parameter("button_R2_topic").as_string();
        std::string button_Share_topic = this->get_parameter("button_Share_topic").as_string();
        std::string button_Options_topic = this->get_parameter("button_Options_topic").as_string();
        std::string button_L3_topic = this->get_parameter("button_L3_topic").as_string();
        std::string button_R3_topic = this->get_parameter("button_R3_topic").as_string();
        
        rpyt_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(rpyt_topic, 10);
        button_X_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_X_topic, 10);
        button_Circle_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_Circle_topic, 10);
        button_Triangle_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_Triangle_topic, 10);
        button_Square_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_Square_topic, 10);
        button_L1_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_L1_topic, 10);
        button_R1_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_R1_topic, 10);
        button_L2_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_L2_topic, 10);
        button_R2_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_R2_topic, 10);
        button_Share_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_Share_topic, 10);
        button_Options_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_Options_topic, 10);
        button_L3_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_L3_topic, 10);
        button_R3_pub_ = this->create_publisher<std_msgs::msg::Bool>(button_R3_topic, 10);

        RCLCPP_INFO(this->get_logger(), "DualShock 4 Joystick Controller Node Started");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Assi principali RPYT -> assi 0,1,2,3 (stick sinistro X/Y, stick destro X/Y)
        if (msg->axes.size() >= 4) {
            std_msgs::msg::Float32MultiArray rpyt_msg;
            rpyt_msg.data = {msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]};
            rpyt_pub_->publish(rpyt_msg);
        }

        // Pulsanti DualShock 4 (12 pulsanti)
        if (msg->buttons.size() >= 12) {
            publish_button(button_X_pub_, msg->buttons[0]);           // X
            publish_button(button_Circle_pub_, msg->buttons[1]);      // Circle
            publish_button(button_Triangle_pub_, msg->buttons[2]);    // Triangle
            publish_button(button_Square_pub_, msg->buttons[3]);      // Square
            publish_button(button_L1_pub_, msg->buttons[4]);          // L1
            publish_button(button_R1_pub_, msg->buttons[5]);          // R1
            publish_button(button_L2_pub_, msg->buttons[6]);          // L2
            publish_button(button_R2_pub_, msg->buttons[7]);          // R2
            publish_button(button_Share_pub_, msg->buttons[8]);       // Share
            publish_button(button_Options_pub_, msg->buttons[9]);     // Options
            publish_button(button_L3_pub_, msg->buttons[10]);         // L3 (stick sinistro premuto)
            publish_button(button_R3_pub_, msg->buttons[11]);         // R3 (stick destro premuto)
        }
    }

    void publish_button(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, int button_value)
    {
        std_msgs::msg::Bool msg;
        msg.data = (button_value != 0);
        pub->publish(msg);
    }

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Publisher per assi
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpyt_pub_;

    // Publisher per pulsanti
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_X_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_Circle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_Triangle_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_Square_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_L1_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_R1_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_L2_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_R2_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_Share_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_Options_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_L3_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_R3_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualShock4Node>());
    rclcpp::shutdown();
    return 0;
}