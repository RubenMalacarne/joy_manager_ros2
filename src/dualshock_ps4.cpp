#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>
#include <array>

using std::placeholders::_1;

class DualShock4Node : public rclcpp::Node
{
public:
    DualShock4Node() : Node("dualshock4_joystick_controller")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&DualShock4Node::joy_callback, this, _1));
        
        this->declare_parameter<std::string>("roll_topic", "joystick/roll");
        this->declare_parameter<std::string>("pitch_topic", "joystick/pitch");
        this->declare_parameter<std::string>("yaw_topic", "joystick/yaw");
        this->declare_parameter<std::string>("thrust_topic", "joystick/thrust");
        this->declare_parameter<std::string>("l2_gear_topic", "joystick/l2_gear");
        this->declare_parameter<std::string>("r2_gear_topic", "joystick/r2_gear");
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
        
        // Parametri per la rimappatura degli assi
        this->declare_parameter<double>("axis_0_min", -1.0);
        this->declare_parameter<double>("axis_0_max", 1.0);
        this->declare_parameter<double>("axis_1_min", -1.0);
        this->declare_parameter<double>("axis_1_max", 1.0);
        this->declare_parameter<double>("axis_2_min", -1.0);
        this->declare_parameter<double>("axis_2_max", 1.0);
        this->declare_parameter<double>("axis_3_min", -1.0);
        this->declare_parameter<double>("axis_3_max", 1.0);
        this->declare_parameter<double>("axis_4_min", -1.0);
        this->declare_parameter<double>("axis_4_max", 1.0);
        this->declare_parameter<double>("axis_5_min", -1.0);
        this->declare_parameter<double>("axis_5_max", 1.0);
        
        // Valori di output rimappati
        std::string roll_topic = this->get_parameter("roll_topic").as_string();
        std::string pitch_topic = this->get_parameter("pitch_topic").as_string();
        std::string yaw_topic = this->get_parameter("yaw_topic").as_string();
        std::string thrust_topic = this->get_parameter("thrust_topic").as_string();
        std::string l2_gear_topic = this->get_parameter("l2_gear_topic").as_string();
        std::string r2_gear_topic = this->get_parameter("r2_gear_topic").as_string();
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
        
        // Caricamento parametri per la rimappatura degli assi
        axis_min_[0] = this->get_parameter("axis_0_min").as_double();
        axis_max_[0] = this->get_parameter("axis_0_max").as_double();
        axis_min_[1] = this->get_parameter("axis_1_min").as_double();
        axis_max_[1] = this->get_parameter("axis_1_max").as_double();
        axis_min_[2] = this->get_parameter("axis_2_min").as_double();
        axis_max_[2] = this->get_parameter("axis_2_max").as_double();
        axis_min_[3] = this->get_parameter("axis_3_min").as_double();
        axis_max_[3] = this->get_parameter("axis_3_max").as_double();
        axis_min_[4] = this->get_parameter("axis_4_min").as_double();
        axis_max_[4] = this->get_parameter("axis_4_max").as_double();
        axis_min_[5] = this->get_parameter("axis_5_min").as_double();
        axis_max_[5] = this->get_parameter("axis_5_max").as_double();
        
        roll_pub_ = this->create_publisher<std_msgs::msg::Float32>(roll_topic, 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>(pitch_topic, 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float32>(thrust_topic, 10);
        l2_gear_pub_ = this->create_publisher<std_msgs::msg::Float32>(l2_gear_topic, 10);
        r2_gear_pub_ = this->create_publisher<std_msgs::msg::Float32>(r2_gear_topic, 10);
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
        for (int i = 0; i < 6; i++) {
            RCLCPP_INFO(this->get_logger(), "Axis %d input range: [%.2f, %.2f]", i, axis_min_[i], axis_max_[i]);
        }
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Assi principali separati individualmente
        if (msg->axes.size() >= 5) {
            // Roll (stick sinistro X)
            publish_axis_remapped(roll_pub_, msg->axes[0], 0);
            
            // Pitch (stick sinistro Y)
            publish_axis_remapped(pitch_pub_, msg->axes[1], 1);
            
            // Yaw (stick destro X)
            publish_axis_remapped(yaw_pub_, msg->axes[3], 3);
            
            // Thrust (stick destro Y)
            publish_axis_remapped(thrust_pub_, msg->axes[4], 4);
        }

        // Assi per gear L2 e R2 -> assi 2 e 5
        if (msg->axes.size() >= 6) {
            publish_axis_remapped(l2_gear_pub_, msg->axes[2], 2);
            publish_axis_remapped(r2_gear_pub_, msg->axes[5], 5);
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
            publish_button(button_L3_pub_, msg->buttons[11]);         // L3 (stick sinistro premuto)
            publish_button(button_R3_pub_, msg->buttons[12]);         // R3 (stick destro premuto)
        }
    }

    void publish_button(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub, int button_value)
    {
        std_msgs::msg::Bool msg;
        msg.data = (button_value != 0);
        pub->publish(msg);
    }

    void publish_axis(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, float value)
    {
        std_msgs::msg::Float32 msg;
        msg.data = value;
        pub->publish(msg);
    }

    void publish_axis_remapped(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, float value, int axis_index)
    {
        std_msgs::msg::Float32 msg;
        msg.data = remap_axis(value, axis_index);
        pub->publish(msg);
    }

    // Funzione per rimappare i valori degli assi
    float remap_axis(float value, int axis_index)
    {
        return axis_min_[axis_index] + ( (value + 1.0) / 2.0 ) * (axis_max_[axis_index] - axis_min_[axis_index]);
    }

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Publisher per assi
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr l2_gear_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr r2_gear_pub_;

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

    // Parametri per la rimappatura degli assi
    std::array<double, 6> axis_min_;  // Valori minimi per ogni asse (0-5)
    std::array<double, 6> axis_max_;  // Valori massimi per ogni asse (0-5)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualShock4Node>());
    rclcpp::shutdown();
    return 0;
}