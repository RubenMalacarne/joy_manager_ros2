#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
        this->declare_parameter<std::string>("button_X_service", "joystick/button_X");
        this->declare_parameter<std::string>("button_Circle_service", "joystick/button_Circle");
        this->declare_parameter<std::string>("button_Triangle_service", "joystick/button_Triangle");
        this->declare_parameter<std::string>("button_Square_service", "joystick/button_Square");
        this->declare_parameter<std::string>("button_L1_service", "joystick/button_L1");
        this->declare_parameter<std::string>("button_R1_service", "joystick/button_R1");
        this->declare_parameter<std::string>("button_L2_service", "joystick/button_L2");
        this->declare_parameter<std::string>("button_R2_service", "joystick/button_R2");
        this->declare_parameter<std::string>("button_Share_service", "joystick/button_Share");
        this->declare_parameter<std::string>("button_Options_service", "joystick/button_Options");
        this->declare_parameter<std::string>("button_L3_service", "joystick/button_L3");
        this->declare_parameter<std::string>("button_R3_service", "joystick/button_R3");
        this->declare_parameter<std::string>("axis_6_right_srv", "joystick/axis_6_left");
        this->declare_parameter<std::string>("axis_6_left_srv", "joystick/axis_6_right");
        this->declare_parameter<std::string>("axis_7_up_service", "joystick/axis_7_up");
        this->declare_parameter<std::string>("axis_7_down_service", "joystick/axis_7_down");
        
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
        std::string button_X_service = this->get_parameter("button_X_service").as_string();
        std::string button_Circle_service = this->get_parameter("button_Circle_service").as_string();
        std::string button_Triangle_service = this->get_parameter("button_Triangle_service").as_string();
        std::string button_Square_service = this->get_parameter("button_Square_service").as_string();
        std::string button_L1_service = this->get_parameter("button_L1_service").as_string();
        std::string button_R1_service = this->get_parameter("button_R1_service").as_string();
        std::string button_L2_service = this->get_parameter("button_L2_service").as_string();
        std::string button_R2_service = this->get_parameter("button_R2_service").as_string();
        std::string button_Share_service = this->get_parameter("button_Share_service").as_string();
        std::string button_Options_service = this->get_parameter("button_Options_service").as_string();
        std::string button_L3_service = this->get_parameter("button_L3_service").as_string();
        std::string button_R3_service = this->get_parameter("button_R3_service").as_string();
        std::string axis_6_right_srv = this->get_parameter("axis_6_right_srv").as_string();
        std::string axis_6_left_srv = this->get_parameter("axis_6_left_srv").as_string();
        std::string axis_7_up_service = this->get_parameter("axis_7_up_service").as_string();
        std::string axis_7_down_service = this->get_parameter("axis_7_down_service").as_string();
        
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
        
        // Initialize button state tracking
        previous_button_states_.fill(false);
        axis_6_negative_state_ = false;
        axis_6_positive_state_ = false;
        axis_7_negative_state_ = false;
        axis_7_positive_state_ = false;
        
        roll_pub_ = this->create_publisher<std_msgs::msg::Float32>(roll_topic, 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>(pitch_topic, 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float32>(thrust_topic, 10);
        l2_gear_pub_ = this->create_publisher<std_msgs::msg::Float32>(l2_gear_topic, 10);
        r2_gear_pub_ = this->create_publisher<std_msgs::msg::Float32>(r2_gear_topic, 10);
        button_X_client_ = this->create_client<std_srvs::srv::SetBool>(button_X_service);
        button_Circle_client_ = this->create_client<std_srvs::srv::SetBool>(button_Circle_service);
        button_Triangle_client_ = this->create_client<std_srvs::srv::SetBool>(button_Triangle_service);
        button_Square_client_ = this->create_client<std_srvs::srv::SetBool>(button_Square_service);
        button_L1_client_ = this->create_client<std_srvs::srv::SetBool>(button_L1_service);
        button_R1_client_ = this->create_client<std_srvs::srv::SetBool>(button_R1_service);
        button_L2_client_ = this->create_client<std_srvs::srv::SetBool>(button_L2_service);
        button_R2_client_ = this->create_client<std_srvs::srv::SetBool>(button_R2_service);
        button_Share_client_ = this->create_client<std_srvs::srv::SetBool>(button_Share_service);
        button_Options_client_ = this->create_client<std_srvs::srv::SetBool>(button_Options_service);
        button_L3_client_ = this->create_client<std_srvs::srv::SetBool>(button_L3_service);
        button_R3_client_ = this->create_client<std_srvs::srv::SetBool>(button_R3_service);
        axis_6_left_client_ = this->create_client<std_srvs::srv::SetBool>(axis_6_right_srv);
        axis_6_right_client_ = this->create_client<std_srvs::srv::SetBool>(axis_6_left_srv);
        axis_7_up_client_ = this->create_client<std_srvs::srv::SetBool>(axis_7_up_service);
        axis_7_down_client_ = this->create_client<std_srvs::srv::SetBool>(axis_7_down_service);

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

        // Asse 6 come servizio left/right
        if (msg->axes.size() >= 7) {
            handle_axis_as_buttons(msg->axes[6], 6, axis_6_left_client_, axis_6_right_client_, 
                                 axis_6_negative_state_, axis_6_positive_state_, "RIGHT", "LEFT");
        }

        // Asse 7 come servizio up/down
        if (msg->axes.size() >= 8) {
            handle_axis_as_buttons(msg->axes[7], 7, axis_7_down_client_, axis_7_up_client_, 
                                 axis_7_negative_state_, axis_7_positive_state_, "DOWN", "UP");
        }

        // Pulsanti DualShock 4 (12 pulsanti)
        if (msg->buttons.size() >= 13) {
            call_button_service_on_change(button_X_client_, msg->buttons[0], 0);           // X
            call_button_service_on_change(button_Circle_client_, msg->buttons[1], 1);      // Circle
            call_button_service_on_change(button_Triangle_client_, msg->buttons[2], 2);    // Triangle
            call_button_service_on_change(button_Square_client_, msg->buttons[3], 3);      // Square
            call_button_service_on_change(button_L1_client_, msg->buttons[4], 4);          // L1
            call_button_service_on_change(button_R1_client_, msg->buttons[5], 5);          // R1
            call_button_service_on_change(button_L2_client_, msg->buttons[6], 6);          // L2
            call_button_service_on_change(button_R2_client_, msg->buttons[7], 7);          // R2
            call_button_service_on_change(button_Share_client_, msg->buttons[8], 8);       // Share
            call_button_service_on_change(button_Options_client_, msg->buttons[9], 9);     // Options
            call_button_service_on_change(button_L3_client_, msg->buttons[10], 10);        // L3 (stick sinistro premuto)
            call_button_service_on_change(button_R3_client_, msg->buttons[11], 11);        // R3 (stick destro premuto)
        }
    }

    void call_button_service_on_change(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
                                        bool button_pressed, int button_index)
    {
        if (button_pressed != previous_button_states_[button_index])
        {
            previous_button_states_[button_index] = button_pressed;

            if (client->service_is_ready())
            {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = button_pressed;

                auto future = client->async_send_request(request);

                RCLCPP_INFO(this->get_logger(),
                            "Button %d state changed to %s",
                            button_index, button_pressed ? "PRESSED" : "RELEASED");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Service for button %d not available", button_index);
            }
        }
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

    void handle_axis_as_buttons(float axis_value, int axis_index,
                               rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr negative_client,
                               rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr positive_client,
                               bool& negative_state, bool& positive_state,
                               const std::string& negative_name, const std::string& positive_name)
    {
        bool negative_pressed = axis_value < -0.5;
        bool positive_pressed = axis_value > 0.5;
        
        // Gestione stato negativo
        if (negative_pressed != negative_state)
        {
            negative_state = negative_pressed;
            if (negative_client->service_is_ready())
            {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = negative_pressed;
                auto future = negative_client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Axis %d %s state changed to %s", 
                           axis_index, negative_name.c_str(), negative_pressed ? "PRESSED" : "RELEASED");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Service for axis %d %s not available", axis_index, negative_name.c_str());
            }
        }
        
        // Gestione stato positivo
        if (positive_pressed != positive_state)
        {
            positive_state = positive_pressed;
            if (positive_client->service_is_ready())
            {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = positive_pressed;
                auto future = positive_client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Axis %d %s state changed to %s", 
                           axis_index, positive_name.c_str(), positive_pressed ? "PRESSED" : "RELEASED");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Service for axis %d %s not available", axis_index, positive_name.c_str());
            }
        }
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

    // Service clients per pulsanti
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_X_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_Circle_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_Triangle_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_Square_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_L1_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_R1_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_L2_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_R2_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_Share_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_Options_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_L3_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_R3_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr axis_6_left_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr axis_6_right_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr axis_7_up_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr axis_7_down_client_;

    // Parametri per la rimappatura degli assi
    std::array<double, 6> axis_min_;  // Valori minimi per ogni asse (0-5)
    std::array<double, 6> axis_max_;  // Valori massimi per ogni asse (0-5)
    
    // Button state tracking
    std::array<bool, 12> previous_button_states_;
    bool axis_6_negative_state_;
    bool axis_6_positive_state_;
    bool axis_7_negative_state_;
    bool axis_7_positive_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualShock4Node>());
    rclcpp::shutdown();
    return 0;
}