#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "joy_manager/srv/switch_command.hpp"
#include <algorithm>
#include <array>
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;

class RadiomasterNode : public rclcpp::Node
{
public:
    RadiomasterNode() : Node("radiomaster_joystick_controller")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&RadiomasterNode::joy_callback, this, _1));

        this->declare_parameter<std::string>("roll_topic", "joystick/roll");
        this->declare_parameter<std::string>("pitch_topic", "joystick/pitch");
        this->declare_parameter<std::string>("yaw_topic", "joystick/yaw");
        this->declare_parameter<std::string>("thrust_topic", "joystick/thrust");
        this->declare_parameter<std::string>("gear_S1_topic", "joystick/gear_S1");
        this->declare_parameter<std::string>("switch_SC_service", "joystick/switch_SC");
        this->declare_parameter<std::string>("switch_SB_service", "joystick/switch_SB");
        this->declare_parameter<std::string>("button_SD_service", "joystick/SD");
        this->declare_parameter<std::string>("button_SA_service", "joystick/SA");
        this->declare_parameter<std::string>("button_SE_service", "joystick/SE");
        this->declare_parameter<std::string>("button_SAB_service", "joystick/SAB");

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
        this->declare_parameter<std::string>("switch_SC_low_label", "LOW");
        this->declare_parameter<std::string>("switch_SC_center_label", "CENTER");
        this->declare_parameter<std::string>("switch_SC_high_label", "HIGH");
        this->declare_parameter<std::string>("switch_SB_low_label", "LOW");
        this->declare_parameter<std::string>("switch_SB_center_label", "CENTER");
        this->declare_parameter<std::string>("switch_SB_high_label", "HIGH");

        std::string roll_topic = this->get_parameter("roll_topic").as_string();
        std::string pitch_topic = this->get_parameter("pitch_topic").as_string();
        std::string yaw_topic = this->get_parameter("yaw_topic").as_string();
        std::string thrust_topic = this->get_parameter("thrust_topic").as_string();
        std::string gear_S1_topic = this->get_parameter("gear_S1_topic").as_string();
        std::string switch_SC_service = this->get_parameter("switch_SC_service").as_string();
        std::string switch_SB_service = this->get_parameter("switch_SB_service").as_string();
        std::string button_SD_service = this->get_parameter("button_SD_service").as_string();
        std::string button_SA_service = this->get_parameter("button_SA_service").as_string();
        std::string button_SE_service = this->get_parameter("button_SE_service").as_string();
        std::string button_SAB_service = this->get_parameter("button_SAB_service").as_string();

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

        // Get switch SC position labels
        switch_SC_labels_[0] = this->get_parameter("switch_SC_low_label").as_string();    // LOW
        switch_SC_labels_[1] = this->get_parameter("switch_SC_center_label").as_string(); // CENTER
        switch_SC_labels_[2] = this->get_parameter("switch_SC_high_label").as_string();   // HIGH

        // Get switch SB position labels
        switch_SB_labels_[0] = this->get_parameter("switch_SB_low_label").as_string();    // LOW
        switch_SB_labels_[1] = this->get_parameter("switch_SB_center_label").as_string(); // CENTER
        switch_SB_labels_[2] = this->get_parameter("switch_SB_high_label").as_string();   // HIGH

        previous_button_states_.fill(false);
        previous_switch_states_.fill(0.0f);

        roll_pub_ = this->create_publisher<std_msgs::msg::Float32>(roll_topic, 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>(pitch_topic, 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float32>(thrust_topic, 10);
        gear_S1_pub_ = this->create_publisher<std_msgs::msg::Float32>(gear_S1_topic, 10);
        switch_SC_client_ = this->create_client<joy_manager::srv::SwitchCommand>(switch_SC_service);
        switch_SB_client_ = this->create_client<joy_manager::srv::SwitchCommand>(switch_SB_service);
        button_SD_client_ = this->create_client<std_srvs::srv::SetBool>(button_SD_service);
        button_SA_client_ = this->create_client<std_srvs::srv::SetBool>(button_SA_service);
        button_SE_client_ = this->create_client<std_srvs::srv::Trigger>(button_SE_service);
        button_SAB_client_ = this->create_client<std_srvs::srv::SetBool>(button_SAB_service);

        RCLCPP_INFO(this->get_logger(), "Radiomaster Joystick Controller Node Started");
        for (int i = 0; i < 5; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Axis %d input range: [%.2f, %.2f]", i, axis_min_[i], axis_max_[i]);
        }
        RCLCPP_INFO(this->get_logger(), "Switch SC discrete 3-state logic (0=%s, 1=%s, 2=%s)",
                    switch_SC_labels_[0].c_str(), switch_SC_labels_[1].c_str(), switch_SC_labels_[2].c_str());
        RCLCPP_INFO(this->get_logger(), "Switch SB discrete 3-state logic (0=%s, 1=%s, 2=%s)",
                    switch_SB_labels_[0].c_str(), switch_SB_labels_[1].c_str(), switch_SB_labels_[2].c_str());
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() >= 5)
        {
            publish_axis_remapped(roll_pub_, msg->axes[0], 0);
            publish_axis_remapped(pitch_pub_, msg->axes[1], 1);
            publish_axis_remapped(yaw_pub_, msg->axes[2], 2);
            publish_axis_remapped(thrust_pub_, msg->axes[3], 3);
        }
        if (msg->axes.size() >= 7)
        {
            publish_axis_remapped(gear_S1_pub_, msg->axes[4], 4);
            call_switch_service_on_change(switch_SC_client_, msg->axes[5], 0); // SC
            call_switch_service_on_change(switch_SB_client_, msg->axes[6], 1); // SB
        }
        if (msg->buttons.size() >= 4)
        {
            call_trigger_service_on_edge(button_SE_client_, msg->buttons[2], 2);
            call_button_service_on_change(button_SA_client_, msg->buttons[1], 1);
            call_button_service_on_change(button_SD_client_, msg->buttons[0], 0);
            call_button_service_on_change(button_SAB_client_, msg->buttons[3], 3);
        }
    }
    void call_trigger_service_on_edge(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                                      bool button_pressed, int button_index)
    {
        if (button_pressed && !previous_button_states_[button_index])
        {
            previous_button_states_[button_index] = true;

            if (client->service_is_ready())
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = client->async_send_request(request);

                RCLCPP_INFO(this->get_logger(),
                            "Button %d pressed -> Trigger service called", button_index);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Trigger service for button %d not available", button_index);
            }
        }
        else if (!button_pressed && previous_button_states_[button_index])
        {
            // Update on release without calling the service
            previous_button_states_[button_index] = false;
        }
    }
    void call_switch_service_on_change(rclcpp::Client<joy_manager::srv::SwitchCommand>::SharedPtr client,
                                       float switch_value, int switch_index)
    {
        int discrete_state;
        std::string position;
        if (switch_value < -0.5f)
        {
            discrete_state = 0; // LOW
        }
        else if (switch_value > 0.5f)
        {
            discrete_state = 2; // HIGH
        }
        else
        {
            discrete_state = 1; // CENTER
        }

        // Get the appropriate label based on switch index
        if (switch_index == 0) // Switch SC
        {
            position = switch_SC_labels_[discrete_state];
        }
        else if (switch_index == 1) // Switch SB
        {
            position = switch_SB_labels_[discrete_state];
        }
        else
        {
            position = "UNKNOWN";
        }

        if (discrete_state != static_cast<int>(previous_switch_states_[switch_index]))
        {
            previous_switch_states_[switch_index] = static_cast<float>(discrete_state);

            if (client->service_is_ready())
            {
                auto request = std::make_shared<joy_manager::srv::SwitchCommand::Request>();
                request->step = discrete_state;
                request->label = position;

                auto future = client->async_send_request(request);

                RCLCPP_INFO(this->get_logger(),
                            "Switch %d state changed to %s (state: %d, raw: %.2f)",
                            switch_index, position.c_str(), discrete_state, switch_value);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Service for switch %d not available", switch_index);
            }
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

    void publish_axis_remapped(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, float value, int axis_index)
    {
        std_msgs::msg::Float32 msg;
        msg.data = remap_axis(value, axis_index);
        pub->publish(msg);
    }

    float remap_axis(float value, int axis_index)
    {
        return axis_min_[axis_index] + ((value + 1.0) / 2.0) * (axis_max_[axis_index] - axis_min_[axis_index]);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gear_S1_pub_;
    rclcpp::Client<joy_manager::srv::SwitchCommand>::SharedPtr switch_SC_client_;
    rclcpp::Client<joy_manager::srv::SwitchCommand>::SharedPtr switch_SB_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_SD_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_SA_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr button_SE_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr button_SAB_client_;

    std::array<double, 6> axis_min_;
    std::array<double, 6> axis_max_;

    std::array<bool, 4> previous_button_states_;
    std::array<float, 2> previous_switch_states_;

    // Switch position labels arrays - [0]=LOW, [1]=CENTER, [2]=HIGH
    std::array<std::string, 3> switch_SC_labels_;
    std::array<std::string, 3> switch_SB_labels_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadiomasterNode>());
    rclcpp::shutdown();
    return 0;
}
