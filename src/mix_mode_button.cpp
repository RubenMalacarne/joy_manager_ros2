#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>

class MixModeButtonNode : public rclcpp::Node
{
public:
    MixModeButtonNode() : Node("mix_mode_button")
    {
        // Declare parameters
        this->declare_parameter<std::string>("button_sa_service", "joystick/SA");
        this->declare_parameter<std::string>("button_sd_service", "joystick/SD");
        this->declare_parameter<std::string>("button_se_service", "joystick/SE");
        this->declare_parameter<std::string>("trigger_service", "/alpine/jump");

        std::string button_sa_service = this->get_parameter("button_sa_service").as_string();
        std::string button_sd_service = this->get_parameter("button_sd_service").as_string();
        std::string button_se_service = this->get_parameter("button_se_service").as_string();
        std::string trigger_service = this->get_parameter("trigger_service").as_string();

        // Initialize button states
        button_sa_state_ = false;
        button_sd_state_ = false;
        button_se_previous_state_ = false;

        // Create services for receiving button states
        sa_service_ = this->create_service<std_srvs::srv::SetBool>(
            button_sa_service,
            std::bind(&MixModeButtonNode::sa_callback, this, std::placeholders::_1, std::placeholders::_2));

        sd_service_ = this->create_service<std_srvs::srv::SetBool>(
            button_sd_service,
            std::bind(&MixModeButtonNode::sd_callback, this, std::placeholders::_1, std::placeholders::_2));

        se_service_ = this->create_service<std_srvs::srv::Trigger>(
            button_se_service,
            std::bind(&MixModeButtonNode::se_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create client for trigger service
        trigger_client_ = this->create_client<std_srvs::srv::Trigger>(trigger_service);

        RCLCPP_INFO(this->get_logger(), "Mix Mode Button Node Started");
        RCLCPP_INFO(this->get_logger(), "Monitoring SA: %s, SD: %s, SE: %s", 
                    button_sa_service.c_str(), button_sd_service.c_str(), button_se_service.c_str());
        RCLCPP_INFO(this->get_logger(), "Trigger service: %s", trigger_service.c_str());
        RCLCPP_INFO(this->get_logger(), "Condition: SA=TRUE AND SD=TRUE, then press SE to trigger jump");
    }

private:
    void sa_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        button_sa_state_ = request->data;
        response->success = true;
        response->message = request->data ? "SA pressed" : "SA released";
        
        RCLCPP_DEBUG(this->get_logger(), "SA state: %s", button_sa_state_ ? "TRUE" : "FALSE");
    }

    void sd_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        button_sd_state_ = request->data;
        response->success = true;
        response->message = request->data ? "SD pressed" : "SD released";
        
        RCLCPP_DEBUG(this->get_logger(), "SD state: %s", button_sd_state_ ? "TRUE" : "FALSE");
    }

    void se_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused parameter

        // Check if both SA and SD are pressed
        if (button_sa_state_ && button_sd_state_)
        {
            RCLCPP_INFO(this->get_logger(), "SE pressed with SA=TRUE and SD=TRUE -> Triggering jump!");
            
            if (trigger_client_->service_is_ready())
            {
                auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = trigger_client_->async_send_request(trigger_request);
                
                response->success = true;
                response->message = "Jump triggered successfully";
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Trigger service not available");
                response->success = false;
                response->message = "Trigger service not available";
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "SE pressed but conditions not met (SA=%s, SD=%s)",
                        button_sa_state_ ? "TRUE" : "FALSE",
                        button_sd_state_ ? "TRUE" : "FALSE");
            response->success = false;
            response->message = "Conditions not met: SA and SD must both be pressed";
        }
    }

    // Service servers
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sa_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sd_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr se_service_;

    // Trigger client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;

    // Button states
    bool button_sa_state_;
    bool button_sd_state_;
    bool button_se_previous_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MixModeButtonNode>());
    rclcpp::shutdown();
    return 0;
}
