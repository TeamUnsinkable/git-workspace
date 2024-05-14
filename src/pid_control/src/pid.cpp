#include "pid_control/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include <std_msgs/msg/float64.hpp>

using namespace pid_ns;
using namespace std::chrono_literals;

PidObject::PidObject() : rclcpp::Node("pid_controller"),
    error_(3, 0), filtered_error_(3, 0), 
    error_deriv_(3, 0),  filtered_error_deriv_(3, 0)
{        
        // Initialize parameters
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<double>("Ki", 0.0);
        this->declare_parameter<double>("Kd", 0.0);
        this->declare_parameter<double>("upper_limit", 1000.0);
        this->declare_parameter<double>("lower_limit", -1000.0);
        this->declare_parameter<double>("windup_limit", 1000.0);
        this->declare_parameter<double>("cutoff_frequency", -1.0);
        this->declare_parameter<std::string>("topic_from_controller", "control_effort");
        this->declare_parameter<std::string>("topic_from_plant", "state");
        this->declare_parameter<std::string>("setpoint_topic", "setpoint");
        this->declare_parameter<std::string>("pid_enable_topic", "pid_enable");
        this->declare_parameter<std::string>("pid_debug_topic", "pid_debug");
        this->declare_parameter<double>("rate", 10);
        this->declare_parameter<double>("setpoint_timeout", -1.0);
        // Angle Limiting Parameters
        this->declare_parameter<bool>("angle_error", false);
        this->declare_parameter<double>("angle_wrap", 2.0 * 3.14159);
        // Create ReEntrant Callback group for multi-threaded recieving
        cb_in_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Get parameters
        this->get_parameter("Kp", Kp_);
        this->get_parameter("Ki", Ki_);
        this->get_parameter("Kd", Kd_);
        this->get_parameter("upper_limit", upper_limit_);
        this->get_parameter("topic_from_controller", topic_from_controller_);
        this->get_parameter("topic_from_plant", topic_from_plant_);
        this->get_parameter("setpoint_topic", setpoint_topic_);
        this->get_parameter("pid_enable_topic", pid_enable_topic_);
        this->get_parameter("pid_debug_topic", pid_debug_pub_name_);
        this->get_parameter("rate", rate_);

        this->get_parameter("upper_limit", upper_limit_);
        this->get_parameter("lower_limit", lower_limit_);
        this->get_parameter("windup_limit", windup_limit_);
        this->get_parameter("cutoff_frequency", cutoff_frequency_);
        this->get_parameter("angle_error", angle_error_);
        this->get_parameter("angle_wrap", angle_wrap_);
        // More parameters ...
        printParamters();

        // Instantiate publishers & subscribers
        control_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_from_controller_, 10);
        pid_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pid_debug_pub_name_ , 10);

        plant_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            this->topic_from_plant_, 10, std::bind(&PidObject::plantStateCallback, this, std::placeholders::_1));
        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            this->setpoint_topic_, 10, std::bind(&PidObject::setpointCallback, this, std::placeholders::_1));
        pid_enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            this->pid_enable_topic_, 10, std::bind(&PidObject::pidEnableCallback, this, std::placeholders::_1));
    
        if (!plant_sub_ || !setpoint_sub_ || !pid_enabled_sub_)
        {
            RCLCPP_FATAL(this->get_logger(), "Subscription failure has occured. Exiting");
            rclcpp::shutdown();
        }

        rclcpp::MessageInfo msg_info;
        while(!setpoint_sub_->take(state_msg_, msg_info ) ) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 800, "Waiting for first setpoint message");
        }

        while(!plant_sub_->take(plant_msg, msg_info)){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 800, "Waiting for first plant message");
        }

        // Create walltimer for publishing
        calc_timer_ = this->create_wall_timer( std::chrono::milliseconds(int64_t((1.0 / rate_) * 1000)), std::bind(&PidObject::doCalcs, this), cb_in_);
    }

void PidObject::printParamters()
{
    RCLCPP_WARN(this->get_logger(), "Setpoint Topic: %s", setpoint_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "PID Debug Topic: %s", pid_debug_pub_name_.c_str());
    RCLCPP_WARN(this->get_logger(), "Plant Sub Topic: %s", topic_from_plant_.c_str());
    RCLCPP_WARN(this->get_logger(), "Setpoint Topic: %s", setpoint_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "Plant Sub Topic: %s", pid_enable_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "Rate is set to %f. Calculated to be %ld", rate_, int64_t((1.0 / rate_) * 1000));

}

// ** ROS Callback Functions **/
void PidObject::plantStateCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    plant_state_ = msg->data;
    new_state_or_setpt_ = true;
}

void PidObject::setpointCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    setpoint_ = msg->data;
    last_setpoint_msg_time_ = this->now();
    new_state_or_setpt_ = true;
}

void PidObject::pidEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    pid_enabled_ = msg->data;
    if (pid_enabled_)
    {
        // Cancel PID Publication
        calc_timer_->reset();
        // Rest of enable handling
    } else {
        // Reset Publication
        calc_timer_->cancel();
        // Rest of disable handling
    }
}

//** PID Logic Functions **//
void PidObject::doCalcs()
{
    if (new_state_or_setpt_)
    {
        // All 3 gains should have the same sign
        if (!((Kp_ <= 0. && Ki_ <= 0. && Kd_ <= 0.) || (Kp_ >= 0. && Ki_ >= 0. && Kd_ >= 0.)))  
            RCLCPP_WARN(this->get_logger(), "All three gains must have same sign for stability!");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No new info, skipping iteration");
        return;
    } 
    
    // Propogate Error
    error_.at(2) = error_.at(1);
    error_.at(1) = error_.at(0);
    error_.at(0) = setpoint_ - plant_state_;

    // If the angle_error param is true, then address discontinuity in error calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_)
    {
        while (error_.at(0) < -1.0 * angle_wrap_ / 2.0)
        {
            error_.at(0) += angle_wrap_;

            // The proportional error will flip sign, but the integral error
            // won't and the filtered derivative will be poorly defined. So,
            // reset them.
            error_deriv_.at(2) = 0.;
            error_deriv_.at(1) = 0.;
            error_deriv_.at(0) = 0.;
            error_integral_ = 0.;
        }

        while (error_.at(0) > angle_wrap_ / 2.0)
        {
          error_.at(0) -= angle_wrap_;

          // The proportional error will flip sign, but the integral error
          // won't and the filtered derivative will be poorly defined. So,
          // reset them.
          error_deriv_.at(2) = 0.;
          error_deriv_.at(1) = 0.;
          error_deriv_.at(0) = 0.;
          error_integral_ = 0.;
        }
    }
    
    // Calculate Delta_T
    if( prev_time_.seconds() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Previous time is 0, doing nothing");
        prev_time_ = this->get_clock().get()->now();
        return;
    } 

    // Iterating time
    delta_t_ = this->get_clock().get()->now().nanoseconds() - prev_time_.nanoseconds();
    prev_time_ = this->get_clock().get()->now();

    // Checking if delta_t is 0 
    if (delta_t_ == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Delta_T is 0 skipping iteration");
        return;
    }

    // Intergrate Error
    error_integral_ += error_.at(0)*(delta_t_/10e9);
    
    if (error_integral_ > fabsf(windup_limit_))     // Check windup integral
      error_integral_ = fabsf(windup_limit_);       // Limit the size of integral
    
    if (error_integral_ < -fabsf(windup_limit_))
      error_integral_ = -fabsf(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
    if (cutoff_frequency_ != -1)
    {
        // Check if tan(_) is really small, could cause c = NaN
        tan_filt_ = tan((cutoff_frequency_ * 6.2832) * (delta_t_/10e9) / 2);
        // Avoid tan(0) ==> NaN
        if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
            tan_filt_ = -0.01;
        if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
            tan_filt_ = 0.01;

        c_ = 1 / tan_filt_;
    }

    filtered_error_.at(2) = filtered_error_.at(1);
    filtered_error_.at(1) = filtered_error_.at(0);
    filtered_error_.at(0) = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                (c_ * c_ - 1.414 * c_ + 1) * filtered_error_.at(2) - (-2 * c_ * c_ + 2) * filtered_error_.at(1));

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_.at(2) = error_deriv_.at(1);
    error_deriv_.at(1) = error_deriv_.at(0);
    error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / (delta_t_/10e9);

    filtered_error_deriv_.at(2) = filtered_error_deriv_.at(1);
    filtered_error_deriv_.at(1) = filtered_error_deriv_.at(0);
    filtered_error_deriv_.at(0) =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_.at(2) + 2 * error_deriv_.at(1) + error_deriv_.at(0) -
        (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_.at(2) - (-2 * c_ * c_ + 2) * filtered_error_deriv_.at(1));

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_.at(0);
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_.at(0);
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;

    if (pid_enabled_)
    {
        control_msg_.data = control_effort_;
        control_effort_pub_.get()->publish(control_msg_);
        
        //If Debug enabled
        if (pid_debug_pub_)
        {
            // Publish PID Debug Data
            std::vector<double> pid_debug_vect { plant_state_, control_effort_, proportional_, integral_, derivative_};
            std_msgs::msg::Float64MultiArray pidDebugMsg;
            pidDebugMsg.data = pid_debug_vect;
            pid_debug_pub_.get()->publish(pidDebugMsg);
        } else {
            error_integral_ = 0.0;
        }

    }
    // End 
    new_state_or_setpt_ = false;
}   

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidObject>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}