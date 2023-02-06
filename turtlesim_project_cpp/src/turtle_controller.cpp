#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using std::placeholders::_1;


class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller"), name_("turtle1"), turtlesim_up_(false)
    {
        catch_closest_turtle_first_ = true;
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(name_ + "/pose", 10,
                            std::bind(&TurtleControllerNode::callbackPose, this, _1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(name_ + "/cmd_vel", 10);
        control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
        std::bind(&TurtleControllerNode::controlLoop, this));

        turtles_to_catch_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10,
                            std::bind(&TurtleControllerNode::callbackTurtleToCatch, this, _1));

    }
private:
    void callbackPose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        pose_ = *pose.get();
        turtlesim_up_ = true;
    }
    
    void callbackTurtleToCatch(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
        {
            if (catch_closest_turtle_first_)
            {
                my_robot_interfaces::msg::Turtle closest_turtle = msg->turtles[0];
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);
                for (int i = 0; i < (int)msg->turtles.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->turtles[i]);
                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = msg->turtles[i];
                        closest_turtle_distance = distance;
                    }
                }
                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->turtles[0];
            }
        }
    }

    void publishCmdVel(double x, double theta)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = x;
        msg.angular.z = theta;
        cmd_vel_publisher_->publish(msg);
    }

    double getDistanceFromCurrentPose(my_robot_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void controlLoop()
    {
        if (!turtlesim_up_ || turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            // postion
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_2_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            catch_turtle_threads.push_back(
                std::make_shared<std::thread>(
                    std::bind(&TurtleControllerNode::callCatchTurtleService, this, turtle_to_catch_.name)));
            turtle_to_catch_.name = "";
        }
        cmd_vel_publisher_->publish(msg);
    }

    void callCatchTurtleService(std::string turtle_name)
    {
        auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            if (!response->success)
            {
                auto new_turtle = my_robot_interfaces::msg::Turtle();
                RCLCPP_INFO(this->get_logger(), "Failed to catch turtle");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "Service call failed");
        }
    }

    std::string name_;
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;
    bool catch_closest_turtle_first_;
    my_robot_interfaces::msg::Turtle turtle_to_catch_;
    std::vector<std::shared_ptr<std::thread>> catch_turtle_threads; 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtles_to_catch_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}