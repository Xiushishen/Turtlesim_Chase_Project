#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"), turtle_counter_(0)
    {
        turtle_name_prefix_ = "turtle";
        alive_turtles_publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);
        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));
        publish_alive_turtles_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TurtleSpawnerNode::publishAliveTurtles, this));
        
        catch_turtle_service_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));
    }
private:
    

    void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawnerNode::callKillTurtleService, this, request->name)));
        response->success = true;
    }

    void callKillTurtleService(std::string turtle_name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            future.get();
            for (int i = 0; i < (int)alive_turtles_.size(); i++)
            {
                if (alive_turtles_[i].name == turtle_name)
                {
                    alive_turtles_.erase(alive_turtles_.begin() + i);
                    publishAliveTurtles();
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "Service call failed");
        }
    }

    void publishAliveTurtles()
    {
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    void spawnNewTurtle()
    {
        turtle_counter_++;
        auto name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        double x = randomDouble() * 10.0;
        double y = randomDouble() * 10.0;
        double theta = randomDouble() * 2 * M_PI;

        spawn_turtle_threads_.push_back(
            std::make_shared<std::thread>(
            std::bind(&TurtleSpawnerNode::callSpawnTurtleService, this, x, y, theta)));
    }   

    void callSpawnTurtleService(double x, double y, double theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (response->name != "")
            {
                auto new_turtle = my_robot_interfaces::msg::Turtle();
                new_turtle.name = response->name;
                new_turtle.x = x;
                new_turtle.y = y;
                new_turtle.theta = theta;
                alive_turtles_.push_back(new_turtle);
                publishAliveTurtles();
                RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", response->name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "Service call failed");
        }
    }

    std::string turtle_name_prefix_;
    int turtle_counter_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;

    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr publish_alive_turtles_timer_;


    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service_;
};
int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}