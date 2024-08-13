#include "rclcpp/rclcpp.hpp"
#include "jaco_teleop/srv/plan_path.hpp"


using namespace std::chrono_literals;

class JacoServoReal : public rclcpp::Node
{
    public:
        JacoServoReal() : Node("jaco_servo_real")
        {

            // CVreate the planpath service 
            plan_path_service_ = create_service<jaco_teleop::srv::PlanPath>("robot_waypoints", &waypoint_callback);

            // Create a timer that calls the servo function every 10ms
            timer_ = this->create_wall_timer(10ms, std::bind(&JacoServoReal::servo, this));
        }

    private:
        void waypoint_callback(const std::shared_ptr<jaco_teleop::srv::PlanPath::Request> request,
        std::shared_ptr<jaco_teleop::srv::PlanPath::Response> response)
        {
            linear_step_size = Eigen::Vector3d{
            request->waypoint.pose.position.x,
            request->waypoint.pose.position.y,
            request->waypoint.pose.position.z};

            angular_step_size = Eigen::AngleAxisd(0.001, Eigen::Vector3d::UnitY());
        }

        void servo()
        {
            // Check if there is a request
            if (request_ == nullptr)
            {
                return;
            }

            // Check if the request is empty
            if (request_->path.poses.size() == 0)
            {
                return;
            }

            // Get the first pose
            auto pose = request_->path.poses.front();

            // Print the pose
            RCLCPP_INFO(this->get_logger(), "Servoing to (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);

            // Remove the first pose
            request_->path.poses.erase(request_->path.poses.begin());
        }

        rclcpp::Service<PlanPath>::SharedPtr plan_path_service_;
        rclcpp::TimerBase::SharedPtr timer_;
        PlanPath::Request::SharedPtr request_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JacoServoReal>());
    rclcpp::shutdown();
    return 0;
}