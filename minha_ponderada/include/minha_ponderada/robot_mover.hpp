#ifndef MINHA_PONDERADA__ROBOT_MOVER_HPP
#define MINHA_PONDERADA__ROBOT_MOVER_HPP

#include <memory>
#include <vector>
#include <utility>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

class RobotMover : public rclcpp::Node
{
public:
    using Path = std::vector<std::pair<int,int>>;

    RobotMover();

    void setPath(const Path& path);
    void executePath();

private:
    std::string getDirection(int fromR, int fromC, int toR, int toC);
    void moveStep(const std::string& direction);
    void onMoveResponse(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future);

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    Path path_;
    size_t currentStep_;
    bool waitingForResponse_;
};

#endif