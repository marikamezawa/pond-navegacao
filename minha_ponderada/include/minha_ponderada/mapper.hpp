#ifndef MINHA_PONDERADA__MAPPER_HPP
#define MINHA_PONDERADA__MAPPER_HPP

#include <memory>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "minha_ponderada/robot_mover.hpp"  

class Mapper : public rclcpp::Node
{
public:
    using Position = std::pair<int, int>;
    using Map2D = std::vector<std::vector<char>>;

    Mapper();
    void startMapping(int startR, int startC, int rows, int cols);
    
    const Map2D& getDiscoveredMap() const { return discoveredMap_; }
    Position getRobotPosition() const { return {currentR_, currentC_}; }
    Position getTargetPosition() const { return targetPos_; }
    bool isTargetFound() const { return targetFound_; }
    
    void printMapMatrix() const;

private:
    void exploreRecursive();
    void updateMapAroundWithSensor();
    void sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg);
    bool moveToPosition(int targetR, int targetC);
    void waitForSensorData();

    std::shared_ptr<RobotMover> mover_;  // ← Usar RobotMover
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensorSub_;

    Map2D discoveredMap_;
    std::set<Position> visited_;
    
    int rows_, cols_;
    int currentR_, currentC_;
    
    Position targetPos_;
    bool targetFound_;
    bool sensorDataReceived_;
    
    cg_interfaces::msg::RobotSensors::SharedPtr lastSensorData_;
};

#endif