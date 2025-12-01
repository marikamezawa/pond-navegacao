#ifndef MINHA_PONDERADA__MAP_LOADER_HPP
#define MINHA_PONDERADA__MAP_LOADER_HPP

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"

class MapLoader : public rclcpp::Node
{
public:
    MapLoader();

    void requestMap();

    int getRows() const { return rows_; }
    int getCols() const { return cols_; }

    const std::vector<std::vector<char>>& getMap() const { return map2D_; }

    int getRobotR() const { return robotR_; }
    int getRobotC() const { return robotC_; }
    int getTargetR() const { return targetR_; }
    int getTargetC() const { return targetC_; }

    bool isMapReady() const { return mapReady_; }

private:
    void onMapReceived(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future);

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;

    std::vector<std::vector<char>> map2D_;
    int rows_ = 0;
    int cols_ = 0;
    int robotR_ = -1;
    int robotC_ = -1;
    int targetR_ = -1;
    int targetC_ = -1;
    bool mapReady_ = false;
};

#endif
