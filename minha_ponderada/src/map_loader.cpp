#include "minha_ponderada/map_loader.hpp"

MapLoader::MapLoader()
: Node("map_loader")
{
    client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
    client_->wait_for_service();
}

void MapLoader::requestMap()
{
    RCLCPP_INFO(this->get_logger(), "Chamando serviço /get_map...");

    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    client_->async_send_request(
        request,
        std::bind(&MapLoader::onMapReceived, this, std::placeholders::_1)
    );
}

void MapLoader::onMapReceived(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future)
{
    auto response = future.get();

    auto &flat = response->occupancy_grid_flattened;
    rows_ = response->occupancy_grid_shape[0];
    cols_ = response->occupancy_grid_shape[1];

    // Converter 1D para 2D
    map2D_.assign(rows_, std::vector<char>(cols_));

    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            map2D_[r][c] = flat[r * cols_ + c][0];
        }
    }

    // Encontrar posições do robô e alvo
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            if (map2D_[r][c] == 'r') { robotR_ = r; robotC_ = c; }
            if (map2D_[r][c] == 't') { targetR_ = r; targetC_ = c; }
        }
    }

    mapReady_ = true;

    RCLCPP_INFO(this->get_logger(), "Mapa carregado! Robô (%d,%d) - Alvo (%d,%d)", 
                robotR_, robotC_, targetR_, targetC_);
}
