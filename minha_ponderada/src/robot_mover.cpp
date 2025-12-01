#include "minha_ponderada/robot_mover.hpp"

RobotMover::RobotMover()
: Node("robot_mover"), currentStep_(0), waitingForResponse_(false)
{
    client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    RCLCPP_INFO(this->get_logger(), "Aguardando serviço /move_command...");
    client_->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Serviço /move_command disponível!");
}

void RobotMover::setPath(const Path& path)
{
    path_ = path;
    currentStep_ = 0;
    RCLCPP_INFO(this->get_logger(), "Caminho definido com %zu passos", path_.size());
}

void RobotMover::executePath()
{
    if (path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Caminho vazio!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Iniciando execução do caminho...");

    // Começar do passo 1 (passo 0 é a posição inicial)
    currentStep_ = 1;

    while (currentStep_ < path_.size() && rclcpp::ok()) {
        auto [fromR, fromC] = path_[currentStep_ - 1];
        auto [toR, toC] = path_[currentStep_];

        std::string direction = getDirection(fromR, fromC, toR, toC);
        
        RCLCPP_INFO(this->get_logger(), "Passo %zu/%zu: (%d,%d) -> (%d,%d) [%s]",
                    currentStep_, path_.size() - 1,
                    fromR, fromC, toR, toC, direction.c_str());

        moveStep(direction);

        // Aguardar resposta
        waitingForResponse_ = true;
        while (waitingForResponse_ && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        currentStep_++;
    }

    RCLCPP_INFO(this->get_logger(), "Caminho completo executado!");
}

std::string RobotMover::getDirection(int fromR, int fromC, int toR, int toC)
{
    int dr = toR - fromR;
    int dc = toC - fromC;

    if (dr == -1 && dc == 0) return "up";
    if (dr == 1 && dc == 0) return "down";
    if (dr == 0 && dc == -1) return "left";
    if (dr == 0 && dc == 1) return "right";

    RCLCPP_ERROR(this->get_logger(), "Direção inválida: (%d,%d) -> (%d,%d)", 
                 fromR, fromC, toR, toC);
    return "up"; // fallback
}

void RobotMover::moveStep(const std::string& direction)
{
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;

    client_->async_send_request(
        request,
        std::bind(&RobotMover::onMoveResponse, this, std::placeholders::_1)
    );
}

void RobotMover::onMoveResponse(rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future)
{
    auto response = future.get();
    
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "✓ Movimento executado com sucesso");
    } else {
        RCLCPP_WARN(this->get_logger(), "✗ Falha no movimento");
    }

    waitingForResponse_ = false;
}