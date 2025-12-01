#include "minha_ponderada/mapper.hpp"
#include <iostream>
#include <stack>

Mapper::Mapper()
: Node("mapper"), 
  targetFound_(false),
  sensorDataReceived_(false)
{
    // Criar RobotMover para reutilizar código de movimento
    mover_ = std::make_shared<RobotMover>();
    
    sensorSub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", 10,
        std::bind(&Mapper::sensorCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Mapper inicializado!");
}

void Mapper::sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg)
{
    lastSensorData_ = msg;
    sensorDataReceived_ = true;
}

void Mapper::waitForSensorData()
{
    sensorDataReceived_ = false;
    auto start = std::chrono::steady_clock::now();
    
    while (!sensorDataReceived_ && rclcpp::ok()) {
        rclcpp::spin_some(this->shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        if (std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count() > 2) {
            RCLCPP_WARN(this->get_logger(), "Timeout aguardando sensor");
            break;
        }
    }
}

void Mapper::updateMapAroundWithSensor()
{
    if (!lastSensorData_) return;
    
    // Mapeamento das 8 direções
    struct { std::string field; int dr, dc; } mappings[] = {
        {"up", -1, 0}, {"down", 1, 0}, {"left", 0, -1}, {"right", 0, 1},
        {"up_left", -1, -1}, {"up_right", -1, 1}, {"down_left", 1, -1}, {"down_right", 1, 1}
    };
    
    for (const auto& m : mappings) {
        int nr = currentR_ + m.dr;
        int nc = currentC_ + m.dc;
        
        if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_ || discoveredMap_[nr][nc] != '?') 
            continue;
        
        std::string val;
        if (m.field == "up") val = lastSensorData_->up;
        else if (m.field == "down") val = lastSensorData_->down;
        else if (m.field == "left") val = lastSensorData_->left;
        else if (m.field == "right") val = lastSensorData_->right;
        else if (m.field == "up_left") val = lastSensorData_->up_left;
        else if (m.field == "up_right") val = lastSensorData_->up_right;
        else if (m.field == "down_left") val = lastSensorData_->down_left;
        else if (m.field == "down_right") val = lastSensorData_->down_right;
        
        if (val == "b" || val == "blocked" || val == "wall") {
            discoveredMap_[nr][nc] = 'b';
        } else if (val == "t" || val == "target") {
            discoveredMap_[nr][nc] = 't';
            targetPos_ = {nr, nc};
            targetFound_ = true;
            RCLCPP_INFO(this->get_logger(), "🎯 Alvo detectado em (%d,%d)!", nr, nc);
        } else if (val == "f" || val == "free" || val == "empty") {
            discoveredMap_[nr][nc] = 'f';
        }
    }
}

void Mapper::startMapping(int startR, int startC, int rows, int cols)
{
    rows_ = rows;
    cols_ = cols;
    currentR_ = startR;
    currentC_ = startC;
    
    discoveredMap_.assign(rows_, std::vector<char>(cols_, '?'));
    discoveredMap_[currentR_][currentC_] = 'f';
    visited_.insert({currentR_, currentC_});
    
    RCLCPP_INFO(this->get_logger(), "Iniciando mapeamento DFS em (%d,%d)", startR, startC);
    
    waitForSensorData();
    updateMapAroundWithSensor();
    exploreRecursive();
    
    RCLCPP_INFO(this->get_logger(), "Mapeamento completo!");
}

void Mapper::exploreRecursive()
{
    std::stack<Position> stack;
    stack.push({currentR_, currentC_});
    
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};
    std::string dirs[] = {"up", "down", "left", "right"};
    
    while (!stack.empty() && rclcpp::ok()) {
        auto [r, c] = stack.top();
        
        if (currentR_ != r || currentC_ != c) {
            if (!moveToPosition(r, c)) {
                stack.pop();
                continue;
            }
        }
        
        waitForSensorData();
        updateMapAroundWithSensor();
        
        bool foundUnvisited = false;
        
        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i], nc = c + dc[i];
            
            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_ || visited_.count({nr, nc})) 
                continue;
            
            char cell = discoveredMap_[nr][nc];
            
            if (cell == 't' || cell == 'b') {
                visited_.insert({nr, nc});
                continue;
            }
            
            if (cell == 'f' || cell == '?') {
                // Usar RobotMover para movimento individual
                std::vector<Position> singleStep = {{currentR_, currentC_}, {nr, nc}};
                mover_->setPath(singleStep);
                mover_->executePath();
                
                // Verificar se movimento foi bem-sucedido
                // (se executePath não lançou erro, assumimos sucesso)
                currentR_ = nr;
                currentC_ = nc;
                if (cell == '?') discoveredMap_[nr][nc] = 'f';
                visited_.insert({nr, nc});
                stack.push({nr, nc});
                foundUnvisited = true;
                break;
            }
        }
        
        if (!foundUnvisited) stack.pop();
    }
}

bool Mapper::moveToPosition(int targetR, int targetC)
{
    std::queue<Position> q;
    std::map<Position, Position> parent;
    std::set<Position> visitedBFS;
    
    q.push({currentR_, currentC_});
    visitedBFS.insert({currentR_, currentC_});
    
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};
    
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();
        
        if (r == targetR && c == targetC) {
            // Reconstruir caminho
            std::vector<Position> path;
            Position curr = {targetR, targetC};
            
            while (curr.first != currentR_ || curr.second != currentC_) {
                path.push_back(curr);
                curr = parent[curr];
            }
            
            path.push_back({currentR_, currentC_});
            std::reverse(path.begin(), path.end());
            
            // Usar RobotMover para executar caminho completo
            mover_->setPath(path);
            mover_->executePath();
            
            currentR_ = targetR;
            currentC_ = targetC;
            return true;
        }
        
        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i], nc = c + dc[i];
            
            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_ || 
                visitedBFS.count({nr, nc}) || discoveredMap_[nr][nc] != 'f') 
                continue;
            
            visitedBFS.insert({nr, nc});
            parent[{nr, nc}] = {r, c};
            q.push({nr, nc});
        }
    }
    
    return false;
}

void Mapper::printMapMatrix() const
{
    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                  MAPA 2D DO LABIRINTO                       ║\n";
    std::cout << "╠═════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Legenda: b=parede | f=livre | t=alvo | r=robô              ║\n";
    std::cout << "║  Dimensões: " << rows_ << "x" << cols_ << " células";
    // Pad para alinhar
    int pad = 42 - std::to_string(rows_).length() - std::to_string(cols_).length();
    for(int i = 0; i < pad; i++) std::cout << " ";
    std::cout << "║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            char cell = discoveredMap_[r][c];
            // Se é a posição atual do robô, mostra 'r'
            if (r == currentR_ && c == currentC_) {
                std::cout << "r ";
            } else {
                std::cout << cell << " ";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}