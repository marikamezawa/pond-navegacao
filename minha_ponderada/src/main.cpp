#include "rclcpp/rclcpp.hpp"
#include "minha_ponderada/map_loader.hpp"
#include "minha_ponderada/pathfinder.hpp"
#include "minha_ponderada/robot_mover.hpp"
#include "minha_ponderada/mapper.hpp"
#include <iostream>

void runPart1()
{
    std::cout << "\n╔═══════════════════════════════════════════╗\n";
    std::cout << "║      PARTE 1: NAVEGAÇÃO COM MAPA (BFS)   ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n\n";

    auto mapLoader = std::make_shared<MapLoader>();
    mapLoader->requestMap();

    while (!mapLoader->isMapReady() && rclcpp::ok()) {
        rclcpp::spin_some(mapLoader);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!mapLoader->isMapReady()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Falha ao carregar mapa!");
        return;
    }

    int rows = mapLoader->getRows(), cols = mapLoader->getCols();
    int robotR = mapLoader->getRobotR(), robotC = mapLoader->getRobotC();
    int targetR = mapLoader->getTargetR(), targetC = mapLoader->getTargetC();

    std::cout << "✅ Mapa: " << rows << "x" << cols << " | Robô: (" << robotR << "," << robotC 
              << ") | Alvo: (" << targetR << "," << targetC << ")\n\n";

    Pathfinder pathfinder;
    pathfinder.setMap(mapLoader->getMap(), rows, cols, robotR, robotC, targetR, targetC);
    
    auto path = pathfinder.runBFS();
    if (path.empty()) {
        std::cout << "❌ Nenhum caminho encontrado!\n";
        return;
    }

    std::cout << "✅ Caminho (" << path.size() << " passos): ";
    for (size_t i = 0; i < path.size(); ++i) {
        auto [r, c] = path[i];
        std::cout << "(" << r << "," << c << ")";
        if (i < path.size() - 1) std::cout << " → ";
    }
    std::cout << "\n\n🚀 Executando...\n";

    auto mover = std::make_shared<RobotMover>();
    mover->setPath(path);
    mover->executePath();

    std::cout << "\n🎉 Robô chegou ao alvo!\n";
}

void runPart2()
{
    std::cout << "\n╔═══════════════════════════════════════════╗\n";
    std::cout << "║   PARTE 2: MAPEAMENTO COM DFS + SENSORES  ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n\n";

    auto mapLoader = std::make_shared<MapLoader>();
    mapLoader->requestMap();

    while (!mapLoader->isMapReady() && rclcpp::ok()) {
        rclcpp::spin_some(mapLoader);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!mapLoader->isMapReady()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Falha ao obter informações!");
        return;
    }

    int rows = mapLoader->getRows(), cols = mapLoader->getCols();
    int robotR = mapLoader->getRobotR(), robotC = mapLoader->getRobotC();

    std::cout << "📍 Labirinto: " << rows << "x" << cols << " | Início: (" 
              << robotR << "," << robotC << ")\n\n";

    auto mapper = std::make_shared<Mapper>();
    std::cout << "🗺️  Iniciando mapeamento DFS...\n\n";
    
    mapper->startMapping(robotR, robotC, rows, cols);
    
    std::cout << "\n✅ Mapeamento concluído! Exibindo mapa descoberto:\n";
    mapper->printMapMatrix();

    if (mapper->isTargetFound()) {
        auto targetPos = mapper->getTargetPosition();
        auto robotPos = mapper->getRobotPosition();
        
        std::cout << "🎯 Alvo: (" << targetPos.first << "," << targetPos.second 
                  << ") | Robô: (" << robotPos.first << "," << robotPos.second << ")\n\n";

        Pathfinder pathfinder;
        pathfinder.setMap(mapper->getDiscoveredMap(), rows, cols,
                         robotPos.first, robotPos.second,
                         targetPos.first, targetPos.second);
        
        auto path = pathfinder.runBFS();
        
        if (!path.empty()) {
            std::cout << "✅ Caminho até alvo (" << path.size() << " passos)\n";
            auto mover = std::make_shared<RobotMover>();
            mover->setPath(path);
            mover->executePath();
            std::cout << "\n🎉 Robô chegou ao alvo!\n";
        } else {
            std::cout << "❌ Caminho não encontrado!\n";
        }
    }

    std::cout << "\n╔═══════════════════════════════════════════╗\n";
    std::cout << "║          MAPEAMENTO FINALIZADO            ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n\n";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::cout << "\n╔═══════════════════════════════════════════╗\n";
    std::cout << "║         PONDERADA - NAVEGAÇÃO AUTÔNOMA    ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n\n";
    std::cout << "Escolha: [1] Parte 1 (BFS) | [2] Parte 2 (DFS+Sensores)\n";
    std::cout << "Opção: ";

    int choice;
    std::cin >> choice;

    if (choice == 1) runPart1();
    else if (choice == 2) runPart2();
    else std::cout << "\n❌ Opção inválida!\n";

    rclcpp::shutdown();
    return 0;
}