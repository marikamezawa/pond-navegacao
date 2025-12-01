#ifndef MINHA_PONDERADA__PATHFINDER_HPP
#define MINHA_PONDERADA__PATHFINDER_HPP

#include <vector>
#include <utility>
#include <queue>
#include <algorithm>

class Pathfinder {
public:
    using Path = std::vector<std::pair<int,int>>;

    Pathfinder() = default;

    void setMap(const std::vector<std::vector<char>>& map,
                int rows, int cols,
                int robotR, int robotC,
                int targetR, int targetC);

    Path runBFS();

private:
    std::vector<std::vector<char>> map_;
    int rows_, cols_;
    int rStart_, cStart_;
    int rGoal_, cGoal_;

    Path reconstructPath(const std::vector<std::vector<std::pair<int,int>>>& parent);
};
#endif
