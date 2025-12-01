#include "minha_ponderada/pathfinder.hpp"

void Pathfinder::setMap(const std::vector<std::vector<char>>& map,
                        int rows, int cols,
                        int robotR, int robotC,
                        int targetR, int targetC)
{
    map_ = map;
    rows_ = rows;
    cols_ = cols;
    rStart_ = robotR;
    cStart_ = robotC;
    rGoal_ = targetR;
    cGoal_ = targetC;
}

Pathfinder::Path Pathfinder::runBFS()
{
    std::queue<std::pair<int,int>> q;
    q.push({rStart_, cStart_});

    std::vector<std::vector<bool>> visited(rows_, std::vector<bool>(cols_, false));
    visited[rStart_][cStart_] = true;

    std::vector<std::vector<std::pair<int,int>>> parent(
        rows_, std::vector<std::pair<int,int>>(cols_, {-1,-1})
    );

    int dr[4] = {-1,1,0,0};
    int dc[4] = {0,0,-1,1};

    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();

        if (r == rGoal_ && c == cGoal_) {
            return reconstructPath(parent);
        }

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_) continue;
            if (visited[nr][nc]) continue;

            char cell = map_[nr][nc];
            if (!(cell == 'f' || cell == 'r' || cell == 't')) continue;

            visited[nr][nc] = true;
            parent[nr][nc] = {r,c};
            q.push({nr,nc});
        }
    }

    return {}; 
}

Pathfinder::Path Pathfinder::reconstructPath(const std::vector<std::vector<std::pair<int,int>>>& parent)
{
    Path path;
    int r = rGoal_, c = cGoal_;
    while (!(r == rStart_ && c == cStart_)) {
        path.push_back({r,c});
        auto p = parent[r][c];
        r = p.first;
        c = p.second;
    }
    path.push_back({rStart_, cStart_});
    std::reverse(path.begin(), path.end());
    return path;
}
