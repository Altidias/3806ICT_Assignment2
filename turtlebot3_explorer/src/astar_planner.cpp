#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "turtlebot3_explorer/PlanPath.h"
#include "turtlebot3_explorer/GetPathCosts.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <numeric>
#include "math_utils.hpp"

using namespace explorer_utils;

struct Node {
    int x, y;
    double g, h;
    int unknown_cnt;
    std::shared_ptr<Node> parent;
    
    Node(int x_, int y_) : x(x_), y(y_), g(0), h(0), 
                          unknown_cnt(0), parent(nullptr) {}
    double f() const { return g + h; }
};

struct NodeCmp {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        if (std::abs(a->f() - b->f()) < 0.1) {
            return a->unknown_cnt > b->unknown_cnt;
        }
        return a->f() > b->f();
    }
};

class Planner {
public:
    Planner(ros::NodeHandle& nh) : nh_(nh) {
        srv_ = nh_.advertiseService("plan_path", &Planner::plan, this);
        
        // params
        nh_.param("allow_unknown", allow_unk_, true);
        nh_.param("robot_radius", r_rad_, 0.15);
        nh_.param("safety_margin", margin_, 0.05);
        nh_.param("use_dynamic_heuristics", use_h_, false);
        nh_.param("heuristic_weight", h_weight_, 1.2);
        nh_.param("unknown_cost_factor", unk_cost_, 1.5);
        nh_.param("max_unknown_cells", max_unk_, 500);
        nh_.param("max_planning_attempts", max_attempts_, 3);
        
        // heuristic client if needed
        if (use_h_) {
            h_client_ = nh_.serviceClient<turtlebot3_explorer::GetPathCosts>("/heuristic_calculator/get_path_costs");
        }
        
        ROS_INFO("A* Planner ready");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer srv_;
    ros::ServiceClient h_client_;
    
    bool allow_unk_;
    double r_rad_;
    double margin_;
    bool use_h_;
    double h_weight_;
    double unk_cost_;
    int max_unk_;
    int max_attempts_;
    
    // 8-way dirs
    const std::vector<std::pair<int, int>> dirs_ = {
        {0,1}, {1,0}, {0,-1}, {-1,0},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    bool plan(turtlebot3_explorer::PlanPath::Request& req,
              turtlebot3_explorer::PlanPath::Response& res) {
        
        // validate input
        if (!valid(req)) {
            res.success = false;
            return true;
        }

        auto grid = to_grid(req);
        
        // try multiple times with relaxed params
        for (int attempt = 0; attempt < max_attempts_; ++attempt) {
            double current_unk_cost = unk_cost_ + (attempt * 0.5);
            int current_max_unk = max_unk_ + (attempt * 100);
            
            auto path = find_path(grid, req.start_x, req.start_y, req.goal_x, req.goal_y, 
                                current_unk_cost, current_max_unk);
            
            if (!path.first.empty()) {
                // process and smooth path
                path = process(path, grid);
                auto smooth = smooth_path(path, grid);
                
                res.path_x = smooth.first;
                res.path_y = smooth.second;
                res.success = true;
                return true;
            }
        }
        
        // desperate attempt
        auto desperate = find_desperate(grid, req.start_x, req.start_y, req.goal_x, req.goal_y);
        
        if (!desperate.first.empty()) {
            res.path_x = desperate.first;
            res.path_y = desperate.second;
            res.success = true;
            return true;
        }
        
        res.success = false;
        return true;
    }

    bool valid(const turtlebot3_explorer::PlanPath::Request& req) {
        if (req.map_data.size() != req.map_width * req.map_height) {
            return false;
        }
        
        // check bounds
        if (req.start_x < 0 || req.start_x >= (int)req.map_width ||
            req.start_y < 0 || req.start_y >= (int)req.map_height ||
            req.goal_x < 0 || req.goal_x >= (int)req.map_width ||
            req.goal_y < 0 || req.goal_y >= (int)req.map_height) {
            return false;
        }
        
        return true;
    }

    std::vector<std::vector<int>> to_grid(const turtlebot3_explorer::PlanPath::Request& req) {
        std::vector<std::vector<int>> grid(req.map_height, std::vector<int>(req.map_width));
        
        // convert occupancy data
        for (size_t y = 0; y < req.map_height; ++y) {
            for (size_t x = 0; x < req.map_width; ++x) {
                int val = req.map_data[y * req.map_width + x];
                
                if (val == -1) {  // unknown
                    grid[y][x] = 2;
                } else if (val < 65) {  // free
                    grid[y][x] = 0;
                } else {  // occupied
                    grid[y][x] = 1;
                }
            }
        }
        
        return inflate(grid);
    }

    std::vector<std::vector<int>> inflate(const std::vector<std::vector<int>>& grid) {
        int h = grid.size();
        int w = grid[0].size();
        
        double cell_res = 0.1;
        int inf_cells = std::max(1, (int)std::ceil((r_rad_ + margin_) / cell_res));
        inf_cells = std::min(inf_cells, 3);  // cap at 3
        
        std::vector<std::vector<int>> inflated = grid;
        
        // inflate obstacles
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (grid[y][x] == 1) {
                    for (int dy = -inf_cells; dy <= inf_cells; ++dy) {
                        for (int dx = -inf_cells; dx <= inf_cells; ++dx) {
                            int ny = y + dy;
                            int nx = x + dx;
                            
                            if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
                                double dist = std::sqrt(dx*dx + dy*dy) * cell_res;
                                
                                if (dist <= r_rad_ + margin_ && inflated[ny][nx] != 1 && inflated[ny][nx] != 2) {
                                    inflated[ny][nx] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        return inflated;
    }

    std::pair<std::vector<int>, std::vector<int>> find_path(
        const std::vector<std::vector<int>>& grid,
        int sx, int sy, int gx, int gy,
        double unk_cost_factor, int max_unknown) {
        
        int h = grid.size();
        int w = grid[0].size();
        
        // check start/goal
        if (grid[sy][sx] == 1) {
            return {{}, {}};
        }
        
        if (grid[gy][gx] == 1) {
            // find nearby free cell
            for (int radius = 1; radius <= 3; ++radius) {
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int new_gx = gx + dx;
                        int new_gy = gy + dy;
                        
                        if (new_gx >= 0 && new_gx < w && new_gy >= 0 && new_gy < h && 
                            grid[new_gy][new_gx] != 1) {
                            gx = new_gx;
                            gy = new_gy;
                            goto goal_found;
                        }
                    }
                }
            }
            return {{}, {}};
        }
        
        goal_found:
        
        std::priority_queue<std::shared_ptr<Node>, 
                           std::vector<std::shared_ptr<Node>>, 
                           NodeCmp> open;
        
        std::vector<std::vector<double>> best_g(h, std::vector<double>(w, INFINITY));
        
        auto start = std::make_shared<Node>(sx, sy);
        start->h = euclideanDistance(sx, sy, gx, gy) * h_weight_;
        open.push(start);
        best_g[sy][sx] = 0;
        
        int iterations = 0;
        const int max_iterations = w * h;
        
        while (!open.empty() && iterations < max_iterations) {
            iterations++;
            
            auto cur = open.top();
            open.pop();
            
            // skip if found better
            if (cur->g > best_g[cur->y][cur->x] + 0.001) {
                continue;
            }
            
            // reached goal?
            if (cur->x == gx && cur->y == gy) {
                return reconstruct(cur);
            }
            
            // too many unknowns?
            if (cur->unknown_cnt > max_unknown) {
                continue;
            }
            
            // check neighbors
            for (size_t i = 0; i < dirs_.size(); ++i) {
                int nx = cur->x + dirs_[i].first;
                int ny = cur->y + dirs_[i].second;
                
                if (!valid_move(grid, cur->x, cur->y, nx, ny)) continue;
                
                double cost = (i < 4) ? 1.0 : 1.414;
                
                int cell = grid[ny][nx];
                if (cell == 1) continue;  // obstacle
                
                int new_unk = cur->unknown_cnt;
                if (cell == 2) {  // unknown
                    if (!allow_unk_) continue;
                    cost *= unk_cost_factor;
                    new_unk++;
                }
                
                double new_g = cur->g + cost;
                
                if (new_g < best_g[ny][nx]) {
                    best_g[ny][nx] = new_g;
                    
                    auto neighbor = std::make_shared<Node>(nx, ny);
                    neighbor->g = new_g;
                    neighbor->h = euclideanDistance(nx, ny, gx, gy) * h_weight_;
                    neighbor->unknown_cnt = new_unk;
                    neighbor->parent = cur;
                    
                    open.push(neighbor);
                }
            }
        }
        
        return {{}, {}};
    }

    std::pair<std::vector<int>, std::vector<int>> find_desperate(
        const std::vector<std::vector<int>>& grid,
        int sx, int sy, int gx, int gy) {
        
        // super relaxed params
        return find_path(grid, sx, sy, gx, gy, 1.1, 1000);
    }

    bool valid_move(const std::vector<std::vector<int>>& grid, 
                    int x1, int y1, int x2, int y2) {
        // bounds check
        if (y2 < 0 || y2 >= (int)grid.size() || x2 < 0 || x2 >= (int)grid[0].size()) {
            return false;
        }
        
        if (grid[y2][x2] == 1) return false;
        
        // diagonal check
        int dx = x2 - x1;
        int dy = y2 - y1;
        
        if (std::abs(dx) == 1 && std::abs(dy) == 1) {
            if (grid[y1 + dy][x1] == 1 && grid[y1][x1 + dx] == 1) {
                return false;  // both adjacent blocked
            }
        }
        
        return true;
    }

    std::pair<std::vector<int>, std::vector<int>> process(
        const std::pair<std::vector<int>, std::vector<int>>& path,
        const std::vector<std::vector<int>>& grid) {
        
        if (path.first.size() < 2) return path;
        
        std::vector<int> px, py;
        
        // keep more waypoints
        px.push_back(path.first[0]);
        py.push_back(path.second[0]);
        
        for (size_t i = 1; i < path.first.size() - 1; ++i) {
            px.push_back(path.first[i]);
            py.push_back(path.second[i]);
        }
        
        px.push_back(path.first.back());
        py.push_back(path.second.back());
        
        return {px, py};
    }

    std::pair<std::vector<int>, std::vector<int>> smooth_path(
        const std::pair<std::vector<int>, std::vector<int>>& path,
        const std::vector<std::vector<int>>& grid) {
        
        if (path.first.size() < 3) return path;
        
        std::vector<int> sx, sy;
        sx.push_back(path.first[0]);
        sy.push_back(path.second[0]);
        
        size_t cur = 0;
        
        while (cur < path.first.size() - 1) {
            size_t far = cur + 1;
            
            // line of sight check
            for (size_t i = cur + 2; i < std::min(cur + 6, path.first.size()); ++i) {
                if (has_los(sx.back(), sy.back(), path.first[i], path.second[i], grid)) {
                    far = i;
                } else {
                    break;
                }
            }
            
            sx.push_back(path.first[far]);
            sy.push_back(path.second[far]);
            cur = far;
        }
        
        return {sx, sy};
    }

    bool has_los(int x1, int y1, int x2, int y2, 
                 const std::vector<std::vector<int>>& grid) {
        // bresenham line check
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int x = x1, y = y1;

        int xinc = (x2 > x1) ? 1 : -1;
        int yinc = (y2 > y1) ? 1 : -1;
        int err = dx - dy;

        while (x != x2 || y != y2) {
            if (y < 0 || y >= (int)grid.size() || 
                x < 0 || x >= (int)grid[0].size()) {
                return false;
            }

            if (grid[y][x] == 1) {
                return false;
            }

            int e2 = err * 2;
            if (e2 > -dy) {
                err -= dy;
                x += xinc;
            }
            if (e2 < dx) {
                err += dx;
                y += yinc;
            }
        }

        // check destination
        if (y2 >= 0 && y2 < (int)grid.size() && 
            x2 >= 0 && x2 < (int)grid[0].size()) {
            return grid[y2][x2] != 1;
        }

        return true;
    }

    std::pair<std::vector<int>, std::vector<int>> reconstruct(std::shared_ptr<Node> goal) {
        std::vector<int> px, py;
        auto cur = goal;
        
        while (cur) {
            px.push_back(cur->x);
            py.push_back(cur->y);
            cur = cur->parent;
        }
        
        std::reverse(px.begin(), px.end());
        std::reverse(py.begin(), py.end());
        
        return {px, py};
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh("~");
    
    Planner planner(nh);
    ros::spin();
    
    return 0;
}