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
    int unknown_cnt;  // track unknown cells
    std::shared_ptr<Node> parent;
    
    Node(int x_, int y_) : x(x_), y(y_), g(0), h(0), 
                          unknown_cnt(0), parent(nullptr) {}
    double f() const { return g + h; }
};

struct NodeCmp {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        // prefer fewer unknown when f similar
        if (std::abs(a->f() - b->f()) < 0.1) {
            return a->unknown_cnt > b->unknown_cnt;
        }
        return a->f() > b->f();
    }
};

class AStar {
public:
    AStar(ros::NodeHandle& nh) : nh_(nh) {
        srv_ = nh_.advertiseService("plan_path", &AStar::plan, this);
        
        // params
        nh_.param("allow_unknown", allow_unk_, true);
        nh_.param("robot_radius", r_rad_, 0.15);
        nh_.param("safety_margin", margin_, 0.05);
        nh_.param("use_dynamic_heuristics", use_h_, true);
        nh_.param("heuristic_weight", h_weight_, 1.5);
        nh_.param("unknown_cost_factor", unk_cost_, 3.0);
        nh_.param("max_unknown_cells", max_unk_, 30);
        
        // heuristic client
        if (use_h_) {
            h_client_ = nh_.serviceClient<turtlebot3_explorer::GetPathCosts>("/heuristic_calculator/get_path_costs");
            
            if (h_client_.waitForExistence(ros::Duration(5.0))) {
                ROS_INFO("heuristic service ready");
            } else {
                use_h_ = false;
            }
        }
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
    
    // 8-way movement
    const std::vector<std::pair<int, int>> dirs_ = {
        {0,1}, {1,0}, {0,-1}, {-1,0},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    bool plan(turtlebot3_explorer::PlanPath::Request& req,
              turtlebot3_explorer::PlanPath::Response& res) {
        
        // validate
        if (!valid_input(req)) {
            res.success = false;
            return true;
        }

        // convert
        auto grid = to_grid(req);
        
        // get heuristics if requested
        std::vector<std::vector<double>> h_costs;
        if (req.use_heuristics && !req.heuristic_costs.empty()) {
            h_costs = conv_h_costs(req);
        }
        
        // find path
        auto path = find_path(grid, req.start_x, req.start_y, req.goal_x, req.goal_y, h_costs);
        
        // retry if failed
        if (path.first.empty()) {
            auto old_cost = unk_cost_;
            auto old_max = max_unk_;
            
            unk_cost_ = 2.0;
            max_unk_ = 50;
            
            path = find_path(grid, req.start_x, req.start_y, req.goal_x, req.goal_y, h_costs);
            
            unk_cost_ = old_cost;
            max_unk_ = old_max;
        }
        
        if (path.first.empty()) {
            res.success = false;
            return true;
        }
        
        // process
        path = process_path(path, grid);
        
        // smooth
        auto smooth = smooth_path(path, grid);
        
        // apply heuristics
        if (use_h_ && req.use_heuristics) {
            auto refined = apply_h(req.robot_id, smooth, grid);
            if (!refined.first.empty()) {
                smooth = refined;
            }
        }
        
        res.path_x = smooth.first;
        res.path_y = smooth.second;
        res.success = true;
        
        return true;
    }

    bool valid_input(const turtlebot3_explorer::PlanPath::Request& req) {
        if (req.map_data.size() != req.map_width * req.map_height) {
            return false;
        }
        
        // bounds
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
        
        // convert occupancy
        for (size_t y = 0; y < req.map_height; ++y) {
            for (size_t x = 0; x < req.map_width; ++x) {
                int val = req.map_data[y * req.map_width + x];
                
                if (val == -1) {  // unknown
                    grid[y][x] = 2;
                } else if (val < 50) {  // free
                    grid[y][x] = 0;
                } else {  // occupied
                    grid[y][x] = 1;
                }
            }
        }
        
        // inflate
        grid = inflate(grid);
        
        return grid;
    }

    std::vector<std::vector<int>> inflate(const std::vector<std::vector<int>>& grid) {
        int h = grid.size();
        int w = grid[0].size();
        
        double cell_res = 0.1;
        
        // calc inflation radius
        int inf_cells = std::ceil(r_rad_ / cell_res);
        
        if (margin_ > 0.01) {
            inf_cells += std::ceil(margin_ / cell_res);
        }
        
        inf_cells = std::max(1, inf_cells);
        
        std::vector<std::vector<int>> inflated = grid;
        
        // inflate obstacles
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (grid[y][x] == 1) {
                    // circular inflate
                    for (int dy = -inf_cells; dy <= inf_cells; ++dy) {
                        for (int dx = -inf_cells; dx <= inf_cells; ++dx) {
                            int ny = y + dy;
                            int nx = x + dx;
                            
                            if (ny >= 0 && ny < h && nx >= 0 && nx < w) {
                                double dist = std::sqrt(dx*dx + dy*dy) * cell_res;
                                
                                if (dist <= r_rad_ + margin_ && inflated[ny][nx] != 1) {
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

    std::vector<std::vector<double>> conv_h_costs(const turtlebot3_explorer::PlanPath::Request& req) {
        std::vector<std::vector<double>> costs(req.map_height, std::vector<double>(req.map_width, 1.0));
        
        if (req.heuristic_costs.size() == req.map_width * req.map_height) {
            for (size_t y = 0; y < req.map_height; ++y) {
                for (size_t x = 0; x < req.map_width; ++x) {
                    double heat = req.heuristic_costs[y * req.map_width + x];
                    costs[y][x] = 1.0 + (heat * 0.5);
                }
            }
        }
        
        return costs;
    }

    std::pair<std::vector<int>, std::vector<int>> find_path(
        const std::vector<std::vector<int>>& grid,
        int sx, int sy, int gx, int gy,
        const std::vector<std::vector<double>>& h_costs = {}) {
        
        int h = grid.size();
        int w = grid[0].size();
        
        // priority queue
        std::priority_queue<std::shared_ptr<Node>, 
                           std::vector<std::shared_ptr<Node>>, 
                           NodeCmp> open;
        
        // best costs
        std::vector<std::vector<double>> best_g(h, std::vector<double>(w, INFINITY));
        
        // start
        auto start = std::make_shared<Node>(sx, sy);
        start->h = euclideanDistance(sx, sy, gx, gy) * h_weight_;
        open.push(start);
        best_g[sy][sx] = 0;
        
        while (!open.empty()) {
            auto cur = open.top();
            open.pop();
            
            // skip if found better
            if (cur->g > best_g[cur->y][cur->x] + 0.001) {
                continue;
            }
            
            // goal?
            if (cur->x == gx && cur->y == gy) {
                return reconstruct(cur);
            }
            
            // too many unknown?
            if (cur->unknown_cnt > max_unk_) {
                continue;
            }
            
            // explore neighbors
            for (size_t i = 0; i < dirs_.size(); ++i) {
                int nx = cur->x + dirs_[i].first;
                int ny = cur->y + dirs_[i].second;
                
                if (!valid_move(grid, cur->x, cur->y, nx, ny)) continue;
                
                // move cost
                double cost = (i < 4) ? 1.0 : 1.414;
                
                // check cell
                int cell = grid[ny][nx];
                if (cell == 1) continue;  // obstacle
                
                // unknown cost
                int new_unk = cur->unknown_cnt;
                if (cell == 2) {  // unknown
                    if (!allow_unk_) continue;
                    cost *= unk_cost_;
                    new_unk++;
                }
                
                // heuristic cost
                if (!h_costs.empty() && ny < (int)h_costs.size() && nx < (int)h_costs[ny].size()) {
                    cost *= h_costs[ny][nx];
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

    bool valid_move(const std::vector<std::vector<int>>& grid, 
                    int x1, int y1, int x2, int y2) {
        // bounds
        if (y2 < 0 || y2 >= (int)grid.size() || x2 < 0 || x2 >= (int)grid[0].size()) {
            return false;
        }
        
        // obstacle
        if (grid[y2][x2] == 1) return false;
        
        // diagonal check
        int dx = x2 - x1;
        int dy = y2 - y1;
        
        if (std::abs(dx) == 1 && std::abs(dy) == 1) {
            // no corner cutting
            if (grid[y1 + dy][x1] == 1 || grid[y1][x1 + dx] == 1) {
                return false;
            }
        }
        
        return true;
    }

    std::pair<std::vector<int>, std::vector<int>> process_path(
        const std::pair<std::vector<int>, std::vector<int>>& path,
        const std::vector<std::vector<int>>& grid) {
        
        if (path.first.size() < 2) return path;
        
        std::vector<int> px, py;
        
        // remove redundant
        px.push_back(path.first[0]);
        py.push_back(path.second[0]);
        
        for (size_t i = 1; i < path.first.size() - 1; ++i) {
            int dx1 = path.first[i] - path.first[i-1];
            int dy1 = path.second[i] - path.second[i-1];
            int dx2 = path.first[i+1] - path.first[i];
            int dy2 = path.second[i+1] - path.second[i];
            
            // keep if direction changes
            if (dx1 != dx2 || dy1 != dy2) {
                px.push_back(path.first[i]);
                py.push_back(path.second[i]);
            }
        }
        
        // add last
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
            
            // find furthest visible
            for (size_t i = cur + 2; i < path.first.size(); ++i) {
                if (has_los(sx.back(), sy.back(), path.first[i], path.second[i], grid)) {
                    far = i;
                } else {
                    break;
                }
            }
            
            // add intermediate if big jump
            double d = euclideanDistance(sx.back(), sy.back(),
                                       path.first[far], path.second[far]);
            
            if (d > 10 && far - cur > 2) {
                size_t mid = cur + (far - cur) / 2;
                sx.push_back(path.first[mid]);
                sy.push_back(path.second[mid]);
            }
            
            sx.push_back(path.first[far]);
            sy.push_back(path.second[far]);
            cur = far;
        }
        
        return {sx, sy};
    }

    bool has_los(int x1, int y1, int x2, int y2, 
                 const std::vector<std::vector<int>>& grid) {
        // conservative los check
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int x = x1, y = y1;

        int xinc = (x2 > x1) ? 1 : -1;
        int yinc = (y2 > y1) ? 1 : -1;
        int err = dx - dy;

        while (x != x2 || y != y2) {
            // check 3x3 area
            for (int cy = -1; cy <= 1; ++cy) {
                for (int cx = -1; cx <= 1; ++cx) {
                    int cx_ = x + cx;
                    int cy_ = y + cy;

                    if (cy_ < 0 || cy_ >= (int)grid.size() || 
                        cx_ < 0 || cx_ >= (int)grid[0].size()) {
                        return false;
                    }

                    if (grid[cy_][cx_] == 1) {
                        return false;
                    }
                }
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

        // check dest area
        for (int cy = -1; cy <= 1; ++cy) {
            for (int cx = -1; cx <= 1; ++cx) {
                int cx_ = x2 + cx;
                int cy_ = y2 + cy;

                if (cy_ >= 0 && cy_ < (int)grid.size() && 
                    cx_ >= 0 && cx_ < (int)grid[0].size()) {
                    if (grid[cy_][cx_] == 1) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    std::pair<std::vector<int>, std::vector<int>> apply_h(
        const std::string& rid,
        const std::pair<std::vector<int>, std::vector<int>>& path,
        const std::vector<std::vector<int>>& grid) {
        
        try {
            // get costs
            turtlebot3_explorer::GetPathCosts srv;
            srv.request.robot_id = rid;
            srv.request.path_x = path.first;
            srv.request.path_y = path.second;
            
            if (!h_client_.call(srv) || !srv.response.success) {
                return path;
            }
            
            // analyze
            double max_c = *std::max_element(srv.response.costs.begin(), srv.response.costs.end());
            double avg_c = std::accumulate(srv.response.costs.begin(), srv.response.costs.end(), 0.0) 
                          / srv.response.costs.size();
            
            // replan if high
            if (max_c > 2.0 || avg_c > 1.5) {
                // create cost map
                auto dyn = create_cost_map(path, srv.response.costs, 
                                          grid[0].size(), grid.size());
                
                // replan
                auto refined = find_path(grid, path.first[0], path.second[0],
                                       path.first.back(), path.second.back(), dyn);
                
                if (!refined.first.empty()) {
                    return refined;
                }
            }
            
        } catch (...) {
        }
        
        return path;
    }
    
    std::vector<std::vector<double>> create_cost_map(
        const std::pair<std::vector<int>, std::vector<int>>& path,
        const std::vector<double>& costs,
        int w, int h) {
        
        std::vector<std::vector<double>> cmap(h, std::vector<double>(w, 1.0));
        
        // spread costs
        for (size_t i = 0; i < path.first.size() && i < costs.size(); ++i) {
            int x = path.first[i];
            int y = path.second[i];
            double cost = costs[i];
            
            // gaussian
            int r = 3;
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                        double d = std::sqrt(dx*dx + dy*dy);
                        double inf = std::exp(-d * d / (2.0 * r * r));
                        cmap[ny][nx] = std::max(cmap[ny][nx], 
                                               1.0 + (cost - 1.0) * inf);
                    }
                }
            }
        }
        
        return cmap;
    }

    std::pair<std::vector<int>, std::vector<int>> reconstruct(std::shared_ptr<Node> goal) {
        std::vector<int> px, py;
        auto cur = goal;
        
        while (cur) {
            px.push_back(cur->x);
            py.push_back(cur->y);
            cur = cur->parent;
        }
        
        // reverse
        std::reverse(px.begin(), px.end());
        std::reverse(py.begin(), py.end());
        
        return {px, py};
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh("~");
    
    AStar planner(nh);
    ros::spin();
    
    return 0;
}