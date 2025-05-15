#include <cmath>
#include <algorithm> 

#include "planner_core.hpp"

namespace robot
{

    PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
    : logger_(logger) {}

    void PlannerCore::plan(const nav_msgs::msg::OccupancyGrid &map, CellIndex &goal, CellIndex &home) {
        global_map = map;
        goal_cell = goal;
        home_cell = home;
        res = global_map.info.resolution;

        path = nav_msgs::msg::Path();
        path.header.frame_id = "sim_world";
        
        this->aStar();
    }

    void PlannerCore::aStar() {
        // initialize the algorithm 
        open_set = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>();
        history.clear();
        closed_set.clear();
        size_t idx_home = home_cell.y * global_map.info.width + home_cell.x;
        double f_home = dist(home_cell, goal_cell);
        double h_home = global_map.data.at(idx_home);
        
        AStarNode home(home_cell, f_home);
        open_set.emplace(home_cell, f_home);
        history.emplace(home_cell, HistoryEntry(f_home, h_home, home_cell));

        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1},
            {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
        };
        
        // start a-star
        while (open_set.size() > 0) {
            // current cell with loest f-score
            AStarNode current = open_set.top();
            open_set.pop();

            if (current.index == goal_cell) {
                findPath();
                break;
            }

            closed_set.emplace(current.index);

            // explore the neighbors
            for (auto &dir : directions) {
                // define the cell index
                CellIndex cell(current.index.x + dir.first, current.index.y + dir.second);

                if (closed_set.find(cell) != closed_set.end())
                    continue;

                size_t idx_cell = cell.y * global_map.info.width + cell.x;
                double cost = global_map.data.at(idx_cell);
                double h_score = history.find(current.index)->second.h_score + dist(cell, current.index) + cost;
                double g_score = dist(cell, goal_cell);
                double f_score = g_score + h_score;

                auto it = history.find(cell);
                if (it != history.end()) 
                {
                    // compare the scores
                    if (f_score < it->second.f_score) {
                        it->second.parent = current.index;
                        it->second.f_score = f_score;
                        it->second.h_score = h_score;
                    }
                } else {
                    HistoryEntry cell_hist(f_score, h_score, current.index);
                    history.emplace(cell, cell_hist);
                    open_set.emplace(cell, f_score);
                }
            }
        }
    }

    void PlannerCore::findPath() {
        // starting from the goal and following the parents
        CellIndex current = goal_cell;

        path.poses.clear();
        geometry_msgs::msg::PoseStamped cell_pose;
        cell_pose.header.frame_id = "sim_world";
        cell_pose.pose.position.x = current.x * global_map.info.resolution + global_map.info.origin.position.x;
        cell_pose.pose.position.y = current.y * global_map.info.resolution + global_map.info.origin.position.y;
        cell_pose.pose.position.z = 0;
        cell_pose.pose.orientation.x = 0;
        cell_pose.pose.orientation.y = 0;
        cell_pose.pose.orientation.z = 0;
        cell_pose.pose.orientation.w = 1;

        path.poses.push_back(cell_pose);

        while (current != home_cell) {
            // find the parent
            CellIndex parent = history.find(current)->second.parent;

            cell_pose.pose.position.x = parent.x * global_map.info.resolution + global_map.info.origin.position.x;
            cell_pose.pose.position.y = parent.y * global_map.info.resolution + global_map.info.origin.position.y;
            
            path.poses.push_back(cell_pose);
            current = parent;
        }
        history.clear();
        closed_set.clear();
    }

    double PlannerCore::dist(const CellIndex &a, const CellIndex &b) {
        // find the width and height
        int width = abs(b.x - a.x);
        int height = abs(b.y - a.y);

        return std::min(width, height)*sqrt(2)*res + abs(width - height)*res;
    }
} 