#include "costmap_core.hpp"
#include <queue>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
    // define the map's info
    local_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    RES_ = 0.1;
    W_ = 60 / RES_;
    H_ = 60 / RES_;
    X_ = -30;
    Y_ = -30;
    MAX_COST_ = 100;

    local_map_->info.resolution = RES_;           
    local_map_->info.width = W_;
    local_map_->info.height = H_;
    // origin position = (0,0,0), orientation = identity
    local_map_->info.origin.position.x = X_;
    local_map_->info.origin.position.y = Y_;
    local_map_->info.origin.position.z = 0.0;
    local_map_->info.origin.orientation.x = 0.0;
    local_map_->info.origin.orientation.y = 0.0;
    local_map_->info.origin.orientation.z = 0.0;
    local_map_->info.origin.orientation.w = 1.0;
}

void CostmapCore::setInput(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    initializeCostmap(scan);

    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    // Step 4: Inflate obstacles
    inflateObstacles();
}

void CostmapCore::initializeCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // create and make the header
    local_map_->data.clear();
    local_map_->header.stamp = scan->header.stamp;
    local_map_->header.frame_id = scan->header.frame_id;  // lidar frame
    local_map_->info.map_load_time = scan->header.stamp;    
    local_map_->data.resize(
        local_map_->info.width * local_map_->info.height, 
        -1  // unknown cells
    );
}

void CostmapCore::convertToGrid(float range, float angle, int &x_grid, int &y_grid){
    // map's resolution
    x_grid = (range * cos(angle) - X_) / RES_;
    y_grid = (range * sin(angle) - Y_) / RES_;
}

void CostmapCore::markObstacle(int x_grid, int y_grid){
    // convert grid to array index
    if (x_grid < W_ && x_grid > 0 && y_grid < H_ && y_grid > 0) {
        size_t idx = y_grid * W_ + x_grid;
        
        local_map_->data.at(idx) = MAX_COST_;
    }
}

void CostmapCore::inflateObstacles()
{
    float radius_cells = 1.5f / RES_;

    std::vector<float> distance_map(local_map_->data.size(),
                                    std::numeric_limits<float>::infinity());

    std::queue<size_t> q;
 
    for (size_t i = 0; i < local_map_->data.size(); i++)
    {
        if (local_map_->data[i] == MAX_COST_) {
            distance_map[i] = 0.0f;
            q.push(i);
        }
    }

    // 8-direction offsets for BFS
    const std::vector<std::pair<int,int>> neighbors = {
        { 1,  0}, {-1,  0}, { 0,  1}, { 0, -1},
        { 1,  1}, { 1, -1}, {-1,  1}, {-1, -1}
    };
    while (!q.empty())
    {
        size_t current_idx = q.front();
        q.pop();

        int x_curr = current_idx % W_;
        int y_curr = current_idx / W_;
        float dist_curr = distance_map[current_idx];

        for (auto & nd : neighbors)
        {
            int nx = x_curr + nd.first;
            int ny = y_curr + nd.second;
            if (nx < 0 || nx >= (int)W_ || ny < 0 || ny >= (int)H_) {
                continue;
            }
            size_t neighbor_idx = ny * W_ + nx;

            float step_dist = std::sqrt(float(nd.first * nd.first + nd.second * nd.second));
            float new_dist = dist_curr + step_dist;

            // update if this path is shorter
            if (new_dist < distance_map[neighbor_idx]) {
                distance_map[neighbor_idx] = new_dist;
                q.push(neighbor_idx);
            }
        }
    }

    // compute inflated cost
    for (size_t i = 0; i < distance_map.size(); i++)
    {
        float d = distance_map[i];
    
        if (std::isinf(d)) {
            local_map_->data[i] = 0;
        }
        else {
            if (d <= radius_cells) {
                float ratio = 1.0f - (d / radius_cells); 
                int cost = static_cast<int>(ratio * MAX_COST_);
                local_map_->data[i] = std::max((int8_t)cost, local_map_->data[i]);
            } else {
                local_map_->data[i] = 0;
            }
        }
    }
}
}