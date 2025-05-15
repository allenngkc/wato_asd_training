#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class PlannerCore {
  public:

    explicit PlannerCore(const rclcpp::Logger& logger);
      
    struct CellIndex
    {
      int x;
      int y;
    
      CellIndex(int xx, int yy) : x(xx), y(yy) {}
      CellIndex() : x(0), y(0) {}
    
      bool operator==(const CellIndex &other) const
      {
        return (x == other.x && y == other.y);
      }
    
      bool operator!=(const CellIndex &other) const
      {
        return (x != other.x || y != other.y);
      }
    };
    
    // Hash function for CellIndex so it can be used in std::unordered_map
    struct CellIndexHash
    {
      std::size_t operator()(const CellIndex &idx) const
      {
        // A simple hash combining x and y
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
      }
    };
    
    // Structure representing a node in the A* open set
    struct AStarNode
    {
      CellIndex index;
      double f_score;  // f = g + h
    
      AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
    };
    
    // Comparator for the priority queue (min-heap by f_score)
    struct CompareF
    {
      bool operator()(const AStarNode &a, const AStarNode &b)
      {
        // We want the node with the smallest f_score on top
        return a.f_score > b.f_score;
      }
    };

    struct HistoryEntry
    {
      double f_score;
      double h_score;
      CellIndex parent;
      // Default Constructor
      HistoryEntry() 
          : f_score(0.0), h_score(0.0), parent() {}
          
      HistoryEntry(double s1, double s2, CellIndex& p)
        : f_score(s1), h_score(s2), parent(p) {}
    };

    void plan(const nav_msgs::msg::OccupancyGrid &map, CellIndex &goal, CellIndex &home);

    void aStar();

    void findPath();

    double dist(const CellIndex &a, const CellIndex &b);

    nav_msgs::msg::Path path;

  private:
    rclcpp::Logger logger_;

    double res;  // global map's resolution

    nav_msgs::msg::OccupancyGrid global_map;
    CellIndex goal_cell, home_cell;

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_set<CellIndex, CellIndexHash> closed_set;
    std::unordered_map<CellIndex, HistoryEntry, CellIndexHash> history;
};

}  

#endif  