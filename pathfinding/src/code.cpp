#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <queue>
#include <limits>
#include <visualization_msgs/Marker.h>

// Define your Node structure for A* algorithm
struct Node {
    int i, j;
    float f, g, h;
    Node* parent;

    Node(int _i, int _j) : i(_i), j(_j), f(0), g(std::numeric_limits<float>::infinity()), h(0), parent(nullptr) {}

    float dist(const Node& other) const {
        return std::sqrt(std::pow(i - other.i, 2) + std::pow(j - other.j, 2));
    }
};

// Structure to compare nodes
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->f > b->f;
    }
};

// Define a heuristic function
float heuristic(const Node& current, const Node& goal) {
    // Using Euclidean distance as heuristic
    return std::sqrt(std::pow(current.i - goal.i, 2) + std::pow(current.j - goal.j, 2));
}

// Function to get neighboring nodes
std::vector<Node*> getNeighbors(Node* node, const std::vector<std::vector<int>>& map)
 {
   
        //returns vector of neighbouring nodes based on allowed movement directions
        std::vector<Node*> neighbors;
        int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
        int numDirs = allowDiagonals ? 8 : 4;

        for (int d = 0; d < numDirs; ++d) {
            int ni = node->i + di[d];
            int nj = node->j + dj[d];
            if (ni >= 0 && ni < map.size() && nj >= 0 && nj < map[0].size() && map[ni][nj] == 0) {
                neighbors.push_back(new Node(ni, nj));
            }
        }

        return neighbors;
    }

// Reconstruct the path if the goal node is found
std::vector<Node*> reconstructPath(Node* goal) {
    std::vector<Node*> path;
    Node* current = goal;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to start from the start node
    return path;
}

// Return the reconstructed path
std::vector<Node*> findPath(Node* start, Node* goal, const std::vector<std::vector<int>>& map) {
    // Initialize open and closed sets
    std::vector<std::vector<bool>> closedSet(map.size(), std::vector<bool>(map[0].size(), false));
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;

    // Add start node to open set
    start->g = 0;
    start->h = heuristic(*start, *goal);
    start->f = start->g + start->h;
    openSet.push(start);

    // Loop until open set is empty or goal is found
    while (!openSet.empty()) {
        // Select the node with the lowest total cost from the open set
        Node* current = openSet.top();
        openSet.pop();

        // Add the selected node to the closed set
        closedSet[current->i][current->j] = true;

        // If the selected node is the goal node, reconstruct the path and return it
        if (current == goal) {
            return reconstructPath(goal);
        }

        // Expand the selected node by generating its neighboring nodes
        std::vector<Node*> neighbors = getNeighbors(current, map);
        for (Node* neighbor : neighbors) {
            // Check if neighbor is valid
            if (neighbor->i < 0 || neighbor->i >= map.size() || neighbor->j < 0 || neighbor->j >= map[0].size() || map[neighbor->i][neighbor->j] != 0) {
                continue; // Skip invalid or obstructed nodes
            }

            // Calculate the cost to reach the neighboring node from the current node
            float tentativeG = current->g + current->dist(*neighbor);

            // If the neighboring node is not in the closed set or has a lower cost than previously recorded
            if (!closedSet[neighbor->i][neighbor->j] || tentativeG < neighbor->g) {
                // Update neighbor's cost and parent
                neighbor->g = tentativeG;
                neighbor->h = heuristic(*neighbor, *goal);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                // Add the neighboring node to the open set
                openSet.push(neighbor);

                // Mark the neighboring node as visited
                closedSet[neighbor->i][neighbor->j] = true;
            }
        }
    }

    // If no path is found, return an empty
    if (path.empty()) {
        ROS_INFO("No path found. Not publishing.");
    }
}