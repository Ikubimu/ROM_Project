#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>

#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using Point = std::pair<int, int>;

static inline int get_value_cell(const nav_msgs::msg::OccupancyGrid& map, Point p)
{
    size_t width = map.info.width;
    size_t index = p.second * width + p.first;
    return map.data[index];
}

static inline Point get_coordenates_cell(const nav_msgs::msg::OccupancyGrid& map, std::pair<double, double> p)
{
    int x = static_cast<int>(std::round((p.first - map.info.origin.position.x) / map.info.resolution));
    int y = static_cast<int>(std::round((p.second - map.info.origin.position.y) / map.info.resolution));
    return Point(x, y);
}

static inline double distancia(const Point& a, const Point& b) {
    return std::sqrt((a.first - b.first) * (a.first - b.first) +
                     (a.second - b.second) * (a.second - b.second));
}


static bool is_connected(const nav_msgs::msg::OccupancyGrid& map, Point p1, Point p2) {
    int x1 = p1.first;
    int y1 = p1.second;
    int x2 = p2.first;
    int y2 = p2.second;

    int dx = x2 - x1;
    int dy = y2 - y1;

    int steps = std::max(std::abs(dx), std::abs(dy));
    double x_inc = static_cast<double>(dx) / steps;
    double y_inc = static_cast<double>(dy) / steps;

    double x = x1;
    double y = y1;

    for (int i = 0; i <= steps; ++i) {
        int cx = static_cast<int>(std::round(x));
        int cy = static_cast<int>(std::round(y));

        int value = get_value_cell(map, {cx, cy});
        if (value > 5) return false;

        x += x_inc;
        y += y_inc;
    }

    return true; 
}

static std::vector<std::vector<int>> get_closest_neighbours(
    const std::vector<Point>& points,
    int k,
    const nav_msgs::msg::OccupancyGrid& map)
{
    int n = points.size();
    std::vector<std::vector<int>> neighbour(n);

    for (int i = 0; i < n; ++i) {
        std::vector<std::pair<double, int>> distances;

        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            double d = distancia(points[i], points[j]);
            distances.push_back({d, j});
        }

        std::sort(distances.begin(), distances.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        int added = 0;
        for (int m = 0; m < static_cast<int>(distances.size()) && added < k; ++m) {
            int vecino_idx = distances[m].second;
            if (is_connected(map, points[i], points[vecino_idx])) {
                neighbour[i].push_back(vecino_idx);
                ++added;
            }
        }
    }

    return neighbour;
}

/******************************************************
******************************************************/

//RECURSIVE METHOD TO TRAVEL THE GRAPH AND FIND THE PATH


static std::vector<bool> visited_nodes;

//remember origin is in pos = 0 and destiny is in pos = 1

bool recursive_find_path(std::vector<int>& index_path,
               const std::vector<std::vector<int>>& neighbours,
               int index)
{
    if(index == 1)
    {
        index_path.push_back(index);
        return true;
    }

    for(long unsigned int i=0; i<neighbours[index].size(); ++i)
    {
        int neighbour_index = neighbours[index][i];
        if(visited_nodes[neighbour_index] == true)
            continue;

        visited_nodes[neighbour_index] = true;

        if(recursive_find_path(index_path, neighbours, neighbour_index) == true)
        {
            index_path.push_back(index);
            return true;
        }
    }
    return false;

}


bool find_path(std::vector<int>& index_path,
               const std::vector<std::vector<int>>& neighbours)
{
    visited_nodes.clear();
    for(long unsigned int i=0; i< neighbours.size(); ++i)
    {
        visited_nodes.push_back(false);
    }

    return recursive_find_path(index_path, neighbours, 0);
}


/******************************************************
******************************************************/


void get_prm_path(std::vector<std::pair<double, double>>& prm_path, int nodes_per_iteration, int n_neighbours, std::pair<double, double> origin, std::pair<double, double> destiny)
{
    prm_path.clear();

    nav_msgs::msg::OccupancyGrid map;
    try {
        nav2_map_server::loadMapFromYaml("/home/alumno.upv.es.iubimuo/map.yaml", map);
        RCLCPP_INFO(rclcpp::get_logger("load_map"), "Mapa cargado con éxito.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Error al cargar el mapa: %s", e.what());
        return;
    }

    uint32_t width = map.info.width;
    uint32_t height = map.info.height;

    // RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Tamaño del mapa: %dx%d", width, height);

    // for (uint32_t y = 0; y < height; ++y) {
    //     for (uint32_t x = 0; x < width; ++x) {
    //         size_t index = y * width + x;
    //         int8_t value = map.data[index];
    //         if(value != 0)
    //         {
    //             RCLCPP_INFO(rclcpp::get_logger("map_logger"),
    //                     "Celda (%d, %d): %d", x, y, value);
    //         }
    //     }
    // }

    std::vector<Point> valid_cells;
    valid_cells.push_back(get_coordenates_cell(map, origin));
    valid_cells.push_back(get_coordenates_cell(map, destiny));

    std::vector<int> index_path;

    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < nodes_per_iteration; ++j) {
            int x = rand() % width;
            int y = rand() % height;
            size_t index = y * width + x;

            if (map.data[index] == 0) {
                valid_cells.emplace_back(x, y);
            }
        }

        auto neighbours = get_closest_neighbours(valid_cells, n_neighbours, map);

        bool ret = find_path(index_path, neighbours);
        if (!ret) {
            index_path.clear();
        }
        else{
            std::reverse(index_path.begin(), index_path.end());
            for(long unsigned int i=0; i<index_path.size(); ++i)
            {
                RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Index: %d", index_path[i]);
            }
            break;
        }

        RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Iteración: %d", i);
    }
    RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Finished generating nodes and neighbours.");
}
