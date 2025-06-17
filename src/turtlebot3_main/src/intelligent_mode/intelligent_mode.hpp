#include <chrono>   //milliseconds
#include <functional> // std::bind
#include <memory>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>

#include <csignal> //interruptions

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// #define PRM_YAML_PATH "/home/alumno.upv.es.iubimuo/map.yaml"  //hardcode by now

using Point = std::pair<double, double>;

static inline double distancia(const Point& a, const Point& b) {
    return std::sqrt((a.first - b.first) * (a.first - b.first) +
                     (a.second - b.second) * (a.second - b.second));
}

static std::vector<std::vector<Point>> get_closest_neighbours(const std::vector<Point>& points, int k) {
    int n = points.size();
    std::vector<std::vector<Point>> neighbour(n);

    for (int i = 0; i < n; ++i) {
        std::vector<std::pair<double, Point>> distances;

        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            double d = distancia(points[i], points[j]);
            distances.push_back({d, points[j]});
        }

        std::sort(distances.begin(), distances.end());

        for (int m = 0; m < k && m < distances.size(); ++m) {
            neighbour[i].push_back(distances[m].second);
        }
    }

    return neighbour;
}

static bool is_connected(const nav_msgs::msg::OccupancyGrid map, Punto p1, Punto p2) {

    
    
}

void get_prm_path(std::vector<Point>& prm_path, int nodes_per_iteration, int n_neighbours)
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
    for(int i=0; i<20; i++)
    {
        for(int j=0; j<nodes_per_iteration; j++)
        {
            double x = static_cast<double>(rand() % width);
            double y = static_cast<double>(rand() % height);

            size_t index = static_cast<size_t>(y) * width + static_cast<size_t>(x);
            if (map.data[index] == 0) {
                valid_cells.push_back({x, y});
            }
        }

        auto neighbours = get_closest_neighbours(valid_cells, n_neighbours);
        RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Iteracion: %d", i);
        
    }
}