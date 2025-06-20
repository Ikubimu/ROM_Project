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

typedef struct
{
    std::vector<int> index_neighbours;
    std::vector<double> distances;
} Neighbours_t;

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

static inline std::pair<double, double> get_cell_coordinates(const nav_msgs::msg::OccupancyGrid& map, Point p)
{
    double x = map.info.origin.position.x + p.first * map.info.resolution;
    double y = map.info.origin.position.y + p.second * map.info.resolution;
    return std::make_pair(x, y);
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


/******************************************************
******************************************************/
/*

Para la implementación de A* se ha recurrido a herramientas externas IA
ChatGPT casi en su totalidad

*/

static std::vector<double> heuristic;

static std::vector<Neighbours_t> get_closest_neighbours(
    const std::vector<Point>& points,
    int k,
    const nav_msgs::msg::OccupancyGrid& map)
{
    int n = points.size();
    std::vector<Neighbours_t> neighbours(n);

    if (n < 2) {
        std::cerr << "ERROR: Se requieren al menos 2 puntos para calcular la heurística." << std::endl;
        return neighbours; // vacío o incompleto
    }

    heuristic.clear();
    heuristic.reserve(n);
    for (int i = 0; i < n; ++i) {
        heuristic.push_back(distancia(points[i], points[1]));
    }

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

            // Validar índice vecino
            if (vecino_idx < 0 || vecino_idx >= n) {
                std::cerr << "ERROR: índice vecino fuera de rango: " << vecino_idx << std::endl;
                continue;
            }

            double dist = distances[m].first;

            if (is_connected(map, points[i], points[vecino_idx])) {
                neighbours[i].index_neighbours.push_back(vecino_idx);
                neighbours[i].distances.push_back(dist);
                ++added;
            }
        }
    }

    return neighbours;
}




static std::vector<bool> visited_nodes;
static std::vector<double> best_cost;

// Recursive A* function
bool recursive_a_star(std::vector<int>& path,
                      const std::vector<Neighbours_t>& neighbours,
                      int current,
                      double current_g_cost)
{
    std::size_t N = neighbours.size();

    if (current < 0 || current >= static_cast<int>(N)) {
        std::cerr << "ERROR: índice current fuera de rango: " << current << std::endl;
        return false;
    }

    if (current == 1) // Nodo destino
    {
        path.push_back(current);
        return true;
    }

    visited_nodes[current] = true;

    const auto& current_neighbors = neighbours[current];
    bool found_path = false;
    std::vector<int> best_subpath;
    double best_f_cost = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < current_neighbors.index_neighbours.size(); ++i)
    {
        int neighbor = current_neighbors.index_neighbours[i];

        if (neighbor < 0 || neighbor >= static_cast<int>(N)) {
            std::cerr << "ERROR: índice vecino fuera de rango en recursive_a_star: " << neighbor << std::endl;
            continue;
        }

        double edge_cost = current_neighbors.distances[i];
        double tentative_g = current_g_cost + edge_cost;

        // Saltar si peor o ya visitado con menor coste
        if (visited_nodes[neighbor] && tentative_g >= best_cost[neighbor])
            continue;

        if (tentative_g < best_cost[neighbor])
            best_cost[neighbor] = tentative_g;
        else
            continue;

        std::vector<int> subpath;
        if (recursive_a_star(subpath, neighbours, neighbor, tentative_g))
        {
            double total_f = tentative_g + heuristic[neighbor];
            if (total_f < best_f_cost)
            {
                best_f_cost = total_f;
                best_subpath = std::move(subpath);
                found_path = true;
            }
        }
    }

    if (found_path)
    {
        path = std::move(best_subpath);
        path.push_back(current);
        return true;
    }

    return false;
}


static bool find_path(std::vector<int>& path,
               const std::vector<Neighbours_t>& neighbours)
{
    std::size_t N = neighbours.size();
    if (N == 0) {
        std::cerr << "ERROR: lista de vecinos vacía." << std::endl;
        return false;
    }

    visited_nodes.assign(N, false);
    best_cost.assign(N, std::numeric_limits<double>::infinity());
    best_cost[0] = 0.0;

    return recursive_a_star(path, neighbours, 0, 0.0);
}


/******************************************************
******************************************************/


void get_prm_path(std::string map_path,
    std::vector<std::pair<double, double>>& prm_path, 
    int nodes_per_iteration, int n_neighbours, 
    std::pair<double, double> origin, 
    std::pair<double, double> destiny)
{
    prm_path.clear();

    nav_msgs::msg::OccupancyGrid map;
    try {
        nav2_map_server::loadMapFromYaml(map_path, map);
        RCLCPP_INFO(rclcpp::get_logger("load_map"), "Mapa cargado con éxito.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Error al cargar el mapa: %s", e.what());
        return;
    }

    uint32_t width = map.info.width;
    uint32_t height = map.info.height;

    std::vector<Point> valid_cells;
    valid_cells.push_back(get_coordenates_cell(map, origin));
    valid_cells.push_back(get_coordenates_cell(map, destiny));

    std::vector<int> index_path;

    int min_iteration = 10;

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
        RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Iteración: %d", i);
        
        if(i<min_iteration) continue;

        bool ret = find_path(index_path, neighbours);
        if (!ret) {
            index_path.clear();
            visited_nodes.clear();
        }
        else{
            std::reverse(index_path.begin(), index_path.end());
            // for(long unsigned int i=0; i<index_path.size(); ++i)
            // {
            //     RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Index: %d", index_path[i]);
            // }
            for(long unsigned int i=0; i<index_path.size(); i++)
            {
                Point p = valid_cells[index_path[i]];
                std::pair<double, double> coordinates = get_cell_coordinates(map, p);
                prm_path.push_back(coordinates);
                RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Node %ld: (%f, %f)", i, coordinates.first, coordinates.second);
            }
            break;
        }

    }
    RCLCPP_INFO(rclcpp::get_logger("map_logger"), "Finished generating nodes and neighbours.");
}
