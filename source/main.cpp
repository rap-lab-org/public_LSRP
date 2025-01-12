/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include "mapfaa_lsrp.hpp"
#include <iostream>
#include <string>
#include "graph_io.hpp"
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
int Test(const std::string& mapPath, const std::string& scenPath, const std::string& durationPath,double time_limit, bool swap);
void write_result_to_xml(std::vector<std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>>>& result, const std::vector<double>& durations, const std::string& map_path,
                         double runtime, double soc, double makespan);
std::vector<std::tuple<int, int, int>> assign_colors(size_t agent_count);
std::vector<std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>>> visual_convert(std::tuple<int, int>* width_height, std::vector<std::vector<std::tuple<long, long, double, double>>>* all_paths);
std::vector<double> readAgentValues(const std::string& filename);

int main(int argc, char* argv[]) {
    if (argc != 5 && argc != 6) {
        std::cerr << "Usage: " << argv[0] << " <map_path> <scen_path> <duration_path> <runtime> [swap]" << std::endl;
        return -1;
    }


    std::string mapPath = argv[1];
    std::string scenPath = argv[2];
    std::string durationPath = argv[3];
    double runtime = std::stod(argv[4]);
    bool swap = false;

    if (argc == 6 && std::string(argv[5]) == "swap") {
        swap = true;
    }

    return Test(mapPath, scenPath, durationPath, runtime, swap);
}


int Test(const std::string& mapPath, const std::string& scenPath, const std::string& durationPath,double time_limit, bool swap) {
    std::cout << "####### LSRP Begin #######" << std::endl;
    auto duration = readAgentValues(durationPath);
    if (duration.empty()) {
        std::cerr << "Agent number is zero." << std::endl;
        return -1;
    }
    raplab::Grid2d g;
    std::vector<std::vector<double>> occupancy_grid;
    raplab::LoadMap_MovingAI(mapPath, &occupancy_grid);
    g.SetOccuGridPtr(&occupancy_grid);
    std::vector<long> starts;
    std::vector<long> goals;
    std::tuple<int, int> width_height;
    raplab::LoadScenarios(scenPath, int(duration.size()), &starts, &goals, &width_height);
    raplab::Lsrp planner;
    planner.SetGraphPtr(&g);
    planner.Setduration(duration);
    planner.set_swap(swap);
    planner.Solve(starts, goals, time_limit, 5.0);  //eps just to suit thr requirement of input
    auto soc = planner.re_soc();
    auto makespan = planner.re_makespan();
    auto runtime = planner.GetRuntime();
    if (runtime <= 30) {
        std::cout << "Solution found: true" << std::endl;
        std::cout << "Runtime: " <<std::fixed << std::setprecision(3) <<runtime << std::endl;
        std::cout << "Makespan: " <<std::fixed << std::setprecision(2)<<makespan << std::endl;
        std::cout << "Soc: " << std::fixed << std::setprecision(2) << soc << std::endl;
        auto* all_paths = planner.get_all_paths();
        auto visual_paths = visual_convert(&width_height, all_paths);
        write_result_to_xml(visual_paths, duration, mapPath,runtime,soc,makespan);
    } else {
        std::cout << "Solution found: false" << std::endl;
    }
    std::cout << "####### LSRP End #######" << std::endl;
    return 1;
}

void write_result_to_xml(std::vector<std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>>>& result, const std::vector<double>& durations, const std::string& map_path,
                         double runtime, double soc, double makespan) {
    std::ofstream file("result.xml");
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing: " << "result.xml" << std::endl;
        return;
    }

    file << "<?xml version=\"1.0\" ?>" << std::endl;
    file << "<root>" << std::endl;

    std::ifstream map_file(map_path);
    if (!map_file.is_open()) {
        std::cerr << "Unable to open map file: random.map" << std::endl;
        file << "</root>" << std::endl;
        file.close();
        return;
    }

    std::string line;
    int width = 0, height = 0;
    std::vector<std::vector<int>> grid;

    while (std::getline(map_file, line)) {
        if (line.substr(0, 5) == "width") {
            width = std::stoi(line.substr(6));
        } else if (line.substr(0, 6) == "height") {
            height = std::stoi(line.substr(7));
        } else if (line.substr(0, 3) == "map") {
            for (int i = 0; i < height; ++i) {
                std::getline(map_file, line);
                std::vector<int> row;
                for (char c : line) {
                    if (c == '.') {
                        row.push_back(0); //
                    } else if (c == '@') {
                        row.push_back(1); //
                    } else if (c == 'T') {
                        row.push_back(2); //
                    }
                }
                grid.push_back(row);
            }
        }
    }

    file << "    <map>" << std::endl;
    file << "        <grid width=\"" << width << "\" height=\"" << height << "\">" << std::endl;
    for (const auto& row : grid) {
        file << "            <row>";
        for (int cell : row) {
            file << cell << " ";
        }
        file << "</row>" << std::endl;
    }
    file << "        </grid>" << std::endl;
    file << "    </map>" << std::endl;

    file << "    <log>" << std::endl;
    file << "        <summary time=\"" << runtime << "\" flowtime=\"" << soc << "\" makespan=\"" << makespan << "\"/>" << std::endl;
    auto colors = assign_colors(result.size());
    for (size_t i = 0; i < result.size(); ++i) {
        file << "        <agent number=\"" << i << "\">" << std::endl;
        file << "            <color RGB=\"(" << std::get<0>(colors[i]) << ", " << std::get<1>(colors[i]) << ", " << std::get<2>(colors[i]) << ")\"/>" << std::endl;
        file << "            <path duration=\"" << durations[i] << "\">" << std::endl;

        for (size_t j = 0; j < result[i].size(); ++j) {
            const auto& section = result[i][j];
            file << "                <section number=\"" << j << "\" start_i=\"" << std::get<0>(std::get<0>(section)) << "\" start_j=\"" << std::get<1>(std::get<0>(section)) << "\" goal_i=\"" << std::get<0>(std::get<1>(section)) << "\" goal_j=\"" << std::get<1>(std::get<1>(section)) << "\" duration=\"" << std::get<3>(section) - std::get<2>(section) << "\"/>" << std::endl;
        }

        file << "            </path>" << std::endl;
        file << "        </agent>" << std::endl;
    }
    file << "    </log>" << std::endl;

    file << "</root>" << std::endl;
    file.close();
}

std::vector<double> readAgentValues(const std::string& filename) {
    std::vector<double> values;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return values;  // 返回空vector
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string agent;
        double value;
        if (iss >> agent >> value) {
            values.push_back(value);
        }
    }

    file.close();
    return values;
}

std::vector<std::tuple<int, int, int>> assign_colors(size_t agent_count) {
    std::vector<std::tuple<int, int, int>> colors;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (size_t _ = 0; _ < agent_count; ++_) {
        while (true) {
            std::tuple<int, int, int> color = std::make_tuple(dis(gen), dis(gen), dis(gen));
            if (color != std::make_tuple(255, 255, 255) && std::find(colors.begin(), colors.end(), color) == colors.end()) {
                colors.push_back(color);
                break;
            }
        }
    }
    return colors;
}


std::vector<std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>>> visual_convert(std::tuple<int, int>* width_height, std::vector<std::vector<std::tuple<long, long, double, double>>>* all_paths){
    std::vector<std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>>> result;
    int width = std::get<0>(*width_height);
    int height = std::get<1>(*width_height);
    for (size_t i = 0; i < all_paths->size(); ++i) {
        std::vector<std::tuple<std::tuple<int, int>, std::tuple<int, int>, double, double>> agent_path;
        for (size_t j = 0; j < all_paths->at(i).size(); ++j) {
            const auto& section = all_paths->at(i)[j];
            int start_i = std::get<0>(section) % width;
            int start_j = std::get<0>(section) / width;
            int goal_i = std::get<1>(section) % width;
            int goal_j = std::get<1>(section) / width;
            double start_time = std::get<2>(section);
            double end_time = std::get<3>(section);
            agent_path.push_back(std::make_tuple(std::make_tuple(start_j, start_i), std::make_tuple(goal_j, goal_i), start_time, end_time));
        }
        result.push_back(agent_path);
    }
    return result;
}
