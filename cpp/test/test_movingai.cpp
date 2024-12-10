/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include "mapf_mpmstar.hpp"
#include "mapf_mpmstar_v2.hpp"
#include "mapfaa_lsrp.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include "graph_io.hpp"
#include <vector>
#include <numeric>
#include <iomanip> // Include this header for std::fixed and std::setprecision
#include <fstream> // Include this header for file operations


int TestMPMstar_movingai();
int Test_Lsrp_movingai();
int TestLsrp();
int Test_Lsrp_movingai_to_file(); // Declare the new function
int Test_Lsrp_movingai_to_file_with_cyclic_duration();
int Test_Lsrp_den520d();
int Test_Lsrp_den520_notbaseline();


int main(){

    //TestMPMstar_movingai();
    //TestLsrp();
    //Test_Lsrp_movingai();
    //Test_Lsrp_movingai_to_file();
    //Test_Lsrp_movingai_to_file_with_cyclic_duration();
    Test_Lsrp_den520d();
    //Test_Lsrp_den520_notbaseline();
    return 0;
};


int TestMPMstar_movingai(){
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/random-32-32-20.map";
    for (int i = 1; i < 2; i+=1) {
        std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/random-32-32-20-scen/random-32-32-20-random-)" + std::to_string(i) + ".scen";
        raplab::Grid2d g;
        std::vector<std::vector<double>> occupancy_grid;
        raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
        g.SetOccuGridPtr(&occupancy_grid);
        double time_limit = 60;
        for (int n = 2; n <= 20; n += 1) {
            //int n = 2;
            std::vector<long> starts;
            std::vector<long> goals;
            std::cout << "Agent number: " << n << std::endl;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::MPMstar planner;
            planner.SetGraphPtr(&g);
            planner.Solve(starts, goals, time_limit, 1.2);
            auto plan = planner.GetPlan();
            auto cost = planner.GetPlanCost();
            auto runtime = planner.Get_runtime();
            int states_generate = planner.states_generate;
            int states_expand = planner.states_expand;
            int max_colsets = planner.max_colsets;
            int max_ngh_size = planner.max_ngh_size;
            int one_step_pibt = planner.one_step_pibt;
            double inflation = cost[0] / planner.fmin[0];
            std::cout << "RunTime: " << runtime << "| SOC :" << cost[0] << "| Fmin: " << planner.fmin[0]
                      << " | Individual optimal cost: " << planner.lowest_bound << "| Approximate w: " << inflation
                      << " | Generate states: " << states_generate << "| Expand states: " << states_expand
                      << "| Max collision set size: " <<
                      max_colsets << "| Max ngh size: " << max_ngh_size <<"| Indi optimal path: "<<planner.optimal_path<< "| One step PIBT: " << one_step_pibt
                      << "| Action tree :" <<
                      planner.action_tree << std::endl;

            //std::cout<<"| SOC :"<<cost[0]<<std::endl;
        }
    }
    /*
    for (auto path: plan) {
        std::cout << " path = " << path << std::endl;
    }*/
    std::cout << "####### moving ai test End #######" << std::endl;
    return 1;
}

int TestLsrp(){
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/warehouse-10-20-10-2-1.map";
    for (int i = 1; i <= 1; i+=1) {
        std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/warehouse-161-63-scen-random/warehouse-10-20-10-2-1-random-)" + std::to_string(i) + ".scen";
        raplab::Grid2d g;
        std::vector<std::vector<double>> occupancy_grid;
        raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
        g.SetOccuGridPtr(&occupancy_grid);
        double time_limit = 60;
        for (int n = 1000; n <= 1000; n += 5) {
            //int n = 2;

            std::vector<long> starts;
            std::vector<long> goals;
            std::cout << "Agent number: " << n << std::endl;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            std::vector<double> duration(starts.size(),1);
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit,1.0);
            auto plan = planner.GetPlan();
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();
            std::cout << "RunTime: " << runtime << "s| SOC :" << soc << "| Makespan: " <<makespan
                      << std::endl;
        }
    }
    std::cout << "####### moving ai test End #######" << std::endl;
    return 1;
}


int Test_Lsrp_movingai() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/warehouse-10-20-10-2-1.map";

    for (int n = 10; n <= 10; n += 30) {
        int success_count = 0;
        double total_runtime = 0;
        double total_soc = 0;
        double total_makespan = 0;

        for (int i = 1; i <= 1; ++i) {
            std::string ScenPath =
                    R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/warehouse-161-63-scen-random/warehouse-10-20-10-2-1-random-)" +
                    std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 30;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            //std::vector<double> duration(starts.size(), 1);
            std::vector<double> duration(starts.size());
            for (size_t j = 0; j < duration.size(); ++j) {
                duration[j] = (j % 5) + 1;
                planner.Setduration(duration);
                planner.Solve(starts, goals, time_limit, 1.0);
                auto plan = planner.GetPlan();
                auto soc = planner.re_soc();
                auto makespan = planner.re_makespan();
                auto runtime = planner.GetRuntime();

                if (runtime <= 30) {
                    ++success_count;
                    total_runtime += runtime;
                    total_soc += soc;
                    total_makespan += makespan;
                }
            }

            double success_rate = (success_count / 25.0) * 100;
            double average_runtime = success_count > 0 ? total_runtime / success_count : 0;
            double average_soc = success_count > 0 ? total_soc / success_count : 0;
            double average_makespan = success_count > 0 ? total_makespan / success_count : 0;

            std::cout << "Agent Count: " << n << std::endl;
            std::cout << std::fixed << std::setprecision(6); // Use fixed format and set precision to 2 decimal places
            std::cout << "Success Rate: " << success_rate << "%" << std::endl;
            std::cout << "Average Runtime: " << average_runtime << " s" << std::endl;
            std::cout << "Average Soc: " << average_soc << std::endl;
            std::cout << "Average Makespan: " << average_makespan << std::endl;
        }

        std::cout << "####### Moving ai test End #######" << std::endl;
        return 1;
    }
}

int Test_Lsrp_movingai_to_file() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/warehouse-10-20-10-2-1.map";
    std::ofstream output_file("C:/Users/David Zhou/Documents/GitHub/public_LSRP/lsrp-warehouse-baseline2-results.txt");

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = 10; n <= 700; n += 30) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 25; ++i) {
            std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/warehouse-161-63-scen-random/warehouse-10-20-10-2-1-random-)" + std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 30;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            std::vector<double> duration(starts.size(), 5);
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit, 1.0);
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();

            if (runtime <= 30) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << runtime << std::endl;
                output_file << "Makespan: " << makespan << std::endl;
                output_file << "Soc: " << soc << std::endl;
            } else {
                output_file << "Solution found: false" << std::endl;
                output_file << "Runtime: 0" << std::endl;
                output_file << "Makespan: 0" << std::endl;
                output_file << "Soc: 0" << std::endl;
            }
        }

        double success_rate = (success_count / 25.0) * 100;
        output_file << "Success rate: " << success_rate << "%" << std::endl << std::endl;
    }

    output_file.close();
    std::cout << "####### Moving ai test End #######" << std::endl;
    return 1;
}


int Test_Lsrp_movingai_to_file_with_cyclic_duration() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/warehouse-10-20-10-2-1.map";
    std::ofstream output_file("C:/Users/David Zhou/Documents/GitHub/public_LSRP/lsrp-warehouse-baseline3-results.txt");

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = 10; n <= 700; n += 30) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 25; ++i) {
            std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/warehouse-161-63-scen-random/warehouse-10-20-10-2-1-random-)" + std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 30;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            std::vector<double> duration(starts.size());
            for (size_t j = 0; j < duration.size(); ++j) {
                duration[j] = (j % 5) + 1;
            }
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit, 1.0);
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();

            if (runtime <= 30) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << runtime << std::endl;
                output_file << "Makespan: " << makespan << std::endl;
                output_file << "Soc: " << soc << std::endl;
            } else {
                output_file << "Solution found: false" << std::endl;
                output_file << "Runtime: 0" << std::endl;
                output_file << "Makespan: 0" << std::endl;
                output_file << "Soc: 0" << std::endl;
            }
        }

        double success_rate = (success_count / 25.0) * 100;
        output_file << "Success rate: " << success_rate << "%" << std::endl << std::endl;
    }

    output_file.close();
    std::cout << "####### Moving ai test End #######" << std::endl;
    return 1;
}

int Test_Lsrp_den520d() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/random-32-32-20.map";
    std::ofstream output_file("C:/Users/David Zhou/Documents/GitHub/public_LSRP/lsrp_swap-room64-results.txt");

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = 450; n <= 450; n += 50) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 20; ++i) {
            std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/random-32-32-20-scen/random-32-32-20-random-)" + std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 60;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            //std::vector<double> duration(starts.size(), 5);
            std::vector<double> duration(starts.size());
            for (size_t j = 0; j < duration.size(); ++j) {
                duration[j] = (j % 5) + 1;
            }
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit, 5.0);
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();

            if (runtime <= 30) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << runtime << std::endl;
                output_file << "Makespan: " << makespan << std::endl;
                output_file << "Soc: " << std::fixed << std::setprecision(2) << soc << std::endl;
            } else {
                output_file << "Solution found: false" << std::endl;
                output_file << "Runtime: 0" << std::endl;
                output_file << "Makespan: 0" << std::endl;
                output_file << "Soc: 0" << std::endl;
            }
        }

        double success_rate = (success_count / 20.0) * 100;
        output_file << "Success rate: " << success_rate << "%" << std::endl;
    }

    output_file.close();
    std::cout << "####### Moving ai test End #######" << std::endl;
    return 1;
}

int Test_Lsrp_den520_notbaseline() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/den520d.map";
    std::ofstream output_file("C:/Users/David Zhou/Documents/GitHub/public_LSRP/lsrp_swap-den520-baseline2-results.txt");

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = 10; n <= 710; n += 50) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 25; ++i) {
            std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/den520d-scen-random/den520d-random-)" + std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 30;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            std::vector<double> duration(starts.size());
            for (size_t j = 0; j < duration.size(); ++j) {
                duration[j] = (j % 5) + 1;
            }
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit, 1.0);
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();

            if (runtime <= 30) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << runtime << std::endl;
                output_file << "Makespan: " << makespan << std::endl;
                output_file << "Soc: " << soc << std::endl;
            } else {
                output_file << "Solution found: false" << std::endl;
                output_file << "Runtime: 0" << std::endl;
                output_file << "Makespan: 0" << std::endl;
                output_file << "Soc: 0" << std::endl;
            }
        }

        double success_rate = (success_count / 25.0) * 100;
        output_file << "Success rate: " << success_rate << "%" << std::endl << std::endl;
    }

    output_file.close();
    std::cout << "####### Moving ai test End #######" << std::endl;
    return 1;
}