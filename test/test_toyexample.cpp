/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include "mapfaa_lsrp.hpp"
#include <iostream>
#include <string>
#include "graph_io.hpp"
#include <vector>
#include <iomanip> // Include this header for std::fixed and std::setprecision
#include <fstream> // Include this header for file operations



int Toyexample();
int edge_hash(long a, long b);



int main(){
    Toyexample();
    return 0;
};

int Toyexample() {
    std::cout << "####### Toy-example Begin #######" << std::endl;
    /*
    3x3 Grid graph：
    (0,0) (0,1) (0,2)                           0  1  2
    (1,0) (1,1) (1,2)  and corresponding ids:   3  4  5
    (2,0) (2,1) (2,2)                           6  7  8
    illustration：2D coordinate (x, y) can be transformed to id：ID = y * width + x
    eg： (0,0) -> 0, (2,2) -> 8

    Obstacles position (X) ：
      0  1  2
      X  4  5
      6  7  8

    start and goals (S) (G) ：
      S1  1  G2
      X   4  5
      S2  7  G1

    time for each agent to move from one cell to another:
    agent 1 duration = 1, agent 2 duration = 0.5

    specific edge duration for specific agent (optional)：
    agent 1： duration = 0.5 at edge (0,1) to (0,2)
    */

    raplab::Grid2d g;
    std::vector<std::vector<double>> occupancy_grid(3, std::vector<double>(3, 0));
    // set
    occupancy_grid[1][0] = 1; // obstacle 1
    g.SetOccuGridPtr(&occupancy_grid);
    // set starts and goals
    std::vector<long> starts = {0, 6}; // (0,0) and (2,0)
    std::vector<long> goals = {8, 2};  // (2,2) and (0,2)
    // set duration for each agent
    std::vector<double> duration = {1, 0.5}; //
    // set specific edge duration for specific agent if required (optional)
    std::unordered_map<int, std::unordered_map<int, double>> edge_duration = {
        {1, {{edge_hash(1, 2), 0.5}}}
    }; // agent 1 duration = 0.5 at edge (0,1) to (0,2)
    // set time limit
    double time_limit = 30;
    // initialize
    raplab::Lsrp planner;
    planner.SetGraphPtr(&g);
    planner.Setduration(duration);
    planner.set_edge_cost(edge_duration); // optional
    // search
    planner.Solve(starts, goals, time_limit, 1.0);  // eps is not used
    // get the result
    auto soc = planner.re_soc();
    auto makespan = planner.re_makespan();
    auto runtime = planner.GetRuntime();
    // output the result
    if (runtime <= 30) {
        std::cout << "Solution found: true" << std::endl;
        std::cout << "Runtime: " << std::fixed << std::setprecision(3) << runtime << std::endl;
        std::cout << "Makespan: " << std::fixed << std::setprecision(2) << makespan << std::endl;
        std::cout << "Soc: " << std::fixed << std::setprecision(2) << soc << std::endl;
    } else {
        std::cout << "Solution found: false" << std::endl;
    }
    std::cout << "####### Toy-example End #######" << std::endl;
    return 1;
}


int edge_hash(long a, long b)
{
    //use cantor hash to match the method inside lsrp
    if (a > b)
    {
        return(a + b) * (a + b + 1) / 2 + b;
    } else {
        return(a + b) * (b + a + 1) / 2 + a;
    }
}



