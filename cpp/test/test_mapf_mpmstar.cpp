/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapf_mpmstar.hpp."
#include "mapf_mpmstar_v2.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>

int TestMPMstar();

int main(){

    TestMPMstar();
    return 0;
};


int TestMPMstar(){
    std::cout << "####### test_mpmstar.cpp - TestMpmstar() Begin #######" << std::endl;
    // ------------static environment, 3x3, obstacle at (y=0,x=0),(y=0.x=1),(y=0,x=2),(y=1,x=0) Three robots---------
    //   x:0  1  2
    // y ---------------
    // 0 | #  #  #  |
    // 1 | #  *  R3 |
    // 2 | R1 *  R2 |
    //   ---------------
    //  Note: *=free, #=obstacle, R1,R2,R3 = robot start, G1,G2,G3 = robot goals
    // -------------------------------------------------------------------------------
    raplab::Grid2d g;
    int r=3,c=3;
    std::vector<std::vector<double>> occupancy_grid;
    occupancy_grid.resize(r);
    for (int i = 0; i < r; i++){
        occupancy_grid[i].resize(c, 0);
    }
    occupancy_grid[0][0] = 1;
    occupancy_grid[0][1] = 1;
    occupancy_grid[0][2] = 1;
    occupancy_grid[1][0] = 1;
    g.SetOccuGridPtr(&occupancy_grid);
    // not using it now
    double time_limit = 300;
    std::vector<long> starts({6,8,5}); // node id = y*NumX + x; e.g. (y=1)*4 + (x=0) = 4
    // the length of starts shows how many agents are there.
    std::vector<long> goals({5,6,8});

    raplab:: MPMstar planner;
    planner.SetGraphPtr(&g);
    planner.Solve(starts, goals, time_limit, 1.0);
    auto plan = planner.GetPlan();
    auto cost = planner.GetPlanCost();
    std::cout<<" The SOC is "<<cost[0]<<std::endl;
    for (auto path: plan) {
        std::cout << " path = " << path << std::endl;
    }
    std::cout << "####### test_mpmstar.cpp - TestPibt() End #######" << std::endl;
}