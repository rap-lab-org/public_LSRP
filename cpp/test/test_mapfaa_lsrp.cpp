/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapfaa_lsrp.hpp"
#include "mapf_pibt.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>

int TestLsrp();

int main(){

    TestLsrp();
    return 0;
};


int TestLsrp(){
    std::cout << "####### test_pibt.cpp - TestLsrp() Begin #######" << std::endl;
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
    int r=4,c=3;
    std::vector<std::vector<double>> occupancy_grid;
    occupancy_grid.resize(r);
    for (int i = 0; i < r; i++){
        occupancy_grid[i].resize(c, 0);
    }
    occupancy_grid[1][0] = 1;
    occupancy_grid[1][2] = 1;
    occupancy_grid[2][0] = 1;
    occupancy_grid[2][2] = 1;
    occupancy_grid[3][0] = 1;
    occupancy_grid[3][2] = 1;
    //occupancy_grid[1][0] = 1;
    g.SetOccuGridPtr(&occupancy_grid);
    // not using it now
    double time_limit = 300;
    std::vector<long> starts({4,7}); // node id = y*NumX + x; e.g. (y=1)*4 + (x=0) = 4
    // the length of starts shows how many agents are there.
    std::vector<long> goals({7,4});
    raplab::Lsrp planner;
    planner.SetGraphPtr(&g);
    std::vector<double> duration(starts.size(),1);
    //std::vector<double> duration = {1,2};
    planner._Solve(starts, goals, time_limit, duration);
    /*
    raplab::Pibt planner;
    planner.SetGraphPtr(&g);
    planner._Solve(starts,goals,30,1.0, nullptr);
    */
    auto plan = planner.GetPlan();
    auto cost = planner.GetPlanCost();
    std::cout<<" The SOC is "<<cost[0]<<std::endl;
    std::cout<<"The Makespan is"<<cost[1]<<std::endl;
    /*
    for (auto path: plan) {
        std::cout << " path = " << path << std::endl;
    }*/
    std::cout << "####### test_LSRP.cpp - TestLSRP() End #######" << std::endl;
    return 0;
}