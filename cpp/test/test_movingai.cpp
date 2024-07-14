/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include "mapf_mpmstar.hpp"
#include "mapf_mpmstar_v2.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include "graph_io.hpp"

int TestMPMstar_movingai();

int main(){

    TestMPMstar_movingai();
    return 0;
};


int TestMPMstar_movingai(){
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/random-32-32-20.map";
    std::string ScenPath= "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/random-32-32-20-scen/random-32-32-20-random-1.scen";
    raplab::Grid2d g;
    std::vector<std::vector<double>> occupancy_grid;
    raplab::LoadMap_MovingAI(MapPath,&occupancy_grid);
    g.SetOccuGridPtr(&occupancy_grid);
    double time_limit = 60;
    //for (int n = 1; n <= 14; n += 1) {
    int n = 2;
        std::vector<long> starts;
        std::vector<long> goals;
        std::cout<<"Agent number: "<<n<<std::endl;
        raplab::LoadScenarios(ScenPath, n, &starts,&goals);
        raplab:: MPMstar planner;
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
        double inflation = cost[0]/planner.fmin[0];
        std::cout<<"RunTime: "<<runtime<<"| SOC :"<<cost[0]<<"| Fmin: "<<planner.fmin[0]<<" | Individual optimal cost: "<<planner.lowest_bound<<"| Approximate w: "<<inflation
        <<" | Generate states: "<<states_generate<<"| Expand states: "<<states_expand<<"| Max collision set size: "<<
        max_colsets<<"| Max ngh size: "<<max_ngh_size<<"| One step PIBT: "<<one_step_pibt<<"| Action tree :"<<
        planner.action_tree<<std::endl;

        //std::cout<<"| SOC :"<<cost[0]<<std::endl;
    //}
    /*
    for (auto path: plan) {
        std::cout << " path = " << path << std::endl;
    }*/
    std::cout << "####### moving ai test End #######" << std::endl;
    return 1;
}
