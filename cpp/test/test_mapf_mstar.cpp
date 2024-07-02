
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mapf_mstar.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>


int TestMstarPolicy();
int TestMstar();
int TestMstar2();

int main(){

  // TestMstarPolicy();
  // TestMstar();
  TestMstar2();
  
  return 0;
};

int TestMstarPolicy(){

  std::cout << "####### test_mstar.cpp - TestMstarPolicy() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  raplab::PlannerGraph* g_ptr ;
  raplab::SparseGraph g;

  timer.Start();
  g.AddVertex(0);
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddVertex(4);

  g.AddArc(0,1, std::vector<double>({11.3}) );
  g.AddArc(1,0, std::vector<double>({0.3}) );
  g.AddEdge(1,2, std::vector<double>({15.5}) );
  g.AddArc(2,3, std::vector<double>({15.5}) );
  g.AddEdge(3,4, std::vector<double>({16}) );
  g.AddArc(4,1, std::vector<double>({17.6}) );
  g.AddEdge(1,7, std::vector<double>({9.9}) );

  g.AddArc(0,3, std::vector<double>({6}) );

  g_ptr = &g;

  auto p = raplab::MstarPolicy();
  p.SetGraphPtr(g_ptr);
  p.Compute(4, 100);
  for (int vv = 0; vv < 5; vv++) {
    std::cout << " v = " << vv << " phi = " << p.Phi(vv) << std::endl;
    std::cout << " v = " << vv << " h = " << p.H(vv) << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### test_mstar.cpp - TestMstarPolicy() End #######" << std::endl;

  return 1;
};


int TestMstar(){

  std::cout << "####### test_mstar.cpp - TestMstar() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  raplab::PlannerGraph* g_ptr ;
  raplab::SparseGraph g;

  timer.Start();
  g.AddVertex(0);
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddVertex(4);

  g.AddArc(0,1, std::vector<double>({11.3}) );
  g.AddArc(1,0, std::vector<double>({0.3}) );
  g.AddEdge(1,2, std::vector<double>({15.5}) );
  g.AddArc(2,3, std::vector<double>({15.5}) );
  g.AddEdge(3,4, std::vector<double>({16}) );
  g.AddArc(4,1, std::vector<double>({17.6}) );
  // g.AddEdge(1,7, std::vector<double>({9.9}) );

  g.AddArc(0,3, std::vector<double>({6}) );

  g_ptr = &g;

  auto p = raplab::Mstar();
  p.SetGraphPtr(g_ptr);

  auto starts = std::vector<long>({0,1});
  auto goals = std::vector<long>({3,4});
  p.Solve(starts, goals, 10, 1.0);
  auto plan = p.GetPlan(); 
  for (auto path: plan) {
    std::cout << " path = " << path << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### test_mstar.cpp - TestMstar() End #######" << std::endl;

  return 1;
};

int TestMstar2(){

  std::cout << "####### test_mstar.cpp - TestMstar2() Begin #######" << std::endl;

  // ------------static environment, 4x4, obstacle at (y=0,x=2), two robots---------
  //   x:0  1  2  3
  // y ---------------
  // 0 | * R1  #  *  |
  // 1 | R2 *  *  *  |
  // 2 | *  *  *  G2 |
  // 3 | *  *  G1 *  |
  //   ---------------
  //  Note: *=free, #=obstacle, R1,R2 = robot start, G1,G2 = robot goals
  // -------------------------------------------------------------------------------

  raplab::Grid2d g;
  int r = 4, c = 4;
  std::vector<std::vector<double> > occupancy_grid;
  occupancy_grid.resize(r);
  for (int i = 0; i < r; i++){
    occupancy_grid[i].resize(c, 0);
  }
  occupancy_grid[0][2] = 1;
  g.SetOccuGridPtr(&occupancy_grid);

  double time_limit = 300; // runtime limit.

  std::vector<long> starts({1,4}); // node id = y*NumX + x; e.g. (y=1)*4 + (x=0) = 4
                                   // the length of starts shows how many agents are there.
  std::vector<long> goals({14,11});

  raplab::Mstar planner;
  planner.SetGraphPtr(&g);
  planner.Solve(starts, goals, time_limit, 1.0);
  auto plan = planner.GetPlan();

  for (auto path: plan) {
    std::cout << " path = " << path << std::endl;
  }



  std::cout << "####### test_mstar.cpp - TestMstar2() End #######" << std::endl;

  return 1;

};