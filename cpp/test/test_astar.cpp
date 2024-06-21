
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_astar.hpp"
#include "debug.hpp"

int TestAstar();
int TestAstarGrid2d();

int main(){

  // TestAstar();

  TestAstarGrid2d();
  
  return 0;
};

int TestAstar(){

  std::cout << "####### TestAstar() Begin #######" << std::endl;
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
  g.AddEdge(4,1, std::vector<double>({17.6}) );
  g.AddEdge(1,7, std::vector<double>({9.9}) );

  g.AddArc(0,3, std::vector<double>({6}) );

  g_ptr = &g;

  auto pp = raplab::Astar();
  pp.SetGraphPtr(g_ptr);
  auto p = pp.PathFinding(0,4);
  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### TestAstar() End #######" << std::endl;

  return 1;
};

int TestAstarGrid2d(){

  std::cout << "####### TestAstarGrid2d() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  raplab::Grid2d g;
  std::vector<std::vector<double> > occupancy_grid;
  occupancy_grid.resize(10);
  for (int i = 0; i < 10; i++){
    occupancy_grid[i].resize(10, 0);
  }
  for (int i = 0; i < 10; i++){
    if (i == 5){continue;}
    occupancy_grid[5][i] = 1;
  }
  g.SetOccuGridPtr(&occupancy_grid);

  timer.Start();
  auto pp = raplab::AstarGrid2d();
  pp.SetGraphPtr(&g);
  auto p = pp.PathFinding(0,99);

  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### TestAstarGrid2d() End #######" << std::endl;

  return 1;
};
