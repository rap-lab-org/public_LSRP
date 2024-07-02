
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_dijkstra.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>


int TestDijkstra();

// int TestDijkstra2();

int TestDijkstraOnHybridGraph();

int main(){

  // TestDijkstra();
  
  // TestDijkstra2();

  TestDijkstraOnHybridGraph();
  
  return 0;
};

int TestDijkstra(){

  std::cout << "####### test_dijkstra.cpp - TestDijkstra() Begin #######" << std::endl;
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

  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  auto p = dijk.PathFinding(0,4);
  auto d_all = dijk.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  dijk.ExhaustiveBackwards(4);
  d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  dijk.ExhaustiveForwards(4);
  d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  timer.PrintDuration();

  std::cout << "####### test_dijkstra.cpp - TestDijkstra() End #######" << std::endl;

  return 1;
};

// int TestDijkstra2(){

//   std::cout << "####### test_dijkstra.cpp - TestDijkstra2() Begin #######" << std::endl;
//   raplab::SimpleTimer timer;
//   timer.Start();

//   raplab::Graph* g_ptr ;
//   raplab::LatticeXYA g;

//   timer.Start();

//   g.Init(3,3,8);
//   g.SetKNeighbor(3);

//   g_ptr = &g;

//   auto dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.PathFinding(0,40);
//   auto d_all = dijk.GetDistAll();
//   auto p = dijk.GetPath(40);
//   for (auto vv : p) {
//     std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
//   }
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.ExhaustiveBackwards(40);
//   d_all = dijk.GetDistAll();
//   std::cout << "-----------" << std::endl;
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.ExhaustiveForwards(0);
//   d_all = dijk.GetDistAll();
//   std::cout << "-----------" << std::endl;
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   timer.PrintDuration();

//   std::cout << "####### test_dijkstra.cpp - TestDijkstra2() End #######" << std::endl;

//   return 1;
// };


int TestDijkstraOnHybridGraph(){

  std::cout << "####### test_graph.cpp - TestDijkstraOnHybridGraph() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

  //
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
  g.AddEdge(3,4, std::vector<double>({15.9}) );
  g.AddEdge(4,1, std::vector<double>({17.6}) );
  // g.AddEdge(1,7, std::vector<double>({9.9}) );

  //
  raplab::Grid2d gg;
  int r = 10, c = 10;
  std::vector<std::vector<double> > occupancy_grid;
  occupancy_grid.resize(r);
  for (int i = 0; i < r; i++){
    occupancy_grid[i].resize(c, 0);
  }
  occupancy_grid[0][2] = 1;
  gg.SetOccuGridPtr(&occupancy_grid);
  gg.SetCostScaleFactor(10.0);

  //
  raplab::HybridGraph2d hg;
  hg.AddGrid2d(&gg);
  hg.AddGrid2d(&gg);
  hg.AddSparseGraph(&g);
  hg.AddSparseGraph(&g);

  std::cout << hg.GetSuccs(4) << std::endl;
  std::cout << hg.GetSuccs(104) << std::endl;
  std::cout << hg.GetSuccs(200) << std::endl;
  std::cout << hg.GetSuccs(205) << std::endl;

  hg.AddExtraEdge(10,104,raplab::InitVecType(1,100.0));
  hg.AddExtraEdge(104,10,raplab::InitVecType(1,100.0));
  hg.AddExtraEdge(193,200,raplab::InitVecType(1,100.0));
  hg.AddExtraEdge(204,207,raplab::InitVecType(1,100.0));


  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(&hg);
  dijk.ExhaustiveBackwards(207);
  auto d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  auto p = dijk.PathFinding(0,209);
  std::cout << " path(0,207) = " << p << std::endl;

  timer.PrintDuration();

  std::cout << "####### test_graph.cpp - TestDijkstraOnHybridGraph() End #######" << std::endl;

  return 1;
};
