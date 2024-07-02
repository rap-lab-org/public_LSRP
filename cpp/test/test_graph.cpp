
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "graph.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>


int TestSparseGraph();
int CompareArrayAndHash();
int TestHybridGraph();

int main(){
  // TestSparseGraph();
  // CompareArrayAndHash();
  TestHybridGraph();
  return 0;
};

int TestSparseGraph(){

  std::cout << "####### test_graph.cpp - TestSparseGraph() Begin #######" << std::endl;
  raplab::SimpleTimer timer;
  timer.Start();

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
  g.AddEdge(1,7, std::vector<double>({9.9}) );

  std::cout << g << std::endl;

  timer.PrintDuration();

  std::cout << "####### test_graph.cpp - TestSparseGraph() End #######" << std::endl;

  return 1;
};


int CompareArrayAndHash(){

  std::cout << "####### test_graph.cpp - CompareArrayAndHash() Begin #######" << std::endl;
  raplab::SimpleTimer timer;

  std::vector<long> aa({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17});
  std::vector<double> bb({10,20,30,40,50,60,70,80,90,100,11,12,13,14,15,16,17});
  std::unordered_map<long, double> cc;
  cc[1] = 10;
  cc[2] = 20;
  cc[3] = 30;
  cc[4] = 40;
  cc[5] = 50;
  cc[6] = 60;
  cc[7] = 70;
  cc[8] = 80;
  cc[9] = 90;
  cc[10] = 100;
  cc[11] = 11;
  cc[12] = 12;
  cc[13] = 13;
  cc[14] = 14;
  cc[15] = 15;
  cc[16] = 16;
  cc[17] = 17;

  int k = 17;
  int iters = 10000;
  double val = 0;

  timer.Start();
  for (int i = 0; i < iters; i++){
    for (int idx = 0; idx < aa.size(); idx++){
      if (aa[idx] == k) {
        val = aa[idx];
        break;
      }
    }
  }
  std::cout << "### using std::vector takes time: " << std::endl;
  timer.PrintDuration();


  timer.Start();
  for (int i = 0; i < iters; i++){
    val = cc[k];
  }
  std::cout << "### using std::unordered_map takes time: " << std::endl;
  timer.PrintDuration();

  std::cout << "####### test_graph.cpp - CompareArrayAndHash() End #######" << std::endl;

  return 1;
};


int TestHybridGraph(){

  std::cout << "####### test_graph.cpp - TestHybridGraph() Begin #######" << std::endl;
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
  hg.AddExtraEdge(190,200,raplab::InitVecType(1,100.0));
  hg.AddExtraEdge(201,207,raplab::InitVecType(1,100.0));

  auto succs = hg.GetSuccs(205);
  for (auto uu : succs){
    std::cout << "c(205," << uu << ") = " << hg.GetCost(205,uu) << std::endl;
  }

  succs = hg.GetSuccs(104);
  for (auto uu : succs){
    std::cout << "c(104," << uu << ") = " << hg.GetCost(104,uu) << std::endl;
  }

  auto preds = hg.GetPreds(201);
  for (auto uu : preds){
    std::cout << "c(201," << uu << ") = " << hg.GetCost(201,uu) << std::endl;
  }

  preds = hg.GetPreds(104);
  for (auto uu : preds){
    std::cout << "c(104," << uu << ") = " << hg.GetCost(104,uu) << std::endl;
  }

  std::cout << "####### test_graph.cpp - TestHybridGraph() End #######" << std::endl;

  return 1;
};
