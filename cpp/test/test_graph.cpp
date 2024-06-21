
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

int main(){
  TestSparseGraph();
  // CompareArrayAndHash();
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

