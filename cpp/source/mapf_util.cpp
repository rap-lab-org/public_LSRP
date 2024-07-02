
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mapf_util.hpp"

namespace raplab{

std::ostream& operator<<(std::ostream& os, const std::vector< std::vector<long> >& ps) {
  os << "{";
  for (size_t i = 0; i < ps.size(); i++) {
    os << "agent:" << i << ",path:{";
    for (auto& jj : ps[i]) {
      os << jj << ",";
    }
    os << "};";
  }
  os << "}.";
  return os;
};


void JointPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps) {
  if (jp.size() == 0) {
    std::cout << "[CAVEAT] JointPath2PathSet input joint path has size zero!" << std::endl;
    return ;
  }
  ps->clear();
  ps->resize(jp[0].size());
  for (int ri = 0; ri < jp[0].size(); ri++) {
    (*ps)[ri] = std::vector<long>();
    (*ps)[ri].resize(jp.size());
    for (size_t j = 0; j < jp.size(); j++){
      (*ps)[ri][j] = jp[j][ri];
    }
  }
  return ;
};


MAPFPlanner::MAPFPlanner() {} ;


MAPFPlanner::~MAPFPlanner() {} ;


void MAPFPlanner::SetGraphPtr(PlannerGraph* g) {
  _graph = g;
  _starts.clear();
  _goals.clear();
};


} // end namespace rzq
