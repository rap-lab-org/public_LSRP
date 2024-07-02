
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_MAPF_UTIL_H_
#define ZHONGQIANGREN_BASIC_MAPF_UTIL_H_

#include "graph.hpp"
#include "type_def.hpp"
#include <chrono>
#include <unordered_map>

namespace raplab{

/**
 *
 */
std::ostream& operator<<(std::ostream& os, const std::vector< std::vector<long> >& joint_path) ;

/**
 *
 */
typedef std::vector< std::vector<long> > PathSet; // nickname, make the name shorter
// #define PathSet std::vector< std::vector<long> >;


void JointPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps) ;

/**
 * @brief
 */
class MAPFPlanner {
public:
  /**
   * @brief
   */
  MAPFPlanner() ;
  /**
   * @brief
   */
  virtual ~MAPFPlanner() ;
  /**
   *
   */
  virtual void SetGraphPtr(PlannerGraph* g) ;
  /**
   * @brief
   */
  virtual int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) = 0;
  /**
   * @brief
   */
  virtual PathSet GetPlan(long nid=-1) = 0;
  /**
   * @brief
   */
  virtual CostVec GetPlanCost(long nid=-1) = 0;
  /**
   * @brief Get statistics as a vector of double float numbers.
   */
  virtual std::unordered_map<std::string, double> GetStats() = 0;

protected:

  PlannerGraph* _graph;
  std::vector<long> _starts;
  std::vector<long> _goals;
  double _time_limit = 0;
};

} // end namespace rzq


#endif  // ZHONGQIANGREN_BASIC_MAPF_UTIL_H_
