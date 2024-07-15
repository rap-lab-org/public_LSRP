
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * Organization: Raplab
 * All Rights Reserved. 
 *******************************************/

#ifndef RAPLAB_BASIC_MAPFAA_UTIL_H_
#define RAPLAB_BASIC_MAPFAA_UTIL_H_

#include "graph.hpp"
#include "type_def.hpp"
#include <chrono>
#include <unordered_map>

namespace raplab{


struct TimePath
{
  std::vector<long> nodes;
  std::vector<double> times;
};

struct TimePathSet
{
  std::vector<TimePath> paths;
};


/**
 * @brief
 */
class MAPFAAPlanner {
  /**
   * @brief
   */
  MAPFAAPlanner() ;
  /**
   * @brief
   */
  virtual ~MAPFAAPlanner() ;
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
  virtual TimePathSet GetPlan(long nid=-1) = 0;
  /**
   * @brief
   */
  virtual CostVec GetPlanCost(long nid=-1) = 0;
  /**
   * @brief Get statistics as a vector of double float numbers.
   */
  virtual std::unordered_map<std::string, double> GetStats() = 0;

};

} // end namespace raplab


#endif  // RAPLAB_BASIC_MAPFAA_UTIL_H_
