
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_
#define ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_

#include "mapf_util.hpp"
#include "union_find.hpp"
#include "search_astar.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace raplab{

#define DEBUG_MSTAR 0
#define MAX_NGH_SIZE 1e7

/**
 * @brief
 */
struct MState
{
  long id = 0;
  std::vector<long> jv; // joint vertex
  CostVec g; // cost vector
  // std::string jv2str() ;
  std::unordered_map<int, int> colSet; // collision set
};

inline std::string jv2str(const std::vector<long>& jv) {
  std::string out("(");
  for (size_t i = 0; i < jv.size(); i++){
    out += std::to_string(jv[i])+",";
  }
  out += ")";
  return out;
};

inline bool jvEqual(const std::vector<long>& jv, const std::vector<long>& jv2) {
  for (size_t i = 0; i < jv.size(); i++){
    if (jv[i] != jv2[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief verify whether collision set a is a subset of b.
 * This is subset check as in M*, not as in rM* !
 */
bool IsColSetSubset(
  const std::unordered_map<int,int>& a, const std::unordered_map<int,int>& b) ;

/**
 * @brief take the combination by select an element from every 
 *  sub-vector in the input vec.
 */
template <typename DataType>
void TakeCombination(
  const std::vector< std::vector<DataType> >&, std::vector< std::vector<DataType> >*) ;

/**
 * @brief
 */
class MstarPolicy : public Dijkstra {
public:
  /**
   *
   */
  MstarPolicy() ;
  /**
   *
   */
  virtual ~MstarPolicy() ;
  /**
   *
   */
  virtual bool Compute(long vd, double time_limit) ;
  /**
   * @brief for debug
   */
  virtual void Print() ;
  /**
   * @brief get the next move.
   */
  virtual long Phi(const long& v) ;
  /**
   * @brief get heuristic vector, component-wise underestimate.
   */
  virtual CostVec H(const long& v) ;
protected:
  std::unordered_map<long, long > _phi;
  std::unordered_map<long, CostVec > _h;

};

/**
 * @brief
 */
class Mstar : public MAPFPlanner
{
public:
  /**
   *
   */
  Mstar() ;
  /**
   *
   */
  virtual ~Mstar() ;
  /**
   * @brief 
   */
  virtual int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) override ;
  /**
   * @brief
   */
  virtual std::vector< std::vector<long> > GetPlan(long nid=-1) override ;
  /**
   * @brief
   */
  virtual CostVec GetPlanCost(long nid=-1) override ;
  /**
   * @brief Get statistics as a vector of double float numbers.
   */
  virtual std::unordered_map<std::string, double> GetStats() override ;
protected:
  /**
   * @brief 
   */
  virtual bool _GetLimitNgh(const long& sid, std::vector< std::vector<long> >* out);
  /**
   * @brief 
   */
  inline long _GenId() { return _id_gen++; };
  /**
   * @brief. Return false if timeout.
   */
  virtual bool _Init() ;
  /**
   * @brief 
   */
  virtual CostVec _H(const std::vector<long>& jv) ;
  /**
   * @brief 
   */
  virtual bool _IfReachGoal(const std::vector<long>& jv) ;
  /**
   * @brief 
   */
  std::unordered_map<int,int> _ColCheck(
    const std::vector<long>& jv1, const std::vector<long>& jv2) ;
  /**
   * @brief Add sid2 to sid1.back_set
   */
  virtual void _AddBackSet(const long& sid1, const long& sid2) ;
  /**
   * @brief Recursive backprop of collision set.
   */
  virtual void _BackProp(const long& sid, const std::unordered_map<int,int>& col_set) ;
  /**
   * @brief union the collision set.
   */
  virtual void _ColSetUnion(
    const std::unordered_map<int,int>& a, std::unordered_map<int,int>* b) ;
  /**
   * @brief reopen a state.
   */
  virtual void _Reopen(const long& sid) ;
  /**
   * @brief get transition cost vector, summed over all agents.
   */
  virtual CostVec _GetTransCost(
    const std::vector<long>& jv1, const std::vector<long>& jv2, long curr_sid) ;
  /**
   * @brief Compare whether a newly generate state has a cost vector that is dominated by some solution found.
   */
  // virtual bool _SolFilter(const CostVec&);
  /**
   * @brief Compare whether a newly generate state should be pruned or not.
   */
  // virtual bool _DomCompare(const std::vector<long>&, const CostVec&, std::vector<long>*) ; 
  /**
   *
   */
  // virtual void _PostProcessResult() ;
  /**
   *
   */
  virtual void _BuildJointPath(long sid, std::vector< std::vector<long> >* jp) ;
  /**
   *
   */
  // virtual void _JPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps) ;
  /**
   * @brief. Compute the cost of a special case where robot stays at the goal for a while and then moves away.
   */
  virtual CostVec _MoveFromGoalCost(const int& ri, long sid) ;

  size_t _nAgent = 0;
  std::chrono::time_point<std::chrono::steady_clock> _t0;
  double _tlimit; // the allowed time for planning.
  std::vector<long> _vo;
  std::vector<long> _vd;
  PlannerGraph* _g;
  std::unordered_map<int, MstarPolicy> _policies;
  long _id_gen = 1;
  std::unordered_map<long, MState> _states;
  std::unordered_map<long,long> _parent; // map a state id to its parent state id.
  std::unordered_map<long, std::unordered_set<long> > back_set_map_;
  std::set< std::pair< CostVec, long> > _open; // <cost vector, state id>
  // std::unordered_map< std::string, std::unordered_set<long> > _frontiers; 
  std::unordered_map< std::string, long > _best; 
  std::vector<long> _sol;
  // std::unordered_map<int, CostVec> _agentCosts;
  std::unordered_map<std::string, double> _stats;
  // MOMAPFResult _result;
  double _wait_cost = 1;
  long _reached_goal_id = -1;
  double _wH = 1.0;
  std::vector< std::vector<long> > _sol_path;

};


} // end namespace rzq


#endif  // ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_
