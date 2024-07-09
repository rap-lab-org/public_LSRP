/*******************************************
 * Author: Shuai Zhou. original from Keisuke Okumura
 * A super fast rule based planning
 * All Rights Reserved.
 *******************************************/

#ifndef SHUAIZHOU_BASIC_MAPF_Pibt_H_
#define SHUAIZHOU_BASIC_MAPF_Pibt_H_

#include "mapf_util.hpp"
#include "mapf_pibt.hpp"
#include "union_find.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include "mapf_mstar.hpp"

namespace raplab{
#define DEBUG_PIBT 0

struct Agent
{
    long id;
    double priority;
    double init_priority;

    Agent(long id, double priority);

    void setPriority(double pri);

    bool operator<(const Agent& other) const;

};

class Pibt: public MAPFPlanner
{
public:

    Pibt() ;

    virtual ~Pibt() ;

    virtual int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) override ;
    int _Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps, std::unordered_map<int, MstarPolicy>* policies);
    virtual std::vector<std::vector<long>> GetPlan(long nid = -1) override;
    virtual CostVec GetPlanCost(long nid = -1) override;
    virtual std::unordered_map<std::string, double> GetStats() override;
    void set_policy(const std::unordered_map<int, MstarPolicy>* policy);
    bool generatePolicy();

protected:
    bool PIBT(Agent* agent1, Agent* agent2, const std::vector<long>& Sfrom, std::vector<long>& Sto);
    bool checkOccupied(long v, const std::vector<long>& Sto);
    Agent* mayPush(long v,const std::vector<long> &Sfrom, const std::vector<long>& Sto);

    std::mt19937 rng_;
    std::unordered_map<int, MstarPolicy> _policies;
    std::vector<std::vector<long>> joint_policy_;
    double cost_;
    std::vector<Agent> agents_;
    std::vector<long> v_init_;
    std::vector<long>  v_f_;
    double _tlimit;
    int debug_counter;
    std::chrono::time_point<std::chrono::steady_clock> _t0;
};
}
#endif
