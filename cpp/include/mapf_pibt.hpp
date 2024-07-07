/*******************************************
 * Author: Shuai Zhou. original from Keisuke Okumura
 * A super fast rule based planning
 * All Rights Reserved.
 *******************************************/

#ifndef ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_
#define ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_

#include "mapf_util.hpp"
#include "union_find.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <random>

namespace raplab{

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
    int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps, std::unordered_map<long, std::unordered_map<long, float>>* dis_table);
    virtual std::vector<std::vector<long>> GetPlan(long nid = -1) override;
    virtual CostVec GetPlanCost(long nid = -1) override;
    virtual std::unordered_map<std::string, double> GetStats() override;
    void set_distable(const std::unordered_map<long, std::unordered_map<long, float>>* distable);

protected:
    bool PIBT(Agent* agent1, Agent* agent2, const std::vector<long>& Sfrom, std::vector<long>& Sto);
    std::unordered_map<long, std::unordered_map<long, float>> generateDistable();
    bool checkOccupied(long v, const std::vector<long>& Sto);
    Agent* mayPush(long v,const std::vector<long> &Sfrom, const std::vector<long>& Sto);


    PlannerGraph* graph_;
    std::mt19937 rng_;
    std::unordered_map<long, std::unordered_map<long, float>> dis_table_;
    std::vector<std::vector<long>> joint_policy_;
    double cost_;
    std::vector<Agent> agents_;
    std::vector<long> v_init_;
    std::vector<long>  v_f_;
};
}
#endif
