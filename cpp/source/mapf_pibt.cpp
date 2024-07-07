/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapf_util.hpp"
#include "union_find.hpp"
#include "search_astar.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <mapf_pibt.hpp>
#include <deque>
#include <algorithm>

namespace raplab{

// Agent
Agent::Agent(long id, double priority) :id(id),priority(priority),init_priority(priority){}

void Agent::setPriority(double pri) {
    priority = pri;
}

bool Agent::operator<(const Agent &other) const {
    return priority < other.priority;
}

////////////////////////////////////////////////////////////////////
////////////////////// Pibt ////////////////////////////////////////
////////////////////////////////////////////////////////////////////

Pibt::Pibt() {}

Pibt::~Pibt() {}

int Pibt::Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps) {
    joint_policy_.clear();
    joint_policy_.push_back(starts);
// initialize agents priority
    agents_.clear();
    double gap = 1 / (starts.size() + 1);
    for (size_t i = 0; i < starts.size(); ++i) {
        agents_.emplace_back(i, 1 - i * gap);
    }

// Main function
    while (true) {
        std::vector<long> Sfrom = joint_policy_.back();
        std::vector<long> Sto(Sfrom.size(), -1);

        if (Sfrom == goals) {
            // find solution
            return 1;
        }
        // update priority
        for (auto &a: agents_) {
            if (Sfrom[a.id] == goals[a.id]) {
                a.priority = a.init_priority;
            } else {
                a.priority += 1;
            }
        }

        // sort agents
        std::sort(agents_.begin(), agents_.end(), [](const Agent &a1, const Agent &a2) {
            return a1.priority > a2.priority;
        });
            // Pibt planning
        for (auto &agent: agents_) {
            if (Sto[agent.id] == -1) {
                PIBT(&agent, nullptr, Sfrom, Sto);
            }
        }
        // if Sto already exist in the joint_policy: fail
        if (std::find(joint_policy_.begin(), joint_policy_.end(), Sto) != joint_policy_.end()) {
            // Fail case
            return -1; // Return appropriate fail value
        }
        joint_policy_.push_back(Sto);
    }
}

bool Pibt::PIBT(Agent *agent1, Agent *agent2, const std::vector<long> &Sfrom, std::vector<long> &Sto) {
    std::vector<long> C = graph_->GetSuccs(Sfrom[agent1->id]);
    C.push_back(Sfrom[agent1->id]);
    std::shuffle(C.begin(), C.end(), rng_);
    //sort node by dis_table value
    std::sort(C.begin(), C.end(), [&](long a, long b) {
        return dis_table_[agent1->id].at(a) < dis_table_[agent1->id].at(b);
    });

    for (long v: C){
        if(checkOccupied(v, Sto)){
            continue;
        }
        if(agent2 != nullptr && Sfrom[agent2->id] == v){
            continue;
        }
        Sto[agent1->id] = v;
        Agent* a2 = mayPush(v, Sfrom, Sto);
        if (a2 != nullptr){
            if(!PIBT(a2, agent1, Sfrom, Sto)){
                continue;
            }
        }
        return true;
    }
    Sto[agent1->id] = Sfrom[agent1->id];
    return false;
}

std::vector<std::vector<long>> Pibt::GetPlan(long nid) {
    return joint_policy_;
}

CostVec Pibt::GetPlanCost(long nid) {
    // Return SOC Cost of Plan
    CostVec cost_vec;
    if (joint_policy_.empty()){
        return cost_vec;
    }
    // initialize
    std::vector<long> Sfrom = joint_policy_[0];
    double cost = 0.0;

    for (size_t i = 1; i < joint_policy_.size(); ++i) {
        const std::vector<long>& Sto = joint_policy_[i];
        for (size_t index = 0; index < Sto.size(); ++index) {
            if (Sto[index] == Sfrom[index] && Sfrom[index] == v_f_[index]) {
                continue;
            }
            cost += 1.0;
        }
        Sfrom = Sto;
    }

    // Add in cost_vec
    cost_vec.push_back(cost);
    return cost_vec;
}

// I dont know what it is
std::unordered_map<std::string, double> Pibt::GetStats() {
    std::unordered_map<std::string, double> stats;
    // Populate stats
    return stats;
}
void Pibt::set_distable(const std::unordered_map<long, std::unordered_map<long, float>>* distable) {
    if (distable != nullptr) {
        dis_table_ = *distable;
    } else {
        dis_table_ = generateDistable();
    }
}


// Check if the node is occupied by some agents
bool Pibt::checkOccupied(long v, const std::vector<long> &Sto) {
    return std::any_of(agents_.begin(), agents_.end(), [&](const Agent& a) {
        return Sto[a.id] == v;
    });
}

// Check if any low priority agent take the place and needs to plan
Agent *Pibt::mayPush(long v, const std::vector<long> &Sfrom, const std::vector<long> &Sto) {
    auto it = std::find_if(agents_.begin(), agents_.end(), [&](const Agent& a) {
        return Sfrom[a.id] == v && Sto[a.id] == -1;
    });

    if (it != agents_.end()) {
        return &(*it);
    } else {
        return nullptr;
    }
}

std::unordered_map<long, std::unordered_map<long, float>> Pibt::generateDistable() {
    std::unordered_map<long, std::unordered_map<long, float>> dist_table;

    for (size_t i = 0; i < v_init_.size(); ++i) {
        std::deque<long> tmp;
        tmp.push_back(v_f_[i]);

        float inf = std::numeric_limits<float>::infinity();
        std::unordered_map<long, float> dist_map;
        dist_map[v_f_[i]] = 0;  // 设置初始点距离为0

        while (!tmp.empty()) {
            long curr = tmp.front();
            tmp.pop_front();

            for (const auto &neigh: graph_->GetSuccs(curr)) {
                if (dist_map.find(neigh) == dist_map.end() || dist_map[neigh] > dist_map[curr] + 1) {  // 更新邻居的距离
                    dist_map[neigh] = dist_map[curr] + 1;
                    tmp.push_back(neigh);
                }
            }
        }

        dist_table[v_f_[i]] = dist_map;
    }
    return dist_table;
}

// True solve function
int Pibt::Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps,
                    std::unordered_map<long, std::unordered_map<long, float>> *dis_table) {
     v_init_ = starts;
     v_f_ = goals;
     set_distable(dis_table);
     Solve(starts, goals,time_limit,eps);
}


}











