/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapf_util.hpp"
#include "mapf_mstar.hpp"
#include "union_find.hpp"
#include "search_astar.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include "mapf_pibt.hpp"
#include <mapf_pibt.hpp>
#include <deque>
#include <algorithm>

namespace raplab{

// _Agent
_Agent::_Agent(long id, double priority) :id(id),priority(priority),init_priority(priority){}

void _Agent::setPriority(double pri) {
    priority = pri;
}

bool _Agent::operator<(const _Agent &other) const {
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
    double gap = 1.0 / (starts.size() + 1);
    for (size_t i = 0; i < starts.size(); ++i) {
        agents_.emplace_back(i, i * gap);
    }
    debug_counter = 0;
    //std::cout<<"debugging True solve"<<std::endl;
    // Main function
    while (true) {
        if (DEBUG_PIBT){
            debug_counter += 1;
            std::cout<< "counting:"<<debug_counter<<std::endl;
        }
        std::vector<long> Sfrom = joint_policy_.back();
        std::vector<long> Sto(Sfrom.size(), -1);

        if (Sfrom == goals) {
            // find solution
            if (DEBUG_PIBT){std::cout<<"solution found"<<std::endl;}
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
        std::sort(agents_.begin(), agents_.end(), [](const _Agent &a1, const _Agent &a2) {
            return a1.priority > a2.priority;
        });
            // Pibt planning
        for (auto &agent: agents_) {
            if (Sto[agent.id] == -1) {
                if (Swap) {
                    PIBT_SWAP(&agent, nullptr,Sfrom,Sto);
                } else {
                    PIBT(&agent, nullptr, Sfrom, Sto);
                }
            }
        }
        // if Sto already exist in the joint_policy: fail
        /*
        if (std::find(joint_policy_.begin(), joint_policy_.end(), Sto) != joint_policy_.end()) {
            // Fail case
            //std::cout<<"fail bro, gotta do some"<<std::endl;
            return -1; // Return appropriate fail value
        }
         */
        joint_policy_.push_back(Sto);
        if (DEBUG_PIBT) {
            for (long v: Sto) {
                    std::cout<<v<<" ";}
        }
        std::cout<<std::endl;
    }
}

bool Pibt::PIBT(_Agent *agent1, _Agent *agent2, const std::vector<long> &Sfrom, std::vector<long> &Sto) {
    std::vector<long> C = _graph->GetSuccs(Sfrom[agent1->id]);
    C.push_back(Sfrom[agent1->id]);
    std::shuffle(C.begin(), C.end(), rng_);
    //sort node by dis_table value
    auto &policy = _policies.at(agent1->id);
    std::sort(C.begin(), C.end(), [&](long a, long b) {
        return policy.H(a) < policy.H(b);
    });

    for (long v: C){
        if(checkOccupied(v, Sto)){
            continue;
        }
        if(agent2 != nullptr && Sfrom[agent2->id] == v){
            continue;
        }
        Sto[agent1->id] = v;
        _Agent* a2 = mayPush(v, Sfrom, Sto);
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

    bool Pibt::PIBT_SWAP(_Agent *agent1, _Agent *agent2, const std::vector<long> &Sfrom, std::vector<long> &Sto) {
        std::vector<long> C = _graph->GetSuccs(Sfrom[agent1->id]);
        C.push_back(Sfrom[agent1->id]);
        std::shuffle(C.begin(), C.end(), rng_);
        //sort node by dis_table value
        auto &policy = _policies.at(agent1->id);
        std::sort(C.begin(), C.end(), [&](long a, long b) {
            return policy.H(a) < policy.H(b);
        });
        // swap part
        auto aj = swap_possible_required(agent1,Sfrom,Sto,C);
        if (aj != nullptr) {
            std::reverse(C.begin(), C.end());
        }
        for (long v: C){
            if(checkOccupied(v, Sto)){
                continue;
            }
            if(agent2 != nullptr && Sfrom[agent2->id] == v){
                continue;
            }
            Sto[agent1->id] = v;
            _Agent* a2 = mayPush(v, Sfrom, Sto);
            if (a2 != nullptr){
                if(!PIBT_SWAP(a2, agent1, Sfrom, Sto)){
                    continue;
                }
            }
            // swap part
            if (v == C[0] && aj != nullptr && Sto[aj->id] == -1) {Sto[aj->id] = Sfrom[agent1->id];}
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
    return stats;
}
void Pibt::set_policy(const std::unordered_map<int, MstarPolicy> *policy) {
    if (policy != nullptr) {
        _policies = *policy;
    } else {
        //std::cout<<"debugging generate Distable"<<std::endl;
        generatePolicy();
    }
}


// Check if the node is occupied by some agents
bool Pibt::checkOccupied(long v, const std::vector<long> &Sto) {
    return std::any_of(agents_.begin(), agents_.end(), [&](const _Agent& a) {
        return Sto[a.id] == v;
    });
}

// Check if any low priority agent take the place and needs to plan
_Agent *Pibt::mayPush(long v, const std::vector<long> &Sfrom, const std::vector<long> &Sto) {
    auto it = std::find_if(agents_.begin(), agents_.end(), [&](const _Agent& a) {
        return Sfrom[a.id] == v && Sto[a.id] == -1;
    });

    if (it != agents_.end()) {
        return &(*it);
    } else {
        return nullptr;
    }
}

_Agent *Pibt::swap_possible_required(_Agent *agent1, const std::vector<long> &Sfrom, const std::vector<long> &Sto,
                                    std::vector<long> C) {
    if (C[0] == Sfrom[agent1->id]) {return nullptr;}
    auto aj = occupied_now(C[0],Sfrom);
    if ((aj != nullptr) && (Sto[aj->id] == -1) && (
    is_swap_required(agent1,aj,Sfrom[agent1->id],Sfrom[aj->id],Sfrom,Sto)) &&(
    is_swap_possible(Sfrom[aj->id],Sfrom[agent1->id],Sfrom,Sto))) {
        return aj;
    }

    for (long u : _graph->GetSuccs(Sfrom[agent1->id])) {
        auto ak = occupied_now(u,Sfrom);
        if(ak == nullptr || C[0] == Sfrom[ak->id]) { continue;}
        if (is_swap_required(ak,agent1,Sfrom[agent1->id],C[0],Sfrom,Sto)&&
            is_swap_possible(C[0],Sfrom[agent1->id],Sfrom,Sto)) {return ak;}
    }
    return nullptr;
}

    bool Pibt::is_swap_required(_Agent *pusher, _Agent *puller, long v_pusher_origin, long v_puller_origin,
                                const std::vector<long> &Sfrom, const std::vector<long> &Sto) {
    long v_pusher = v_pusher_origin;
    long v_puller = v_puller_origin;
    long tmp;
    auto &policy_pusher = _policies.at(pusher->id);
    auto &policy_puller = _policies.at(puller->id);
    while (policy_pusher.H(v_puller) < policy_pusher.H(v_pusher)) {
        std::vector<long> nghs = _graph->GetSuccs(v_puller);
        auto n = nghs.size();
        for (long u: nghs) {
            auto a = occupied_now(u,Sfrom);
            if (u == v_pusher ||
                    (_graph->GetSuccs(u)).size() == 1 && a != nullptr && u == Sto[a->id]) {
                --n;
            } else {
                tmp = u;
            }
        }
        if (n >= 2){return false;}
        if (n <= 0){ break;}
        v_pusher = v_puller;
        v_puller = tmp;
    }
        return (policy_puller.H(v_pusher) < policy_puller.H(v_puller)) &&
                (policy_pusher.H(v_pusher)[0] == 0 ||
                policy_pusher.H(v_puller) < policy_pusher.H(v_pusher));
    }

    bool Pibt::is_swap_possible(long v_pusher_origin, long v_puller_origin, const std::vector<long> &Sfrom,
                                const std::vector<long> &Sto) {
        long v_pusher = v_pusher_origin;
        long v_puller = v_puller_origin;
        long tmp;
        while (v_puller != v_pusher_origin) {
            std::vector<long> nghs = _graph->GetSuccs(v_puller);
            auto n = nghs.size();
            for (long u: nghs) {
                auto a = occupied_now(u,Sfrom);
                if (u == v_pusher ||
                    (_graph->GetSuccs(u)).size() == 1 && a != nullptr && u == Sto[a->id]) {
                    --n;
                } else {
                    tmp = u;
                }
            }
            if (n >= 2){return true;}
            if (n <= 0){return false;}
            v_pusher = v_puller;
            v_puller = tmp;
        }
        return false;
    }

_Agent *Pibt::occupied_now(long v, const std::vector<long> &Sfrom) {
    for (int i = 0;i < agents_.size(); i++) {
        if (Sfrom[agents_[i].id] == v){
            return &agents_[i];
        }
    }
    return nullptr;
}

bool Pibt::generatePolicy() {
    _policies.clear();
    for (int ri = 0; ri < v_init_.size(); ri++) {
        _policies[ri] = MstarPolicy();
        _policies[ri].SetGraphPtr(_graph);
        //double remain_time = _tlimit - std::chrono::duration<double>(
        //        std::chrono::steady_clock::now() - _t0).count();
        bool good = _policies[ri].Compute(v_f_[ri], _tlimit);
        if (!good) { // timeout for policy computation.
            return false;
        }
    }
    return true;
}

//  solve function
int Pibt::_Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps,
                 std::unordered_map<int, MstarPolicy> *policies) {
     //std::cout<<"debugging Main solve"<<std::endl;
     v_init_ = starts;
     v_f_ = goals;
     _tlimit = time_limit;
     set_policy(policies);
     return  Solve(starts, goals, time_limit, eps);
}

}











