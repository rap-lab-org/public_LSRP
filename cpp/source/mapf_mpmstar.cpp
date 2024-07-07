/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "mapf_mpmstar.hpp"
#include "mapf_mstar.hpp"
#include "union_find.hpp"

namespace raplab{


////////////////////////////////////////////////////////////////////
////////////////////// Mstar ///////////////////////
////////////////////////////////////////////////////////////////////

    MPMstar::MPMstar() {};

    MPMstar::~MPMstar() {};

    // virtual int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) = 0;

    int MPMstar::Solve(
            std::vector<long>& starts, std::vector<long>& goals, double time_limit, double wH)
    {
        // init search
        _vo = starts;
        _vd = goals;
        _tlimit = time_limit;
        _wH = wH;
        _nAgent = _vo.size();
        // Pibt planner
        planner.SetGraphPtr(_graph);
        _Init();
        if ( DEBUG_MPMstar ) { std::cout << "[DEBUG] after init..." << std::endl; }

        // main while loop
        while ( true ) {

            _stats["num_iter"] += 1;

            // check termination
            // if ( _open.empty() ) {
            //   if (_sol.size() > 0) {
            //     std::cout << "[DEBUG] MPMstar::Search, find all pareto !" << std::endl;
            //     _result.find_all_pareto = true;
            //   }
            //   break;
            // }
            // check timeout
            if ( std::chrono::duration<double>(std::chrono::steady_clock::now() - _t0).count() > _tlimit ) {
                std::cout << "[INFO] MPMstar::Search times out!" << std::endl;
                break; // time out!
            }

            // pop from open
            MState& s = _states[ _open.begin()->second ];
            _open.erase(_open.begin());


            // check for solution
            if (_IfReachGoal(s.jv)) {
                _stats["success"] = 1;
                _reached_goal_id = s.id;
                // _sol.push_back(s.id); // new solution found.
                // if (_sol.size() == 1) {
                //   // first sol
                //   _result.rt_firstSol = std::chrono::duration<double>(
                //     std::chrono::steady_clock::now() - _t0).count();
                // }
                // continue;
                if ( DEBUG_MPMstar ) {std::cout << "[INFO] MPMstar finds a solution ! " << std::endl;}
                break;
            }
            // std::cout << " will expand " << std::endl;

            // expand
            std::vector< std::vector<long> > nghs;
            bool success = _GetLimitNgh(s.id, &nghs);
            // std::cout << " after get limited ngh" << std::endl;
            if (!success) {
                break;
            }
            // _result.n_expanded++;

            _stats["num_exp"] += 1;
            for (auto& ngh : nghs) { // loop over all limited neighbors.

                auto colSet = _ColCheck(s.jv, ngh);
                if (colSet.size() > 0) { // there is collision.
                    _BackProp(s.id, colSet);
                    continue;
                }

                // collision-free
                // get cost vector
                auto cuv = _GetTransCost(s.jv, ngh, s.id);
                CostVec g_ngh = s.g + cuv;
                CostVec f_ngh = g_ngh + _H(ngh); // note that _wH is considered within _H()
                // if ( _SolFilter(f_ngh) ) { // dominated by some already found solution.
                //   continue;
                // }

                // // dom check
                // std::vector<long> dom_ids;
                // if ( _DomCompare(ngh, g_ngh, &dom_ids) ) {
                //   // do dom-back-prop.
                //   for (const auto& dom_id : dom_ids) {
                //     _BackProp(s.id, _states[dom_id].colSet);
                //     _AddBackSet(s.id, dom_id);
                //   }
                // }

                auto ngh_str = jv2str(ngh);
                if ( (_best.find( ngh_str ) != _best.end()) &&
                     (_states[_best[ngh_str]].g[0] <= g_ngh[0] ) )
                {
                    continue;
                }

                // add to open.
                long curr_id = _GenId();
                _states[curr_id] = MState();
                _states[curr_id].id = curr_id;
                _states[curr_id].jv = ngh;
                _states[curr_id].g = g_ngh;
                _states[curr_id].colSet = std::unordered_map<int, int>(); // collision set, empty set.
                _open.insert( std::make_pair(f_ngh, curr_id) );
                _stats["num_gen"] += 1;
                // _result.n_generated++;
                // if (_best.find(ngh_str) == _best.end()) {
                //   _best[ngh_str] = std::unordered_set<long>();
                // }
                _best[ngh_str] = curr_id;
                _AddBackSet(s.id, curr_id);
                _parent[curr_id] = s.id; // track parent.

            }
        } // end while loop

        std::vector< std::vector<long> > jp;
        _BuildJointPath(_reached_goal_id, &jp);
        JointPath2PathSet(jp, &_sol_path);

        _stats["search_time"] = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - _t0).count();

        // _PostProcessResult();
        return 1;
    };

    std::vector< std::vector<long> > MPMstar::GetPlan(long nid) {
        // the input nid is useless.
        return _sol_path;
    };

    CostVec MPMstar::GetPlanCost(long nid) {
        // the input nid is useless.
        return _states[_reached_goal_id].g;
    };

    std::unordered_map<std::string, double> MPMstar::GetStats() {
        return _stats;
    } ;
    void MPMstar::_GetNgh(const long &sid, std::vector<std::vector<long>> *out) {
        out->clear();
        const auto& colSet = _states[sid].colSet;
        if(!colSet.empty()){
            std::vector<std::vector<long>> pibt_policy = Pibt_process(colSet, sid);
            if(pibt_policy.empty()){
                _GetLimitNgh(sid,out);
                return;
            }else{

            }
        } else {
            std::pair<std::unordered_map<int, int>, std::vector<std::vector<long>>> col_policy = _Pibt_policy[sid];
            //if (col_policy)
        }
    }

    std::vector<std::vector<long>> MPMstar::Pibt_process(const std::unordered_map<int, int> colSet, const long &sid) {
        std::vector<std::vector<long>> pibt_policy;
        std::vector<long> start;
        std::vector<long> goal;
        // generate folloing policy
        std::unordered_map<int, MstarPolicy> colPolicies;
        int newId = 0;
        for (long agentId : colSet) {
            colPolicies[newId] = _policies.at(agentId);
            ++newId;
        }

        return pibt_policy;
    }

    void MPMstar::_GetPibtNgh(std::pair<std::unordered_map<int, int>, std::vector<std::vector<long>>> colSet_and_vec,
                              const long &sid, std::vector<std::vector<long>> *out) {
        return;
    };

    bool MPMstar::_GetLimitNgh(const long& sid, std::vector< std::vector<long> >* out)
    {
        out->clear();
        const auto& colSet = _states[sid].colSet;
        // if (colSet.size() > MAX_COLSET_SIZE) {
        //   std::cout << "[WARNING] MPMstar::_GetLimitNgh, col set of size " << colSet.size() << " too large !" << std::endl;
        //   return false;
        // }
        const auto& jv = _states[sid].jv;
        std::vector< std::vector<long> > ngh_vec;
        ngh_vec.resize(_nAgent);
        long ngh_size = 1;
        for (int ri = 0; ri < _nAgent; ri++){ // loop over robots
            if (colSet.find(ri) == colSet.end()){ // not in collision
                // ngh_size *= _policies[ri].Phi(jv[ri]).size();
                // for (const auto& u : _policies[ri].Phi(jv[ri]) ){
                //   ngh_vec[ri].push_back(u);
                // }
                ngh_vec[ri].push_back(_policies[ri].Phi(jv[ri]));
            }else{ // in collision
                auto idvl_nghs = _graph->GetSuccs(jv[ri]);
                ngh_size *= idvl_nghs.size();
                for (const auto& u : idvl_nghs) {
                    ngh_vec[ri].push_back(u);
                }
                ngh_vec[ri].push_back(jv[ri]); // wait in place
            }
        }


        // std::cout << " take combi" << std::endl;
        TakeCombination(ngh_vec, out);
        // std::cout << " take combi done" << std::endl;
        return true;
    };


    bool MPMstar::_Init() {

        if ( _nAgent != _vd.size() ){
            throw std::runtime_error("[ERROR] MPMstar::_Init, _vo.size() != _vd.size() !") ;
        }

        // set up timer
        _t0 = std::chrono::steady_clock::now(); // for timing.
        // _result.n_generated = 0;
        // _result.n_expanded = 0;
        // _result.rt_initHeu = 0.0;
        // _result.n_maxColSet = 0;
        // _result.n_branch = 0;

        _reached_goal_id = -1;
        _stats["success"] = 0;
        _stats["num_iter"] = 0;
        _stats["num_exp"] = 0;
        _stats["num_gen"] = 0;
        _stats["search_time"] = -1;

        // init policies.
        for (int ri = 0; ri < _nAgent; ri++) {
            if ( DEBUG_MPMstar ) { std::cout << "[DEBUG] MPMstar::_Init, construct policy for agent " << ri << std::endl; }
            _policies[ri] = MstarPolicy();
            _policies[ri].SetGraphPtr(_graph);
            double remain_time = _tlimit - std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - _t0).count();
            bool good = _policies[ri].Compute(_vd[ri], remain_time);
            if (!good) { // timeout for policy computation.
                return false;
            }
        }
        // _result.rt_initHeu = std::chrono::duration<double>(
        //     std::chrono::steady_clock::now() - _t0).count();

        // init search
        long curr_id = _GenId();
        _parent[curr_id] = -1;
        _states[curr_id] = MState();
        _states[curr_id].id = curr_id;
        _states[curr_id].jv = _vo;
        _states[curr_id].g = CostVec(_graph->CostDim(), 0);
        _states[curr_id].colSet = std::unordered_map<int, int>(); // collision set, empty set.
        _open.insert( std::make_pair(_H(_vo), curr_id) );
        // _result.n_generated++;
        // _frontiers[jv2str(_vo)] = std::unordered_set<long>();
        // _frontiers[jv2str(_vo)].insert(curr_id);
        _best[jv2str(_vo)] = curr_id;
        return true;
    };

    CostVec MPMstar::_H(const std::vector<long>& jv) {
        CostVec out(_graph->CostDim(), 0);
        for (int ri = 0; ri < _nAgent; ri++) {
            out += _policies[ri].H(jv[ri]);
        }
        if (_wH > 1.0) { // heuristic inflation
            for (size_t j = 0; j < out.size(); j++) {
                out[j] = long(out[j] * _wH) ; // NOTE the long conversion here !
            }
        }
        return out;
    };

    bool MPMstar::_IfReachGoal(const std::vector<long>& jv) {
        return jvEqual(jv, _vd);
    };

    std::unordered_map<int,int> MPMstar::_ColCheck(
            const std::vector<long>& jv1, const std::vector<long>& jv2)
    {
        std::unordered_map<int,int> out;
        for (int ix = 0; ix < _nAgent; ix++) { // loop over every pair of agents
            for (int iy = ix+1; iy < _nAgent; iy++) {
                if ( (jv2[ix] == jv2[iy]) || ((jv1[ix] == jv2[iy]) && (jv1[iy] == jv2[ix])) )
                { // vertex conflict or swap locations conflict
                    if (out.find(ix) == out.end()) { // ix is new
                        if (out.find(iy) == out.end()) { // iy is new
                            out[ix] = iy;
                            out[iy] = iy;
                        }else{ // iy is already there
                            out[ix] = out[iy];
                        }
                    } // end of if
                    else { // ix is not new
                        if (out.find(iy) == out.end()) { // iy is new
                            out[iy] = out[ix];
                        }else{ // both are not new
                            raplab::UFUnion(&out, ix, iy);
                        }
                    } // end of else
                } // end of if
            } // end for iy
        } // end for ix
        return out;
    };


    void MPMstar::_AddBackSet(const long& sid1, const long& sid2)
    {
        if (back_set_map_.find(sid2) == back_set_map_.end()) {
            back_set_map_[sid2] = std::unordered_set<long>();
        }
        back_set_map_[sid2].insert(sid1);
        return;
    };

    void MPMstar::_BackProp(const long& sid, const std::unordered_map<int,int>& col_set) {
        if ( IsColSetSubset(col_set, _states[sid].colSet) ) {
            return;
        }else{
            _ColSetUnion(col_set, &(_states[sid].colSet) );
            // if (_states[sid].colSet.size() > _result.n_maxColSet) {
            //   _result.n_maxColSet = _states[sid].colSet.size();
            // }
            _Reopen(sid);
            if (back_set_map_.find(sid) == back_set_map_.end()) {
                return;
            }
            auto& bset = back_set_map_[sid];
            for (auto iter = bset.begin(); iter != bset.end(); iter++) {
                _BackProp(*iter, col_set);
            }
        }
    };

    void MPMstar::_ColSetUnion(const std::unordered_map<int,int>& a, std::unordered_map<int,int>* b)
    {
        for (auto iter = a.begin(); iter != a.end(); iter++) {
            if (b->find(iter->first) == b->end()) { // element *iter in a but not in b.
                (*b)[iter->first] = iter->second;
            }
        }
        return ;
    }

    void MPMstar::_Reopen(const long& sid) {
        auto& s = _states[sid];
        _open.insert( std::make_pair( s.g+_H(s.jv), sid) );
        return;
    };

    CostVec MPMstar::_GetTransCost(
            const std::vector<long>& jv1, const std::vector<long>& jv2, long curr_id)
    {
        CostVec out(_graph->CostDim(), 0);
        for (size_t ix = 0; ix < _nAgent; ix++) { // loop over all agents
            if (jv1[ix] == _vd[ix] && jv2[ix] == jv1[ix]) { // stay at goal
                continue;
            }
            if (jv2[ix] == jv1[ix]) {
                out[0] += _wait_cost;
                continue;
            }
            if (jv1[ix] == _vd[ix] && jv2[ix] != _vd[ix]) { // move off goals
                out += _MoveFromGoalCost(ix, curr_id); // add accumulated wait cost at goal, because the robot moves away.
            }
            out += _graph->GetCost(jv1[ix],jv2[ix]);
        }
        return out;
    };

// bool MPMstar::_SolFilter(const CostVec& fu) {
//   for (const auto& gid : _sol) {
//     if ( search::EpsDominance(_states[gid].g, fu) ) {
//       _result.n_solFiltered++;
//       return true;
//     }
//   }
//   return false;
// };

// bool MPMstar::_DomCompare(
//   const std::vector<long>& ju, const CostVec& gu, std::vector<long>* dom_ids)
// {
//   dom_ids->clear();
//   for ( const auto& sid : _frontiers[jv2str(ju)] ) {
//     if ( search::EpsDominance(_states[sid].g, gu) ) {
//       dom_ids->push_back(sid);
//     }
//   }
//   if (dom_ids->size() > 0) {
//     return true;
//   }
//   return false;
// };


// void MPMstar::_PostProcessResult()
// {
//   for (const auto& gid : _sol) {
//     std::vector< std::vector<long> > jp;
//     _BuildJointPath(gid, &jp);
//     _result.solus[gid] = PathSet();
//     _JPath2PathSet(jp, &(_result.solus[gid]));
//     // std::cout << _result.solus[gid] << std::endl;
//     _result.costs[gid] = _states[gid].g;
//   }
//   _result.n_initRoot = -1;

//   _result.n_branch = 1.0*_result.n_branch / _result.n_expanded;
//   return ;
// };

    void MPMstar::_BuildJointPath(long sid, std::vector< std::vector<long> >* jp)
    {
        std::vector< std::vector<long> > reversed_jp;
        while (sid != -1) {
            reversed_jp.push_back(_states[sid].jv);
            sid = _parent.at(sid);
        }
        jp->clear();
        for (int idx = reversed_jp.size()-1; idx >= 0; idx--) { // convert to right order.
            jp->push_back(reversed_jp[idx]);
        }
        return ;
    };

    CostVec MPMstar::_MoveFromGoalCost(const int& ri, long sid) {
        CostVec out(_graph->CostDim(), 0);
        long v = _states[sid].jv[ri];
        while (true) {
            long pid = _parent[sid];
            long u = _states[pid].jv[ri];
            if ((u == v) && (u == _vd[ri])) {
                out[0] += _wait_cost; //_agentCosts[ri];
            }else{
                break;
            }
            sid = pid;
        }
        return out;
    }

} // end namespace rzq
