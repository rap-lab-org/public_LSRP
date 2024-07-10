/*******************************************
 * Author: Zhongqiang Richard Ren, Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapf_mpmstar.hpp"
#include "mapf_mstar.hpp"
#include "mapf_pibt.hpp"
#include "union_find.hpp"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <set>

namespace raplab{

    struct VectorHash {
        std::size_t operator()(const std::vector<long>& v) const {
            std::size_t seed = v.size();
            for (const auto& i : v) {
                seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };


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
        look_ahead_factor = int(70 - 3000/(50+_vo.size()));
        _Init();
        if ( DEBUG_MPMstar ) { std::cout << "[DEBUG] after init..." << std::endl; }
        if ( Statics ) {runtime = 0, states_generate = 0, states_expand = 0, max_colsets = 0, max_ngh_size = 0,all_action_counts = 0,count_of_pibt=0,
                        fail_of_pibt=0,one_step_pibt = 0, loweest_bound = _H(starts)[0];}
        int counter = 0;

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

            if ( DEBUG_MPMstar ) { std::cout << "[DEBUG] Current state..." << s.id <<std::endl; }
            if (Statics) { states_expand += 1;}

            // check for solution
            if (_IfReachGoal(s.jv)) {
                _stats["success"] = 1;
                _reached_goal_id = s.id;
                if (Statics) {runtime = std::chrono::duration<double>(std::chrono::steady_clock::now() - _t0).count();}
                if ( DEBUG_MPMstar ) {std::cout << "[INFO] MPMstar finds a solution ! " << std::endl;}
                break;
            }
            // std::cout << " will expand " << std::endl;

            // expand
            std::vector< std::vector<long> > nghs;
            bool success = _GetNgh(s.id, &nghs);
            // std::cout << " after get limited ngh" << std::endl;
            if (!success) {
                break;
            }
            if (Statics) {
                if (nghs.size() > max_ngh_size){
                    max_ngh_size = nghs.size();
                }
            }
            // _result.n_expanded++;

            _stats["num_exp"] += 1;
            for (auto& ngh : nghs) { // loop over all limited neighbors.

                auto colSet = _ColCheck(s.jv, ngh);
                if (colSet.size() > 0) { // there is collision.
                    _LookAhead(ngh,&colSet);
                    _BackProp(s.id, colSet);
                    continue;
                }
                if (DEBUG_MPMstar){std::cout<<"Col set size: "<< std::endl;}

                // collision-free
                // get cost vector
                auto cuv = _GetTransCost(s.jv, ngh, s.id);
                CostVec g_ngh = s.g + cuv;
                CostVec f_ngh = g_ngh + _wH * _H(ngh); // note that _wH is considered within _H()
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
        if ( DEBUG_MPMstar ) { std::cout << "[DEBUG] Solution not found but run out open?..." << std::endl; }
        if (Statics) {states_generate = _id_gen;}

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
        if(Statics){cost_times = _states[_reached_goal_id].g[0]/loweest_bound;}
    };

    std::unordered_map<std::string, double> MPMstar::GetStats() {
        return _stats;
    } ;

    bool MPMstar::_PibtorNot(long sid) {
        const auto& colSet = _states[sid].colSet;
        const auto& jv = _states[sid].jv;
        if (_Pibt_policy.find(jv) != _Pibt_policy.end()) {
            int max_size = 0;
            for (const auto &policy: _Pibt_policy.at(jv)) {
                if (policy.first.size() > max_size) {
                    max_size = policy.first.size();
                }
            }
            return colSet.size() > max_size;
        } else {
            return true;
        }
    }

    bool MPMstar::_Get_Ngh_neo(const long &sid, std::vector<std::vector<long>> *out) {
         out->clear();
         auto& colSet = _states[sid].colSet;
         auto& jv = _states[sid].jv;
         if(!colSet.empty()){
            std::vector<int> Pibt_agent = _Get_pibt_agent(sid,colSet);
            _One_step_Pibt(Pibt_agent,sid,out);
            return true;
         } else {
            if (_planAgent.find(jv) == _planAgent.end()){
                return  _GetLimitNgh(sid,out);
            } else {
                _One_step_Pibt(_planAgent.at(jv),sid,out);
                std::vector<std::vector<long>> ngh3;
                _GetLimitNgh(sid,&ngh3);
                out->insert(out->end(), ngh3.begin(), ngh3.end());
                _RemoveDuplicates(out);
                return true;
            }
        }
    }

    bool MPMstar::_GetNgh(const long &sid, std::vector<std::vector<long>> *out) {
        out->clear();
        const auto& colSet = _states[sid].colSet;
        const auto& jv = _states[sid].jv;
        if(!colSet.empty()){
            if (Statics) {
                if (colSet.size() > max_colsets) {
                    max_colsets = colSet.size();
                }
            }
            if (_PibtorNot(sid)) {
                std::vector<std::vector<long>> pibt_policy;
                //if (colSet.size() > 0.5 * _vd.size()) {
                     pibt_policy = Pibt_process(colSet, sid);
                     if (Statics){count_of_pibt += 1;}
                //} else {
                //   pibt_policy = Pibt_one(colSet, sid);
                //    if (Statics){one_step_pibt += 1;}
                //}
                if(pibt_policy.empty()) {
                        if (Statics) { fail_of_pibt += 1; }
                        // check if it has pibt policy  though we fail we could till do max colset plan
                        if (_Pibt_policy.find(jv) != _Pibt_policy.end()) {
                        // has pibt policy
                            _getMaxPibtngh(_Pibt_policy.at(jv), sid, out);
                            _RemoveDuplicates(out);
                            return true;
                         } else {
                        // no pibt policy get full action,  worst choice
                         if (Statics) { all_action_counts += 1; }
                            return _GetLimitNgh(sid, out);
                        }
                } else {
                    // pibt success and directly use it cause it has biggest set
                    //_UnitePolicy(colSet, pibt_policy, jv);
                    std::vector<int> col;
                    for (const auto& pair : colSet) {
                        int agentId = pair.first;
                        col.push_back(agentId);
                    }
                    _GetPibtNgh(col, pibt_policy,sid,out);
                    //_getMaxPibtngh(_Pibt_policy.at(jv), sid, out);
                    _RemoveDuplicates(out);
                    return true;
                }
            } else {
            _getMaxPibtngh(_Pibt_policy.at(jv), sid, out);
            _RemoveDuplicates(out);
            return true;
        }
        } else {
            // No needs to plan pibt, get best policy
            if (_Pibt_policy.find(jv) != _Pibt_policy.end()) {
                std::unordered_map<std::vector<int>, std::vector<std::vector<long>>> policies = _Pibt_policy.at(jv);
                _getMaxPibtngh(policies,sid,out);
                std::vector<std::vector<long>> ngh3;
                _GetLimitNgh(sid,&ngh3);
                out->insert(out->end(), ngh3.begin(), ngh3.end());
                _RemoveDuplicates(out);
                ngh3.clear();
                return true;
            } else {
                // pibt policy not exists, optimal policy
                // Thank you so much you are the kindest scenario
                return _GetLimitNgh(sid, out);
            }
        }
    }

    std::vector<std::vector<long>> MPMstar::Pibt_one(const std::unordered_map<int, int> colSet, const long &sid) {
        std::vector<std::vector<long>> pibt_policy;
        std::vector<int> col;
        for (const auto& pair : colSet) {
            int agentId = pair.first;
            col.push_back(agentId);
        }
        _One_step_Pibt(col,sid,&pibt_policy);
        return pibt_policy;
    }

    std::vector<std::vector<long>> MPMstar::Pibt_process(const std::unordered_map<int, int> colSet, const long &sid) {
        std::vector<std::vector<long>> pibt_policy;
        std::vector<long> start;
        std::vector<long> goal;
        const auto& Sfrom = _states[sid].jv;
        // generate folloing policy
        std::unordered_map<int, MstarPolicy> colPolicies;
        int newId = 0;
        for (const auto& pair : colSet) {
            int agentId = pair.first;
            start.push_back(Sfrom[agentId]);
            goal.push_back(_vd[agentId]);
            colPolicies[newId] = _policies.at(agentId);
            ++newId;
        }
        bool result = planner._Solve(start,goal,_tlimit,1.0,&colPolicies);
        if (result){
            pibt_policy = planner.GetPlan();
            pibt_policy.erase(pibt_policy.begin());
            return pibt_policy;
        } else {
            // not success return a blank vector;
            return pibt_policy;
        }
        return pibt_policy;
    }

    bool MPMstar::_GetPibtNgh(std::vector<int> Pibt_agent, std::vector<std::vector<long>> pibt_policy,
                              const long &sid, std::vector<std::vector<long>> *out) {
        std::vector<long> pibt_sto = pibt_policy[0];
        const auto& Sfrom = _states[sid].jv;
        std::vector< std::vector<long> > ngh_vec;
        ngh_vec.resize(_nAgent);
        // insert pibt
        for (size_t index = 0; index < Pibt_agent.size(); ++index) {
            ngh_vec[Pibt_agent[index]].push_back(pibt_sto[index]);
        }
        std::unordered_set<int> pibt_agent_set(Pibt_agent.begin(), Pibt_agent.end());
        for (size_t ri = 0; ri < _nAgent; ++ri) {
            if (pibt_agent_set.find(ri) == pibt_agent_set.end()) {
                ngh_vec[ri].push_back(_policies[ri].Phi(Sfrom[ri]));
            }
        }
        TakeCombination(ngh_vec, out);
        // for its ngh jv state  inherits the policy
        std::vector<long> jv = out->at(0);
        std::vector<std::vector<long>> kid_policy;
        std::copy(pibt_policy.begin() + 1, pibt_policy.end(), std::back_inserter(kid_policy));
        if (!kid_policy.empty()) {
            if (_Pibt_policy.find(jv) != _Pibt_policy.end()) {
                _Pibt_policy[jv][Pibt_agent] = kid_policy;
            } else {
                std::unordered_map<std::vector<int>, std::vector<std::vector<long>>> colAndPolicy;
                colAndPolicy[Pibt_agent] = kid_policy;
                _Pibt_policy[jv] = colAndPolicy;
            }
        }
        return true;
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

    void MPMstar::_UnitePolicy(std::unordered_map<int, int> colSet, std::vector<std::vector<long>> pibt_policy,
                               std::vector<long> jv) {
        std::vector<int> col;
        for (const auto& pair : colSet) {
            int agentId = pair.first;
            col.push_back(agentId);
        }
        if (_Pibt_policy.find(jv) != _Pibt_policy.end()) {
            _Pibt_policy[jv][col] = pibt_policy;
        } else {
            std::unordered_map<std::vector<int>, std::vector<std::vector<long>>> colAndPolicy;
            colAndPolicy[col] = pibt_policy;
            _Pibt_policy[jv] = colAndPolicy;
        }



    }
    bool MPMstar::_RemoveDuplicates(std::vector<std::vector<long>>* out) {
        std::unordered_set<std::vector<long>, VectorHash> unique_elements;
        std::vector<std::vector<long>> unique_out;

        for (const auto& element : *out) {
            if (unique_elements.insert(element).second) {
                unique_out.push_back(element);
            }
        }

        *out = std::move(unique_out);
        return true;
    }

    bool MPMstar::_getMaxPibtngh(std::unordered_map<std::vector<int>, std::vector<std::vector<long>>> policies,
                                 const long &sid, std::vector<std::vector<long>> *out) {
        std::vector<std::vector<int>> Maxcols;
        size_t max_size = 0;
        for (const auto& policy : policies) {
            if (policy.first.size() > max_size) {
                max_size = policy.first.size();
            }
        }
        for (const auto& policy : policies) {
            if (policy.first.size() == max_size) {
                Maxcols.push_back(policy.first);
                break;
            }
        }
        //debugger
        for (const auto& col : Maxcols){
            std::vector<std::vector<long>> out_i;
            _GetPibtNgh(col, policies.at(col),sid,&out_i);
            out->insert(out->end(), out_i.begin(), out_i.end());
        }
        return true;
    }

    void MPMstar::_LookAhead(std::vector<long> jv , std::unordered_map<int, int> *col_set) {
        for (int i = 0; i < look_ahead_factor; i++){
            if (jv == _vd){
                return;
            }
            std::vector<long> jv_next;
            for (int ri = 0; ri < _nAgent; ri++){ // loop over robots
                jv_next.push_back(_policies[ri].Phi(jv[ri]));
            }
            auto col = _ColCheck(jv, jv_next);
            _ColSetUnion(col, col_set);
            jv = jv_next;
            jv_next.clear();
        }
    }

    std::vector<int> MPMstar::_Get_pibt_agent(const long &sid, std::unordered_map<int, int> col_set) {
        auto& jv = _states[sid].jv;
        std::vector<int> col;
        for (const auto& pair : col_set) {
            int agentId = pair.first;
            col.push_back(agentId);
        }
        if (_planAgent.find(jv) == _planAgent.end()){
            _planAgent[jv] = col;
            return col;
        } else {
            if (_planAgent[jv].size() >= col_set.size()){
                return _planAgent[jv];
            } else {
                _planAgent[jv] = col;
                return col;
            }
        }
    }

    void MPMstar::_One_step_Pibt(std::vector<int> agent, const long &sid, std::vector<std::vector<long>> *out) {
        std::vector<int> copy_agent;
        std::copy(agent.begin() , agent.end(), std::back_inserter(copy_agent));

        std::vector<long> start;
        const auto& Sfrom = _states[sid].jv;
        for (const auto& a : agent) {
            start.push_back(Sfrom[a]);
        }
        std::sort(copy_agent.begin(), copy_agent.end(), [&](int a, int b) {
            return _policies[a].H(Sfrom[a]) < _policies[b].H(Sfrom[b]);
        });
        std::vector<Agent> pibt_agents;
        double gap = 1.0 / (copy_agent.size() + 1);
        for (size_t i = 0; i < copy_agent.size(); ++i) {
            pibt_agents.emplace_back(i, 1 - i * gap);
        }
        for (auto &agent: pibt_agents){
            agent.set_trueid(copy_agent[agent.id]);
        }
        std::vector<long> Sto(Sfrom.size(), -1);
        for (auto &agent: pibt_agents) {
            if (Sto[agent.id] == -1) {
                Pibt(&agent, nullptr, Sfrom, Sto,pibt_agents);
            }
        }

        std::vector< std::vector<long> > ngh_vec;
        ngh_vec.resize(_nAgent);
        // insert pibt
        for (auto &agent: pibt_agents){
            ngh_vec[agent.true_id].push_back(Sto[agent.id]);
        }
        std::unordered_set<int> pibt_agent_set(copy_agent.begin(), copy_agent.end());
        for (size_t ri = 0; ri < _nAgent; ++ri) {
            if (pibt_agent_set.find(ri) == pibt_agent_set.end()) {
                ngh_vec[ri].push_back(_policies[ri].Phi(Sfrom[ri]));
            }
        }
        TakeCombination(ngh_vec, out);
        //inherit
        /*
        std::vector<long> jv = out->at(0);
        if (_planAgent.find(jv) == _planAgent.end()){
            _planAgent[jv] = copy_agent;
        } else {
            if (_planAgent[jv].size() < copy_agent.size()){
                _planAgent[jv] = copy_agent;
            }
        }
        */
    }



    bool MPMstar::Pibt(Agent *agent1, Agent *agent2, const std::vector<long> &Sfrom, std::vector<long> &Sto,
                       std::vector<Agent> agents_) {
        std::vector<long> C = _graph->GetSuccs(Sfrom[agent1->id]);
        C.push_back(Sfrom[agent1->id]);
        //sort node by dis_table value
        auto &policy = _policies.at(agent1->true_id);
        std::sort(C.begin(), C.end(), [&](long a, long b) {
            return policy.H(a) < policy.H(b);
        });


        for (long v: C){
            if(checkOccupied(v, Sto,agents_)){
                continue;
            }
            if(agent2 != nullptr && Sfrom[agent2->id] == v){
                continue;
            }
            Sto[agent1->id] = v;
            Agent* a2 = mayPush(v, Sfrom, Sto,agents_);
            if (a2 != nullptr){
                if(!Pibt(a2, agent1, Sfrom, Sto,agents_)){
                    continue;
                }
            }
            return true;
        }
        Sto[agent1->id] = Sfrom[agent1->id];
        return false;
    }

    bool MPMstar::checkOccupied(long v, const std::vector<long> &Sto,std::vector<Agent> agents_) {
        return std::any_of(agents_.begin(), agents_.end(), [&](const Agent& a) {
            return Sto[a.id] == v;
        });
    }

// Check if any low priority agent take the place and needs to plan
    Agent *MPMstar::mayPush(long v, const std::vector<long> &Sfrom, const std::vector<long> &Sto, std::vector<Agent> agents_) {
        auto it = std::find_if(agents_.begin(), agents_.end(), [&](const Agent& a) {
            return Sfrom[a.id] == v && Sto[a.id] == -1;
        });

        if (it != agents_.end()) {
            return &(*it);
        } else {
            return nullptr;
        }
    }


} // end namespace rzq
