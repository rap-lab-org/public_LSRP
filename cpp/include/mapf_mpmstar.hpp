
/*******************************************
 * Author: Zhongqiang Richard Ren, Shuai Zhou.
 * All Rights Reserved.
 *******************************************/


#ifndef SHUAIZHOU_BASIC_MAPF_MPMSTAR_H_
#define SHUAIZHOU_BASIC_MAPF_MPMSTAR_H_

#include "mapf_util.hpp"
#include "mapf_mstar.hpp"
#include "mapf_pibt.hpp"
#include "union_find.hpp"
#include "search_astar.hpp"
#include <set>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <memory>
#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <unordered_map>


namespace raplab{

#define DEBUG_MPMstar 0
#define Statics 1
std::ostream& operator<<(std::ostream& os, MState& state);
void CompileHelper();

    struct tree_node
    {
        long _id;
        long _action;
        int _depth;
        std::shared_ptr<tree_node> _parent;

        tree_node(long id, long action, int depth = -1, std::shared_ptr<tree_node> parent = nullptr)
                : _id(id), _action(action), _depth(depth), _parent(parent) {}

    };



/**
 * @brief
 */
    class MPMstar : public MAPFPlanner
    {
    public:
        /**
         *
         */
        MPMstar() ;
        /**
         *
         */
        virtual ~MPMstar() ;
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

        double Get_runtime () {return runtime;}

        int Get_totalStates() {return states_generate;}

        int Get_expandStates() {return states_expand;}

        int Get_maxColsets() {return max_colsets;}

        int Get_maxNghSize() {return max_ngh_size;}

        int Get_FULLactionCount() {return all_action_counts;}

        // optimize part
        double runtime;
        int states_generate;
        int states_expand;
        int max_colsets;
        int max_ngh_size;
        int all_action_counts;
        int action_tree;
        int fail_of_pibt;
        int one_step_pibt;
        int lowest_bound;
        double cost_times;
        CostVec fmin;



    protected:
        /**
         * @brief
         */
        virtual bool _GetLimitNgh(const long& sid, std::vector< std::vector<long> >* out);
        /**
         * @brief
         */
        virtual bool _ActionTree(const long& sid,std::vector< std::vector<long> >* out);
        /**
         * @brief
         */
         virtual bool _Get_OneStepPibt(std::set<int> col, const long& sid, std::vector< std::vector<long> >* out);
        /**
         * @brief
         */
        virtual bool _GetMPMngh(const long& sid, std::vector< std::vector<long> >* out);
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

        virtual void _LookAhead(std::vector<long> jv, std::unordered_map<int,int>* col_set);


        virtual void _One_step_Pibt(const long &sid, std::vector<long> *Sto);

        virtual void _Update_Focal();

        virtual void _Update_closed(const long &sid, const std::vector<std::vector<long>>* out);

        virtual bool _Check_closedset(const long &sid, std::vector<std::vector<long>>* out);

        virtual void _Debug_print(long sid);

        bool Pibt(Agent *agent1, Agent *agent2, const std::vector<long> &Sfrom, std::vector<long> &Sto,std::vector<Agent> agents_);

        bool checkOccupied(long v, const std::vector<long> &Sto,std::vector<Agent> agents_);

        Agent *mayPush(long v, const std::vector<long> &Sfrom, const std::vector<long> &Sto, std::vector<Agent> agents_);

        void _Get_action(const long& sid, std::vector<long>* Sto);

        // not using anymore
        /*
        virtual std::vector<int> _Get_pibt_agent(const long& sid, std::unordered_map<int, int> col_set);

        virtual bool _PibtorNot(long sid);

        virtual bool _Pibt_required(long sid);

        virtual bool _getMaxPibtngh(std::unordered_map<std::vector<int>, std::vector<std::vector<long>>> policies,
                                    const long& sid, std::vector< std::vector<long> >* out);

        virtual bool _GetPibtNgh(std::vector<int> Pibt_agent, std::vector<std::vector<long>> pibt_policy,
                                 const long& sid, std::vector< std::vector<long> >* out);

        virtual bool _RemoveDuplicates(std::vector<std::vector<long>>* out);

        virtual bool _GetNgh(const long& sid, std::vector< std::vector<long> >* out);

        virtual std::vector<std::vector<long>> Pibt_process(const std::unordered_map<int, int> colSet, const long& sid);
         */


        raplab::Pibt planner;
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
        std::set< std::pair< CostVec , long> > _focal; // <h_value cost vector, state id>
        // std::unordered_map< std::string, std::unordered_set<long> > _frontiers;
        std::unordered_map< std::string, long > _best;
        std::vector<long> _sol;
        // std::unordered_map<int, CostVec> _agentCosts;
        std::unordered_map<std::string, double> _stats;
        // MOMAPFResult _result;
        double _wait_cost = 1;
        long _reached_goal_id = -1;
        //double _runtime;
        double _wH = 1.0;
        std::vector< std::vector<long> > _sol_path;
        std::unordered_map<std::vector<long> , std::unordered_map<std::vector<int>, std::vector<std::vector<long>>>> _Pibt_policy;
        int look_ahead_factor;

        std::unordered_map<std::vector<long>, std::vector<int>> _planAgent;
        std::unordered_map<std::vector<long>, bool> _Fullyexpanded_table;
        std::unordered_map<std::vector<long>, std::vector<tree_node>> _All_tree;
        std::unordered_map<std::vector<long>, std::set<std::vector<long>>> _All_closed_Set;
        std::unordered_map<std::vector<long>, std::vector<Agent>> _All_pibtAgent_order;
        std::unordered_map<std::vector<long>, std::vector<int>> _All_node_Agentorder;
        std::unordered_map<std::vector<long>,std::set<int>> Ic_cache;


        void savePathsToCoordinates();
    };


} // end namespace rzq


#endif  // ZHONGQIANGREN_BASIC_MAPF_MSTAR_H_
