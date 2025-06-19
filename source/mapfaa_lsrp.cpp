/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "mapfaa_lsrp.hpp"
#include <functional>
#include <fstream>

namespace raplab{

    /**
     * State of certain agent in a period of time
     * @param parent_v parent vertex
     * @param v  arriving vertex
     * @param parent_time time leaving parent
     * @param time arriving time
     */
    // lsrp: timestamp state
    State::State() {};

    State::State(long parent_v, long v, double parent_time, double time)
            : p(parent_v), v(v), startT(parent_time), endT(time) {}

    bool State::operator==(const State& other) const {
        return (p == other.p && v == other.v && startT == other.startT && endT == other.endT);
    }

    std::size_t State::hash() const {
        std::size_t hash_value = 0;
        std::hash<int> int_hash;
        std::hash<double> double_hash;
        hash_value ^= int_hash(p) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        hash_value ^= int_hash(v) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        hash_value ^= double_hash(startT) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        hash_value ^= double_hash(endT) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        return hash_value;
    }

    std::tuple<long, long, double, double> State::get_tuple() const {
        return std::make_tuple(p, v, startT, endT);
    }

    double State::get_endT() const{
        return endT;
    }
    double State::get_startT() const{
        return startT;
    }

    long State::get_v() const{
        return v;
    }

    long State::get_p() const {
        return p;
    }

    // lsrp- agent
    /**
     * Agent  each agents information
     * @param id
     * @param start start vertex
     * @param goal  goal vertex
     */
    Agent::Agent() {};
    Agent::Agent(int id, long start, long goal)
            : id(id),priority(0.0), curr(new State(start, start, 0.0, 0.0)), at_goal(false), init_pri(0.0),goal(goal) {
    }

    void Agent::set_init_priority(double priority) {
        this->init_pri = priority;
        this->priority = priority;
    }

    void Agent::set_priority(double pri) {
        this->priority = pri;
    }

    bool Agent::operator==(const Agent& other) const {
        return this->id == other.id;
    }

    int Agent::get_id() const{
        return id;
    }

    State* Agent::get_curr() const {
        return curr;
    }

    bool Agent::is_at_goal() const{
        return at_goal;
    }

    double Agent::get_init_priority() const{
        return init_pri;
    }

    double Agent::get_priority() const{
        return priority;
    }


    long Agent::get_goal() const {
        return goal;
    }

    void Agent::set_curr(State* &curr) {
        this->curr = curr;
    }

    void Agent::set_at_goal(bool at_goal) {
        this->at_goal = at_goal;
    }



    //Lsrp part main function
    Lsrp::Lsrp() {};

    Lsrp::~Lsrp() {
        for (Agent* agent : _agents) {
            delete agent;
        }
        _agents.clear();
    };

    CostVec Lsrp::GetPlanCost(long nid) {
        CostVec out(_graph->CostDim(), 0);
        out[0] = re_soc();
        out[1] = re_makespan();
        return out;
    }

    int Lsrp::Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps)
    {
        if (starts.empty()) {return 1;}
        _Sinit = starts;
        _Send = goals;
        _dis_table = generate_distable();
        set_agents();
        _S_T = set_initPolicy();
         Set_minduration();
        _time_limit = time_limit;
        _lsrp();
        return 1;
    }


    void Lsrp::Set_minduration()
    {
        _min_duration = *std::min_element(_duration.begin(), _duration.end());
        if (!edge_cost.empty()) {
            for (const auto& outer_pair : edge_cost) {
                for (const auto& inner_pair : outer_pair.second) {
                    if (inner_pair.second < _min_duration) {
                        _min_duration = inner_pair.second;
                    }
                }
            }
        }
    }

// A list stored all the distance_table for each agent.
// Heuristic function related
    std::vector<std::unordered_map<long, double>> Lsrp::generate_distable() {
        std::vector<std::unordered_map<long, double>> distable;
        for (size_t i = 0; i < _Sinit.size(); ++i) {
            distable.push_back(generate_single_dis_table(static_cast<int>(i)));
        }
        return distable;
    }

//A method generate bfs value for each agent, each coordination corresponds to a specific value and were used as
//heuristic value for each state
//Because bfs promises the optimal path for a single agent and it is relatively fast
    std::unordered_map<long, double> Lsrp::generate_single_dis_table(int agent) {
        std::deque<long> tmp;
        tmp.push_back(_Send[agent]);
        const double inf = std::numeric_limits<double>::max();
        std::unordered_map<long, double> dist_table;

        dist_table[_Send[agent]] = 0;

        while (!tmp.empty()) {
            long curr = tmp.front();
            tmp.pop_front();
            std::vector<long> successors = _graph->GetSuccs(curr);
            for (long neigh : successors) {
                if (!edge_cost.empty() && edge_cost.find(agent) != edge_cost.end() && edge_cost[agent].find(edge_hash(curr, neigh)) != edge_cost[agent].end()) {
                    // this edge cost for this agent is specified
                    if (dist_table.find(neigh) == dist_table.end() || dist_table[neigh] > dist_table[curr] + edge_cost[agent].at(edge_hash(curr, neigh))) {
                        dist_table[neigh] = dist_table[curr] + edge_cost[agent].at(edge_hash(curr, neigh));
                        tmp.push_back(neigh);
                    }
                } else {
                    // this edge cost for this agent is not specified so using the default duration cost
                    if (dist_table.find(neigh) == dist_table.end() || dist_table[neigh] > dist_table[curr] + _duration[agent]) {
                        dist_table[neigh] = dist_table[curr] + _duration[agent];
                        tmp.push_back(neigh);
                    }
                }
            }
        }
        return dist_table;
    }

    // A method using cantor hash the edges
    int Lsrp::edge_hash(long a, long b) const{
        if (a > b)
        {
            return(a + b) * (a + b + 1) / 2 + b;
        }
        return(a + b) * (b + a + 1) / 2 + a;
    }

//A function calculates heuristic value of a given state
//        take in a serial number of a agent eg: 1
//        and a coordination id eg: 34
//        return h value
    double Lsrp::get_h(const Agent& agent, const long& coord) {
        return _dis_table[agent.get_id()].at(coord);
    }

//Set up initial priority based on decreasing order of their duration
//        set a list of agents class, contains a numbers of agents
//        And set up initial priority by given a unique value in the interval between 0 - 1
    void Lsrp::set_agents() {
        size_t n = _Send.size();
        double gap = 1.0 / (n + 1);
        for (int i = 0; i < _Sinit.size(); ++i) {
            _agents.push_back(new Agent(static_cast<int>(i), _Sinit[i], _Send[i]));
        }
        for (int i = 0; i<_agents.size(); i++) {
            _agents[i]->set_init_priority(i * gap);
        }
    }

// A method set initial states
// eg: [(s11, s21, s31, s41, ·······)]
// also set occupation and referred time policy
    std::vector<std::vector<State*>> Lsrp::set_initPolicy() {
        std::vector<std::vector<State*>> policy;
        std::vector<State*> States_init;
        States_init.reserve(_Sinit.size());
        for (int i = 0; i< _agents.size(); i++) {
            States_init.push_back(_agents[i]->curr);
        }
        policy.push_back(States_init);
        return policy;
    }

// Check if all agents reach goal
    bool Lsrp::reach_Goal() const {
        for (const auto& agent : _agents) {
            if (!agent->is_at_goal()) {
                return false;
            }
        }
        return true;
    }

// get tmin2
// A method get next t
// Using for the wait time of certain agent
// Inspired by ls-rM*
// if there are no time, return None
    double Lsrp::get_tmin2() const {
        if (_T.empty()) {
            return -1; // Return NaN if there is no next time
        }
        return _T.top();
    }

// A function which extracts agents from all agents
// The filter is based on the given t and extracts agent whose curr state with arriving time at t
    std::vector<Agent*> Lsrp::extract_Agents(double t) {
        std::vector<Agent*> return_agents;
        for (int i = 0; i <  _agents.size(); i++) {
            if (_agents[i]->get_curr()->get_endT() == t) {
                return_agents.push_back(_agents[i]);
            }
        }
        return return_agents;
    }


//A method that generate state tuple
//that agent no needed to plan in this timestamp are added in
//while those reach endT are set to be None and implemented by Lsrp later
    std::vector<State*> Lsrp::get_rawSnext(std::vector<State*> S_from,
                                                           const std::vector<Agent*>& curr_agents, double t) const {
        std::vector<State*> re_S;
        const std::vector<State*>* t_policy_ptr = nullptr;

        auto it = _cache.find(t);
        if (it != _cache.end()) {
            t_policy_ptr = &(it->second);
        }

        // 遍历 S_from
        for (size_t index = 0; index < S_from.size(); ++index) {
            Agent* agent_ptr = _agents[index];
            if (std::find(curr_agents.begin(), curr_agents.end(), agent_ptr) != curr_agents.end()) {
                // 如果 t_policy_ptr 不是 nullptr，则将 t_policy_ptr[index] 添加到 re_S 中
                if (t_policy_ptr != nullptr) {
                    re_S.push_back((*t_policy_ptr)[index]);
                } else {
                    re_S.push_back(nullptr);
                }
            } else {
                re_S.push_back(S_from[index]);
            }
        }

        return re_S;
    }




//Update the priority of each agent, even though some of their agent still at their last moving state
//There are three versions of it. Referred to the details from following content.
    void Lsrp::update_Priority() {
        for (int i = 0; i <  _agents.size(); i++) {
            if (_agents[i]->is_at_goal()) {
                _agents[i]->set_priority(_agents[i]->get_init_priority());
            } else {
                _agents[i]->set_priority(_agents[i]->get_priority() + 1);
            }
        }
    }

// return the given duration
    double Lsrp::get_duration(const Agent& agent,long v1, long v2) const {
        if (v1 == v2) {
            return 0; // If the vertex is the same, return 0. wait action should be determined by asypush
        }
        auto edge = edge_hash(v1,v2);
        if (!edge_cost.empty() && edge_cost.find(agent.get_id()) != edge_cost.end() && edge_cost.at(agent.get_id()).find(edge) != edge_cost.at(agent.get_id()).end())
        {
            return edge_cost.at(agent.get_id()).at(edge);
        }
        return _duration[agent.get_id()];
    }

// A Method generate state
    State Lsrp::generate_state(const long& v, const Agent& agent,
                                   const std::vector<State*>& Sfrom, double* tmin2) const {
        auto parent_ptr = Sfrom[agent.get_id()];
        if (parent_ptr == nullptr) {
            throw std::runtime_error("Invalid state pointer for agent.");
        }

        const State& parent = *parent_ptr;

        // The wait situation
        if (v == parent.get_v()) {
            double endT = (tmin2 != nullptr && *tmin2 != -1) ? *tmin2 : parent.get_endT() + _min_duration;
            return State(parent.get_v(), v, parent.get_endT(), endT);
        }

        // move situation
        double endT = parent.get_endT() + get_duration(agent, Sfrom.at(agent.get_id())->get_v(),v);
        return State(parent.get_v(), v, parent.get_endT(), endT);
    }

//Used in get_successor
//Check if a coordination(Vertex) is occupied by some states in the Sto and Sfrom.v
//the collision model wo used here is the same as lsrm*
//No worry edge collision， p is occupied and no swap would happened
//Also avoid that vertex is being pibted
    bool Lsrp::check_Occupied(const Agent& agent, const long& v,
                                  const std::vector<State*>& Sto,
                                  const std::vector<long>& constrain_list, bool in_push_possible) const {
        for (size_t index = 0; index < Sto.size(); ++index) {
            if (index == agent.get_id()) {
                continue;
            }

            auto state_opt = Sto[index];
            if (state_opt == nullptr) {
                continue;
            }

            State* state = state_opt;
            if (v == state->get_v() || v == state->get_p()) {
                // if the v is occupied, then bye bye
                return true;
            }
        }

        if (in_push_possible) {
            if (v == agent.get_curr()->get_v()) {
                return true;
            }
            // you should not go to the place where it has not decided yet but should be occupied by themselves
            if (std::find(constrain_list.begin(), constrain_list.end(), v) != constrain_list.end()) {
                return true;
            }
        }

        return false;
    }

//A method check if the low priority deadlock situation is going to happen
//If it is, Return True and starts the lsrp process
    bool Lsrp::check_potential_deadlock(const long& v, const Agent& ag,
                                            const std::vector<State*>& Sfrom,
                                            const std::vector<State*>& Sto) const {
        auto parent = Sfrom[ag.get_id()];
        if (parent->get_v() == v && Sto[ag.get_id()] == nullptr) {
            return true;
        }
        return false;
    }

//generate the ag that needed to be inheritance priority
    Agent* Lsrp::push_required(const std::vector<Agent*>& curr_agents, const Agent& agent,
                                             const long& v,
                                             const std::vector<State*>& Sfrom,
                                             const std::vector<State*>& Sto) const {
        for (const auto& ag_ptr : curr_agents) {
            if (*ag_ptr == agent) {
                continue;
            }
            if (check_potential_deadlock(v, *ag_ptr, Sfrom, Sto)) {
                return ag_ptr;
            }
        }
        return nullptr;
    }


//Helper function
//insert state to specific time's policy
    void Lsrp::insert_policy(const std::vector<std::tuple<Agent, State*>>& agent_state_list,
                                 std::unordered_map<double, std::vector<State*>>& new_policy) const {
        for (const auto& agent_state : agent_state_list) {
            const Agent& agent = std::get<0>(agent_state);
            auto state = std::get<1>(agent_state);
            double t = state->get_startT();
            if (new_policy.find(t) == new_policy.end()) {
                new_policy[t] = std::vector<State*>(_Sinit.size(), nullptr);
            }
            new_policy[t][agent.get_id()] = state;
        }
    }

// Merge successful policy with self.t_policy
// Also insert time to the timestamp list
    void Lsrp::merge_policy(const std::unordered_map<double, std::vector<State*>>& new_policy, double curr_t) {
        for (const auto& pair : new_policy) {
            const auto& t = pair.first;
            const auto& S_t = pair.second;
            auto it = _cache.find(t);
            if (it == _cache.end()) {
                _cache[t] = S_t;
                if (_T_set.find(t) == _T_set.end() && t != curr_t) {
                    _T.push(t);
                    _T_set.insert(t);
                }
            } else {
                std::vector<State*>& policy = it->second;
                for (size_t index = 0; index < S_t.size(); ++index) {
                    if (S_t[index] != nullptr) {
                        if (index >= policy.size()) {
                            policy.resize(index + 1);
                        }
                        policy[index] = S_t[index];
                    }
                }
            }
        }
    }

// Update all the information
    void Lsrp::update(const std::vector<Agent*>& curr_agents, std::vector<State*> Sto) {
        _S_T.push_back(Sto);

        for (Agent* agent : curr_agents) {
            int id = agent->get_id();
            agent->set_curr(Sto[id]);

            // Input time
            double endT = Sto[id]->get_endT();
            if (_T_set.find(endT) == _T_set.end()) {
                _T.push(endT);
                _T_set.insert(endT);
            }

            // Update each at_goal
            if (Sto[id]->get_v() == agent->get_goal()) {
                agent->set_at_goal(true);
            } else {
                // Some agents might leave their goal point to make space for others
                agent->set_at_goal(false);
            }
        }
    }


// Calculate the soc cost of the algorithm
    double Lsrp::get_Soc() {
        //double g = 0.0;
        std::vector<double> sum_g(_agents.size(), 0.0);

        // Iterate through policy and calculate social cost
        for (size_t index = 1; index < _S_T.size(); ++index) {
            const std::vector<State*>& Q = _S_T[index];
            const std::vector<State*>& prev_Q = _S_T[index - 1];

            // Assuming Q and prev_Q have the same size
            for (size_t i = 0; i < Q.size(); ++i) {
                const State& state = *Q[i];
                const State& prev_state = *prev_Q[i];

                // Compare current state with previous state
                if (state.get_startT() == prev_state.get_startT()) {
                    continue;
                }

                if (state.get_v() == prev_state.get_v() && state.get_v() == _Send[i]) {
                    // Waiting
                    continue;
                } else {
                    // Moving
                    sum_g[i] = state.get_endT();
                }

            }
        }
        _soc = std::accumulate(sum_g.begin(), sum_g.end(), 0.0); ;
        return _soc;
    }

//Return makespan cost
    double Lsrp::get_makespan() {
        // Get the last policy entry
        const std::vector<State*>& Sfrom = _S_T.back();

        // Find the maximum endT in Sfrom
        double maxT = -1.0;
        for (const auto& opt_state : Sfrom) {
            if (opt_state && opt_state->get_endT() > maxT) {
                maxT = opt_state->get_endT();
            }
        }
        _makespan = maxT;
        return maxT;
    }

// """
//        extract each agents' policy
//        """
    void Lsrp::extract_policy(){
        std::vector<std::vector<std::tuple<long, long, double, double>>> all_paths(_agents.size());

        // Add the first
        for (size_t i = 0; i < _agents.size(); ++i) {
            all_paths[i].push_back(_S_T[0][i]->get_tuple());
        }

        // Iterate over policy and extract paths
        for (size_t index = 1; index < _S_T.size(); ++index) {
            for (size_t i = 0; i < _agents.size(); ++i) {
                std::tuple<long, long, double, double> tmp = _S_T[index][i]->get_tuple();
                if (tmp != all_paths[i].back()) {
                    all_paths[i].push_back(tmp);
                }
            }
        }

        _all_paths = all_paths; // for visualize

        for (auto individu_path : all_paths) {
            TimePath timePath;
            for (size_t i = 0; i < individu_path.size(); ++i) {
                long start_node, end_node;
                double start_time, end_time;
                std::tie(start_node, end_node, start_time, end_time) = individu_path[i];

                // Insert start node and time
                if (i == 0 || start_node != end_node) {
                    timePath.nodes.push_back(start_node);
                    timePath.times.push_back(start_time);
                }

                // Insert end node and time if it's a move
                if (start_node != end_node) {
                    timePath.nodes.push_back(end_node);
                    timePath.times.push_back(end_time);
                } else if (i > 0 && start_node == end_node) { // Handle waiting times
                    timePath.times.back() = end_time;
                }
            }
            _paths.push_back(timePath);
        }
    }


    Agent *Lsrp::swap_required_possible(const std::vector<Agent *> &curr_agents, const Agent &agent,
                                        const std::vector<State *> &Sfrom, std::vector<State *> &Sto,
                                        std::vector<long> &C) {
        if (C[0] == Sfrom[agent.get_id()]->get_v()) {return nullptr;} // the agent wants to stay here
        auto aj = Check_occupied_forSwap(curr_agents,C[0],Sfrom,Sto, true);
        if (aj != nullptr && swap_required(agent,*aj,Sfrom,Sto,Sfrom[agent.get_id()]->get_v(),
                                           Sfrom[aj->get_id()]->get_v())
        && swap_possible(Sfrom,Sto,Sfrom[aj->get_id()]->get_v(),Sfrom[agent.get_id()]->get_v())) {
            return aj;
        }
        for (long u : _graph->GetSuccs(Sfrom[agent.get_id()]->get_v()))
        {
            auto ak = Check_occupied_forSwap(curr_agents,u,Sfrom,Sto, true);
            if (ak == nullptr || C[0] == Sfrom[ak->get_id()]->get_v()) { continue;}
            if (swap_required(*ak,agent,Sfrom,Sto,Sfrom[agent.get_id()]->get_v(),C[0]) &&
                    swap_possible(Sfrom,Sto,C[0],Sfrom[agent.get_id()]->get_v())) {
                return ak;
            }
        }
        return nullptr;
    }

    bool Lsrp::swap_required(const Agent &pusher, const Agent &puller, const std::vector<State *> &Sfrom,
                             std::vector<State *> &Sto,long v_pusher_init,long v_puller_init) {
        //initialize
        long v_pusher = v_pusher_init;
        long v_puller = v_puller_init;
        long next;  // the next move of puller
        while (get_h(pusher,v_puller) < get_h(pusher,v_pusher))  // avoid endless loop
        {
            std::vector<long> v_puller_nghs = _graph->GetSuccs(v_puller);
            int n = v_puller_nghs.size();
            for (long u : v_puller_nghs)
            {
                auto a = Check_occupied_forSwap({},u,Sfrom,Sto, false);
                if (u == v_pusher ||
                    (_graph->GetSuccs(u)).size() == 1 && a != nullptr && _Send[a->get_id()] == u){
                    --n;
                } else {
                    next = u;
                }
            }
            if (n >= 2) {return false;} // swap not required, because the push can satisfy the requirement
            if (n <= 0) { break;} // swap impossible -> dead end;
            // none of them  n = 1 , keep exploring
            v_pusher = v_puller;
            v_puller = next;
        }
        bool condition1 = (get_h(puller,v_pusher)< get_h(puller,v_puller));
        bool condition2 = (get_h(pusher,v_pusher) == 0
                           || get_h(pusher,v_puller)< get_h(pusher,v_pusher));
        return condition1 && condition2;
        // check if  when reach the dead end, the distance of pusher and puller to their goal are lowest among two of them
    }

    bool Lsrp::swap_possible(const std::vector<State *> &Sfrom, std::vector<State *> &Sto, long v_pusher_init,
                             long v_puller_init) {
        //initialize
        long v_pusher = v_pusher_init;
        long v_puller = v_puller_init;
        long next;  // the next move of puller
        while (v_puller != v_pusher_init)  // avoid endless loop
        {
            std::vector<long> v_puller_nghs = _graph->GetSuccs(v_puller);
            int n = v_puller_nghs.size();
            for (long u : v_puller_nghs)
            {
                auto a = Check_occupied_forSwap({},u,Sfrom,Sto, false);
                if (u == v_pusher ||
                (_graph->GetSuccs(u)).size() == 1 && a != nullptr && _Send[a->get_id()] == u){
                    --n;
                } else {
                    next = u;
                }
            }
            if (n >= 2) {return true;} // swap possible -> because there are wider place to swap,
            if (n <= 0) {return false;} // swap impossible -> dead end;
            // none of them  n = 1 , keep exploring
            v_pusher = v_puller;
            v_puller = next;
        }
        return false; // swap impossible there is a loop here
    }

    Agent *Lsrp::Check_occupied_forSwap(const std::vector<Agent *> &curr_agents, const long &u,
                                        const std::vector<State *> &Sfrom, const std::vector<State *> &Sto,
                                        bool curr_A_required) {
        if (curr_A_required) {
            for (const auto &ag_ptr: curr_agents) {
                // check id this agent satisfies the requirement that its next state is not decided yet and its last state arrives at here
                if (check_potential_deadlock(u, *ag_ptr, Sfrom, Sto)) { // 解引用指针传递给函数
                    return ag_ptr; // 返回agent对象
                }
            }
            return nullptr;
        } else {
            for (const auto &ag_ptr: _agents) {
                if (Sfrom[ag_ptr->get_id()]->get_v() == u) {
                    return ag_ptr;
                }
            }
            return nullptr;
        }
    }

       double Lsrp::_asy_push(Agent &agent, std::vector<State *> &Sto, const std::vector<State *> &Sfrom,
                       const std::vector<Agent *> &curr_agents, double tmin2, double curr_t, std::vector<long> &constrain_list, bool bp) {
        //abandoned version   The integration of push_required_possible and  pibt
        std::vector<long> C;
        if (!bp)
        {
            C = {agent.get_curr()->get_v()};
        } else
        {
            C = {};
        }
        const auto& neighbors = _graph->GetSuccs(agent.get_curr()->get_v());
        C.insert(C.end(), neighbors.begin(), neighbors.end());

        std::shuffle(C.begin(), C.end(), _rng);
        std::sort(C.begin(), C.end(), [&](const long& coord1, const long& coord2) {
            double val1 = get_duration(agent, Sfrom[agent.get_id()]->get_v(), coord1) + get_h(agent, coord1);
            double val2 = get_duration(agent, Sfrom[agent.get_id()]->get_v(), coord2) + get_h(agent, coord2);
            if (val1 != val2) {
                return val1 < val2;
            }
            if (coord1 == agent.get_curr()->get_v()) return false;
            if (coord2 == agent.get_curr()->get_v()) return true;
            return false;
        });
        if (!bp && highest_pri_agents(agent)) {
            long current_v = agent.get_curr()->get_v();
            auto it = std::find(C.begin(), C.end(), current_v);
            if (it != C.end() && std::distance(C.begin(), it) != 1) {
                C.erase(it);
                C.insert(C.begin() + 1, current_v);
            }
        }

        for (const auto& v : C) {
            if (check_Occupied(agent, v, Sto, constrain_list, bp)) {
                continue;
            }

            auto ag_opt = push_required(curr_agents, agent, v, Sfrom, Sto);
            if (ag_opt != nullptr) {
                auto ag = ag_opt;

                constrain_list.push_back(agent.get_curr()->get_v());

                double twait;
                // std::tie(twait, new_policy) = push_possible(*ag, Sto, Sfrom, curr_agents, tmin2, curr_t, new_constrain_list);
                twait = _asy_push(*ag, Sto, Sfrom, curr_agents, tmin2, curr_t, constrain_list, true);
                if (twait == -1) {
                    constrain_list.erase(
                            std::remove(constrain_list.begin(), constrain_list.end(), agent.get_curr()->get_v()),
                            constrain_list.end()
                    );
                    continue;
                }

                auto parent = Sfrom[agent.get_id()];
                State* next_state = new State(parent->get_v(), parent->get_v(), parent->get_endT(), twait);
                Sto[agent.get_id()] = next_state;
                // push possible so we wait here
                double tmove = twait + get_duration(agent,parent->get_v(),v);
                State* next_next_state = new State(parent->get_v(), v, twait, tmove);
                // at next timestamp, go to the push_required agent's place
                std::vector<std::tuple<Agent, State*>> agent_state_list;
                agent_state_list.push_back({agent, next_state});
                agent_state_list.push_back({agent, next_next_state});
                std::unordered_map<double, std::vector<State*>> new_policy;
                insert_policy(agent_state_list, new_policy);
                merge_policy(new_policy, curr_t);
                return tmove;
            } else {
                State* next_state = new State(generate_state(v, agent, Sfrom, &tmin2));
                Sto[agent.get_id()] = next_state;
                // directly insert next state because this must be the final step of push_possible
                std::unordered_map<double, std::vector<State*>> new_policy;
                std::vector<std::tuple<Agent, State*>> agent_state_list;
                agent_state_list.push_back({agent, next_state});
                insert_policy(agent_state_list, new_policy);
                merge_policy(new_policy, curr_t);
                return next_state->get_endT();
            }
        }
        return -1;
    }

    double Lsrp::_asy_push_swap(Agent &agent, std::vector<State *> &Sto, const std::vector<State *> &Sfrom,
                       const std::vector<Agent *> &curr_agents, double tmin2, double curr_t,std::vector<long> &constrain_list, bool bp) {
        //abandoned version   The integration of push_required_possible and  pibt
        std::vector<long> C;
        if (!bp)
        {
            C = {agent.get_curr()->get_v()};
        } else
        {
            C = {};
        }
        const auto& neighbors = _graph->GetSuccs(agent.get_curr()->get_v());
        C.insert(C.end(), neighbors.begin(), neighbors.end());

        std::shuffle(C.begin(), C.end(), _rng);
        std::sort(C.begin(), C.end(), [&](const long& coord1, const long& coord2) {
            double val1 = get_duration(agent, Sfrom[agent.get_id()]->get_v(), coord1) + get_h(agent, coord1);
            double val2 = get_duration(agent, Sfrom[agent.get_id()]->get_v(), coord2) + get_h(agent, coord2);
            if (val1 != val2) {
                return val1 < val2;
            }
            if (coord1 == agent.get_curr()->get_v()) return false;
            if (coord2 == agent.get_curr()->get_v()) return true;
            return false;
        });
        auto ak = swap_required_possible(curr_agents,agent,Sfrom,Sto,C);
        if (ak != nullptr) {
            std::reverse(C.begin(), C.end());
        }

        if (!bp && highest_pri_agents(agent)) {
            long current_v = agent.get_curr()->get_v();
            auto it = std::find(C.begin(), C.end(), current_v);
            if (it != C.end() && std::distance(C.begin(), it) != 1) {
                C.erase(it); // 移除当前节点
                C.insert(C.begin() + 1, current_v); // 插入到第二个位置 } }
            }
        }

            for (const auto& v : C) {
                if (check_Occupied(agent, v, Sto, constrain_list, bp)) {
                    continue;
                }

                auto ag_opt = push_required(curr_agents, agent, v, Sfrom, Sto);
                if (ag_opt != nullptr) {
                    auto ag = ag_opt;

                    constrain_list.push_back(agent.get_curr()->get_v());

                    double twait;
                    // std::tie(twait, new_policy) = push_possible(*ag, Sto, Sfrom, curr_agents, tmin2, curr_t, new_constrain_list);
                    twait = _asy_push_swap(*ag, Sto, Sfrom, curr_agents, tmin2, curr_t, constrain_list, true);
                    if (twait == -1) {
                        // not push_possible  we go with
                        constrain_list.erase(
                                std::remove(constrain_list.begin(), constrain_list.end(), agent.get_curr()->get_v()),
                                constrain_list.end()
                        );
                        continue;
                    }

                    std::unordered_map<double, std::vector<State*>> new_policy;
                    auto parent = Sfrom[agent.get_id()];
                    State* next_state = new State(parent->get_v(), parent->get_v(), parent->get_endT(), twait);
                    Sto[agent.get_id()] = next_state;
                    // push possible so we wait here
                    double tmove = twait + get_duration(agent,parent->get_v(),v);
                    State* next_next_state = new State(parent->get_v(), v, twait, tmove);
                    // at next timestamp, go to the push_required agent's place
                    std::vector<std::tuple<Agent, State*>> agent_state_list;
                    agent_state_list.push_back({agent, next_state});
                    agent_state_list.push_back({agent, next_next_state});
                    if (!bp && v == C.front() && v != Sfrom[agent.get_id()]->get_v() && ak != nullptr &&
                    Sto[ak->get_id()] == nullptr) {
                        const State* parent_ak = Sfrom[ak->get_id()];
                        State* next_ak_state = new State(parent_ak->get_v(),parent_ak->get_v(),
                                                         parent_ak->get_endT(),tmove);
                        Sto[ak->get_id()] = next_ak_state;
                        State* next_next_ak_state = new State(parent_ak->get_v(),Sfrom[agent.get_id()]->get_v(),
                                                              tmove,tmove + get_duration(*ak,parent_ak->get_v(),Sfrom[agent.get_id()]->get_v()));
                        agent_state_list.push_back({*ak, next_ak_state});
                        agent_state_list.push_back({*ak, next_next_ak_state});
                    }
                    insert_policy(agent_state_list, new_policy);
                    merge_policy(new_policy, curr_t);
                    return tmove;
                } else {
                    State* next_state = new State(generate_state(v, agent, Sfrom, &tmin2));
                    Sto[agent.get_id()] = next_state;
                    std::vector<std::tuple<Agent, State*>> agent_state_list;
                    std::unordered_map<double, std::vector<State*>> new_policy;
                    if (!bp && v == C.front() && v != Sfrom[agent.get_id()]->get_v() && ak != nullptr &&
                    Sto[ak->get_id()] == nullptr) {
                        const State* parent_ak = Sfrom[ak->get_id()];
                        State* next_ak_state = new State(parent_ak->get_v(),parent_ak->get_v(),
                                                         parent_ak->get_endT(),curr_t + get_duration(agent,Sfrom[agent.get_id()]->get_v(),v));
                        Sto[ak->get_id()] = next_ak_state;
                        State* next_next_ak_state = new State(parent_ak->get_v(),Sfrom[agent.get_id()]->get_v(),
                                                              curr_t + get_duration(agent,Sfrom[agent.get_id()]->get_v(),v),
                                                              curr_t + get_duration(agent,Sfrom[agent.get_id()]->get_v(),v) + get_duration(*ak,parent_ak->get_v(),Sfrom[agent.get_id()]->get_v()));
                        agent_state_list.push_back({*ak, next_ak_state});
                        agent_state_list.push_back({*ak, next_next_ak_state});
                    }
                    agent_state_list.push_back({agent, next_state});
                    insert_policy(agent_state_list, new_policy);
                    merge_policy(new_policy, curr_t);
                    return next_state->get_endT();
                }
            }
        return -1;
    }

    int Lsrp::_lsrp() {
        _T.push(0.0);  // start time: 0 for all
        _T_set.insert(0.0);

        // Set a timeout limit of 30 seconds
//        std::chrono::seconds timeout_limit(30);
        std::chrono::duration<double> timeout_limit(_time_limit);
        auto start_time = std::chrono::steady_clock::now();
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (elapsed_time > timeout_limit) {
                std::chrono::duration<double> duration = current_time - start_time;
                _runtime =  duration.count();
                return {};
            }


            // get the current t
            double t = _T.top();
            _T.pop();
            _T_set.erase(t);

            // get the next t
            double t2 = get_tmin2();
            const auto& S_prev = _S_T.back();
            if (reach_Goal()) {
                auto current_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> duration = current_time - start_time;
                _runtime = duration.count();
                get_makespan();
                get_Soc();
                extract_policy();
                return 1;
            }

            // extract the agents who should move in this planning loop
            auto curr_agents = extract_Agents(t);

            // generate raw Sto from Sfrom and curr_agents
            std::vector<State*> Snext = get_rawSnext(_S_T.back(), curr_agents, t);

            // update priority
            update_Priority();

            // Sort the agents by their priority
            std::sort(curr_agents.begin(), curr_agents.end(), [](const Agent* a, const Agent* b) {
                return a->get_priority() > b->get_priority();
            });


            // Generate path
            for (auto& agent : curr_agents) {
                if (Snext[agent->get_id()] == nullptr) {
                    if (_swap) {
                        std::vector<long> const_list;
                        _asy_push_swap(*agent, Snext,S_prev, curr_agents,t2, t,const_list,false);
                    } else {
                        std::vector<long> const_list;
                        _asy_push(*agent, Snext, S_prev, curr_agents, t2, t,const_list,false);
                    }
                }
            }

            // 1.update time list, 2.set agents curr, 3.add Sto to policy
            update(curr_agents, Snext);
        }
    }

    double Lsrp::re_makespan() {
        return _makespan;
    }

    double Lsrp::re_soc() {
        return _soc;
    }

    TimePathSet Lsrp::GetPlan(long nid) {
        return _paths;
    }

    std::unordered_map<std::string, double> Lsrp::GetStats() {
        return _stats;
    }

    bool Lsrp::highest_pri_agents(Agent &agent) {
        double pri = agent.get_priority();
        for (int i = 0; i < _agents.size(); i++) {
            if (_agents[i]->get_priority() > pri) {return false;}
        }
        return true;
    }

}