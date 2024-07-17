//
// Created by David Zhou on 24-7-15.
//

#include "mapfaa_lsrp.hpp"
#include <functional>

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


    /**
     * Lsrp part
     * @param nid
     * @return
     */
    //Lsrp part main function

    Lsrp::Lsrp() {};

    Lsrp::~Lsrp() {};

    CostVec Lsrp::GetPlanCost(long nid) {
        CostVec out(_graph->CostDim(), 0);
        if (Soc && !Makespan) {
            out[0] = re_soc();
        } else if (!Soc && Makespan){
            out[0] = re_makespan();
        } else {
            out[0] = re_soc();
            out[1] = re_makespan();
        }
        return out;
    }

    int Lsrp::Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, double eps)
    {
        return 1;
        // useless input, fake function
    }

    int
    Lsrp::_Solve(std::vector<long> &starts, std::vector<long> &goals, double time_limit, std::vector<double> duration) {
        if(Debug_asyPibt) {std::cout<<"First layer solve arrives"<<std::endl;}
        _Sinit = starts;
        _Send = goals;
        _duration = duration;
        _dis_table = generate_distable();
        if(Debug_asyPibt) {std::cout<<"Dis_table successfully generated"<<std::endl;}
        set_agents();
        if(Debug_asyPibt) {std::cout<<"Agent set"<<std::endl;}
        _policy = set_initPolicy();
        if(Debug_asyPibt) {std::cout<<"POlicy generated"<<std::endl;}
        _min_duration = *std::min_element(_duration.begin(), _duration.end());
        _time_limit = time_limit;
        solve();
        return 1;
    }

// A list stored all the distance_table for each agent.
// Heuristic function related
    std::vector<std::unordered_map<long, int>> Lsrp::generate_distable() {
        std::vector<std::unordered_map<long, int>> distable;
        for (size_t i = 0; i < _Sinit.size(); ++i) {
            distable.push_back(generate_single_dis_table(static_cast<int>(i), _duration[i]));
        }
        return distable;
    }

//A method generate bfs value for each agent, each coordination corresponds to a specific value and were used as
//heuristic value for each state
//Because bfs promises the optimal path for a single agent and it is relatively fast
    std::unordered_map<long, int> Lsrp::generate_single_dis_table(int agent, double duration) {
        std::deque<long> tmp;
        tmp.push_back(_Send[agent]);
        const int inf = std::numeric_limits<int>::max();
        std::unordered_map<long, int> dist_table;

        dist_table[_Send[agent]] = 0;

        while (!tmp.empty()) {
            long curr = tmp.front();
            tmp.pop_front();
            std::vector<long> successors = _graph->GetSuccs(curr);
            for (long neigh : successors) {
                if (dist_table.find(neigh) == dist_table.end() || dist_table[neigh] > dist_table[curr] + duration) {
                    dist_table[neigh] = dist_table[curr] + duration;
                    tmp.push_back(neigh);
                }
            }
        }
        return dist_table;
    }

//A function calculates heuristic value of a given state
//        take in a serial number of a agent eg: 1
//        and a coordination eg: (0, 0)
//        return and int value
    int Lsrp::get_h(const Agent& agent, const long& coord) {
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

        std::vector<std::pair<int, double>> tmp(_duration.size());
        // sort by duration
        if (Duration_sort) {
            for (size_t i = 0; i < _duration.size(); ++i) {
                tmp[i] = {i, _duration[i]};
            }
        } else {
            // sort by distance
            for (size_t i = 0; i < _duration.size(); ++i) {
                tmp[i] = {i, double(_dis_table[i][_Sinit[i]])};
            }
        }

        std::sort(tmp.begin(), tmp.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
            return a.second < b.second;
        });

        for (size_t index = 0; index < tmp.size(); ++index) {
            _agents[tmp[index].first]->set_init_priority(index * gap);
        }
    }

// A method set initial states
// eg: [(s11, s21, s31, s41, ·······)]
// sort agent by their duration， the bigger the duration， the higher the priority
// also set occupation and referred time policy
    std::vector<std::vector<State*>> Lsrp::set_initPolicy() {
        std::vector<std::vector<State*>> policy;
        std::vector<State*> States_init;
        States_init.reserve(_Sinit.size());
        for (int i = 0; i< _agents.size(); i++) {
            States_init.push_back(_agents[i]->curr);
        }
        policy.push_back(States_init); // 直接插入 vector<State>
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
        if (_time_list.empty()) {
            return -1; // Return NaN if there is no next time
        }
        return _time_list.top();
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
    std::vector<State*> Lsrp::get_rawSto(std::vector<State*> S_from,
                                                           const std::vector<Agent*>& curr_agents, double t) const {
        std::vector<State*> re_S;
        const std::vector<State*>* t_policy_ptr = nullptr;

        // 获取当前时间 t 对应的策略
        auto it = _t_policy.find(t);
        if (it != _t_policy.end()) {
            t_policy_ptr = &(it->second);
        }

        // 遍历 S_from
        for (size_t index = 0; index < S_from.size(); ++index) {
            // 检查 agents[index] 是否在 curr_agents 中
            Agent* agent_ptr = _agents[index];
            if (std::find(curr_agents.begin(), curr_agents.end(), agent_ptr) != curr_agents.end()) {
                // 如果 t_policy_ptr 不是 nullptr，则将 t_policy_ptr[index] 添加到 re_S 中
                if (t_policy_ptr != nullptr) {
                    re_S.push_back((*t_policy_ptr)[index]);
                } else {
                    re_S.push_back(nullptr);
                }
            } else {
                // 否则，将 S_from[index] 添加到 re_S 中
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
    double Lsrp::get_duration(const Agent& agent) const {
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
        double endT = parent.get_endT() + get_duration(agent);
        return State(parent.get_v(), v, parent.get_endT(), endT);
    }

//Used in get_successor
//Check if a coordination(Vertex) is occupied by some states in the Sto and Sfrom.v
//the collision model wo used here is the same as lsrm*
//No worry edge collision， p is occupied and no swap would happened
//Also avoid that vertex is being pibted
    bool Lsrp::check_Occupied(const Agent& agent, const long& v,
                                  const std::vector<State*>& Sto,
                                  const std::vector<long>& constrain_list, bool in_pibt) const {
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

        if (in_pibt) {
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

//A method check if the low priority deadlock situation is going to happened
//If it is, Return True and starts the asy-PIBT process
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
    Agent* Lsrp::pi_needed(const std::vector<Agent*>& curr_agents, const Agent& agent,
                                             const long& v,
                                             const std::vector<State*>& Sfrom,
                                             const std::vector<State*>& Sto) const {
        for (const auto& ag_ptr : curr_agents) {
            if (*ag_ptr == agent) { // 解引用指针并比较
                continue;
            }
            if (check_potential_deadlock(v, *ag_ptr, Sfrom, Sto)) { // 解引用指针传递给函数
                return ag_ptr; // 返回智能体对象
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
        for (const auto& [t, S_t] : new_policy) {
            auto it = _t_policy.find(t);
            if (it == _t_policy.end()) {
                _t_policy[t] = S_t;
                if (_time_set.find(t) == _time_set.end() && t != curr_t) {
                    _time_list.push(t);
                    _time_set.insert(t);
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
        _policy.push_back(Sto); // 将 Sto 添加到 policy 中

        for (Agent* agent : curr_agents) {  // 使用指针遍历
            int id = agent->get_id();       // 通过指针访问成员函数
            agent->set_curr(Sto[id]);      // 更新 agent 的 curr 状态

            // Input time
            double endT = Sto[id]->get_endT();
            if (_time_set.find(endT) == _time_set.end()) {
                _time_list.push(endT);
                _time_set.insert(endT);
            }

            // Update each at_goal
            if (Sto[id]->get_v() == agent->get_goal()) {
                agent->set_at_goal(true);
            } else {
                // Some agents might leave their goal point to make space for others
                agent->set_at_goal(false);
            }
        }

        if (Debug_asyPibt) {
            for (int i = 0; i < Sto.size(); i++) {
                std::cout<<Sto[i]->get_v()<<" ";
            }
            std::cout<<std::endl;
        }
    }


// Calculate the soc cost of the algorithm
    double Lsrp::get_Soc() {
        double g = 0.0;

        // Iterate through policy and calculate social cost
        for (size_t index = 1; index < _policy.size(); ++index) {
            const std::vector<State*>& Q = _policy[index];
            const std::vector<State*>& prev_Q = _policy[index - 1];

            // Assuming Q and prev_Q have the same size
            for (size_t i = 0; i < Q.size(); ++i) {
                const State& state = *Q[i];
                const State& prev_state = *prev_Q[i];

                // Compare current state with previous state
                if (state == prev_state) {
                    continue;
                }

                // Calculate duration difference
                g += state.get_endT() - state.get_startT();
            }
        }
        _soc = g;
    }

//Return makespan cost
    double Lsrp::get_makespan() {
        // Get the last policy entry
        const std::vector<State*>& Sfrom = _policy.back();

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

//useless function no used at all just for test
    void Lsrp::add_lastStep() {
        // Get the last policy entry
        std::vector<State*> Sfrom = _policy.back();

        // Get the max T
        double maxT = get_makespan();

        // Extract agents at maxT
        std::vector<Agent *> max_agents = extract_Agents(maxT);

        // Prepare final state vector S_final
        std::vector<State*> S_final;
        for (size_t index = 0; index < _agents.size(); ++index) {
            Agent *agent = _agents[index];
            if (std::find_if(max_agents.begin(), max_agents.end(),
                             [&agent](Agent* a) { return a == agent; }) != max_agents.end()) {
                S_final.push_back(Sfrom[index]);
            } else {
                double startT = Sfrom[index]->get_endT();
                long v = Sfrom[index]->get_v();
                S_final.push_back(new State(v, v, startT, maxT));
            }
        }

        // Append S_final to policy
        _policy.push_back(S_final);
    }

// """
//        extract each agents' policy
//        """
    std::vector<std::vector<std::tuple<long, long, double, double>>> Lsrp::extract_policy() const {
        std::vector<std::vector<std::tuple<long, long, double, double>>> paths(_agents.size());

        // Add the first
        for (size_t i = 0; i < _agents.size(); ++i) {
            paths[i].push_back(_policy[0][i]->get_tuple());
        }

        // Iterate over policy and extract paths
        for (size_t index = 1; index < _policy.size(); ++index) {
            for (size_t i = 0; i < _agents.size(); ++i) {
                std::tuple<long, long, double, double> tmp = _policy[index][i]->get_tuple();
                if (tmp != paths[i].back()) {
                    paths[i].push_back(tmp);
                }
            }
        }

        return paths;
    }


// Existing implementations...

    std::tuple<double, std::unordered_map<double, std::vector<State*>>> Lsrp::asynchronous_Pibt(const Agent& agent, std::vector<State*> &Sto, const std::vector<State*>& Sfrom, const std::vector<Agent*>& curr_agents, double tmin2, double curr_t, const std::vector<long>& constrain_list, bool in_pibt) {
        std::vector<long> C = {agent.get_curr()->get_v()};
        const auto& neighbors = _graph->GetSuccs(agent.get_curr()->get_v());
        C.insert(C.end(), neighbors.begin(), neighbors.end());

        std::shuffle(C.begin(), C.end(), _rng);
        std::sort(C.begin(), C.end(), [&](const long& coord1, const long& coord2) {
            return get_h(agent, coord1) < get_h(agent, coord2);
        });

        for (const auto& v : C) {
            if (check_Occupied(agent, v, Sto, constrain_list, in_pibt)) {
                continue;
            }

            auto ag_opt = pi_needed(curr_agents, agent, v, Sfrom, Sto);
            if (ag_opt != nullptr) {
                auto ag = ag_opt;

                std::vector<long> new_constrain_list;
                if (in_pibt) {
                    new_constrain_list = constrain_list;
                    new_constrain_list.push_back(agent.get_curr()->get_v());
                } else {
                    new_constrain_list = {Sfrom[agent.get_id()]->get_v()};
                }

                double twait;
                std::unordered_map<double, std::vector<State*>> new_policy;
                std::tie(twait, new_policy) = asynchronous_Pibt(*ag, Sto, Sfrom, curr_agents, tmin2, curr_t, new_constrain_list, true);

                if (twait == -1) {
                    if (in_pibt) {
                        return std::make_tuple(-1, std::unordered_map<double, std::vector<State*>>());
                    } else {
                        continue;
                    }
                }

                auto parent = Sfrom[agent.get_id()];
                State* next_state = new State(parent->get_v(), parent->get_v(), parent->get_endT(), twait);
                Sto[agent.get_id()] = next_state;

                double tmove = twait + get_duration(agent);
                State* next_next_state = new State(parent->get_v(), v, twait, tmove);

                std::vector<std::tuple<Agent, State*>> agent_state_list;
                agent_state_list.push_back({agent, next_state});
                agent_state_list.push_back({agent, next_next_state});

                insert_policy(agent_state_list, new_policy);

                if (in_pibt) {
                    return std::make_tuple(tmove, new_policy);
                } else {
                    merge_policy(new_policy, curr_t);
                    return std::make_tuple(true, std::unordered_map<double, std::vector<State*>>());
                }
            } else {
                if (in_pibt) {
                    auto parent = Sfrom[agent.get_id()];
                    double tmove = parent->get_endT() + get_duration(agent);
                    State* next_state = new State(parent->get_v(), v, parent->get_endT(), tmove);
                    Sto[agent.get_id()] = next_state;

                    std::unordered_map<double, std::vector<State*>> new_policy;
                    std::vector<std::tuple<Agent, State*>> agent_state_list;
                    agent_state_list.push_back({agent, next_state});
                    insert_policy(agent_state_list, new_policy);

                    return std::make_tuple(tmove, new_policy);
                } else {
                    State* next_state = new State(generate_state(v, agent, Sfrom, &tmin2));
                    Sto[agent.get_id()] = next_state;
                    return std::make_tuple(-1, std::unordered_map<double, std::vector<State*>>());
                }
            }
        }

        if (in_pibt) {
            return std::make_tuple(-1, std::unordered_map<double, std::vector<State*>>());
        } else {
            return std::make_tuple(true, std::unordered_map<double, std::vector<State*>>());
        }
    }


    std::vector<std::vector<std::tuple<long, long, double, double>>> Lsrp::solve() {
        if(Debug_asyPibt) {std::cout<<"Second layer solve arrives"<<std::endl;}
        _time_list.push(0.0);  // start time: 0 for all
        _time_set.insert(0.0);

        // Set a timeout limit of 30 seconds
        std::chrono::seconds timeout_limit(30);
        auto start_time = std::chrono::steady_clock::now();
        if(Debug_asyPibt){std::cout<<"loop begin"<<std::endl;}
        while (true) {
            /* Todo  unindex the time limit before we finally put in use
            // time limit check
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            if (elapsed_time > timeout_limit) {
                //std::cerr << "Timeout limit exceeded (30 seconds). Aborting.\n";
                // Handle timeout failure, return an appropriate failure state or throw an exception
                // For demonstration, returning an empty policy
                return {};
            }
             */


            // get the current t
            double t = _time_list.top();
            _time_list.pop();
            _time_set.erase(t);

            // get the next t
            double t2 = get_tmin2();
            const auto& Sfrom = _policy.back();


            if (reach_Goal()) {
                if(Debug_asyPibt){std::cout<<"Solution found"<<std::endl;}
                add_lastStep();
                get_makespan();
                get_Soc();
                return extract_policy();
            }

            // extract the agents who should move in this planning loop
            auto curr_agents = extract_Agents(t);

            // generate raw Sto from Sfrom and curr_agents
            std::vector<State*> Sto = get_rawSto(_policy.back(), curr_agents, t);

            // update priority
            update_Priority();

            // Sort the agents by their priority
            std::sort(curr_agents.begin(), curr_agents.end(), [](const Agent* a, const Agent* b) {
                return a->get_priority() > b->get_priority();
            });


            // Generate path
            for (auto& agent : curr_agents) {
                if (Sto[agent->get_id()] == nullptr) {
                    asynchronous_Pibt(*agent, Sto, Sfrom, curr_agents, t2, t, {}, false);
                }
            }

            // 1.update time list, 2.set agents curr, 3.add Sto to policy
            update(curr_agents, Sto);
        }
    }

    double Lsrp::re_makespan() {
        return _makespan;
    }

    double Lsrp::re_soc() {
        return _soc;
    }

    TimePathSet Lsrp::GetPlan(long nid) {
        return raplab::TimePathSet();
    }

    std::unordered_map<std::string, double> Lsrp::GetStats() {
        return _stats;
    }


}