//
// Created by David Zhou on 24-7-15.
//

#ifndef CPPRAPLAB_MAPFAA_LSRP_HPP
#define CPPRAPLAB_MAPFAA_LSRP_HPP

#include "mapfaa_util.hpp"
#include <vector>
#include <tuple>
#include <queue>
#include <unordered_map>
#include <random>
#include <optional>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <iostream>
#include <string>
#include <cassert>


namespace raplab {

#define Debug_asyPibt 0
#define Makespan 0
#define Soc 0
#define Distance_sort 0
#define Duration_sort 1
#define Swap 1
    /*
    struct nullopt_t {
        constexpr nullopt_t(int) {}  // 构造函数用于禁止隐式转换
    };

    constexpr nullopt_t nullopt{0};  // 定义一个全局的 nullopt 常量


    template <typename T>
    class Optional {
    public:
        Optional() : has_value(false) {}

        Optional(nullopt_t) : has_value(false) {}

        Optional(const T& value) : has_value(true) {
            new (&storage) T(value); // 在预分配的内存中构造 T 对象
        }

        Optional(T&& value) : has_value(true) {
            new (&storage) T(std::move(value)); // 在预分配的内存中构造 T 对象
        }

        Optional(const Optional& other) : has_value(other.has_value) {
            if (other.has_value) {
                new (&storage) T(*other); // 复制构造
            }
        }

        Optional(Optional&& other) noexcept : has_value(other.has_value) {
            if (other.has_value) {
                new (&storage) T(std::move(*other)); // 移动构造
            }
        }

        Optional& operator=(const Optional& other) {
            if (this != &other) {
                if (has_value) {
                    reinterpret_cast<T*>(&storage)->~T(); // 销毁旧值
                }
                has_value = other.has_value;
                if (other.has_value) {
                    new (&storage) T(*other); // 复制构造
                }
            }
            return *this;
        }

        Optional& operator=(Optional&& other) noexcept {
            if (this != &other) {
                if (has_value) {
                    reinterpret_cast<T*>(&storage)->~T(); // 销毁旧值
                }
                has_value = other.has_value;
                if (other.has_value) {
                    new (&storage) T(std::move(*other)); // 移动构造
                }
            }
            return *this;
        }

        Optional& operator=(nullopt_t) {
            if (has_value) {
                reinterpret_cast<T*>(&storage)->~T(); // 销毁旧值
                has_value = false;
            }
            return *this;
        }

        ~Optional() {
            if (has_value) {
                reinterpret_cast<T*>(&storage)->~T(); // 销毁值
            }
        }

        explicit operator bool() const {
            return has_value;
        }

        T& operator*() {
            assert(has_value);
            return *reinterpret_cast<T*>(&storage);
        }

        const T& operator*() const {
            assert(has_value);
            return *reinterpret_cast<const T*>(&storage);
        }

        T* operator->() {
            assert(has_value);
            return reinterpret_cast<T*>(&storage);
        }

        const T* operator->() const {
            assert(has_value);
            return reinterpret_cast<const T*>(&storage);
        }

        void reset() {
            if (has_value) {
                reinterpret_cast<T*>(&storage)->~T();
                has_value = false;
            }
        }

        const T& value() const {
            assert(has_value);
            return *reinterpret_cast<const T*>(&storage);
        }

        T& value() {
            assert(has_value);
            return *reinterpret_cast<T*>(&storage);
        }

        bool has_value;

    private:
        alignas(T) unsigned char storage[sizeof(T)];

    };
     */

    struct State {
    public:
        State();

        State(long parent_v, long v, double parent_time, double time);

        bool operator==(const State &other) const;

        std::size_t hash() const;

        std::tuple<long, long, double, double> get_tuple() const;

        double get_endT() const;

        double get_startT() const;

       long get_v() const;

        long get_p() const;


    private:
        long p;
        long v;
        double startT;
        double endT;


    };

    struct StateHash {
        std::size_t operator()(const State& s) const {
            return s.hash();
        }
    };


    struct Agent {
    public:
        Agent();

        Agent(int id, long start, long goal);

        void set_init_priority(double priority);

        void set_priority(double pri);

        int get_id() const;

        State* get_curr() const;

        bool operator==(const Agent &other) const;

        bool is_at_goal() const;

        double get_init_priority() const;

        double get_priority() const;

        void set_curr(State* &curr);

        void set_at_goal(bool at_goal);

        long get_goal() const;

        State* curr;

    private:
        int id;
        double priority;
        double init_pri;
        long goal;
        bool at_goal;


    };

    class Lsrp : public MAPFAAPlanner {
    public:

        Lsrp();

        /**
        *
        */
        virtual ~Lsrp();


        virtual bool reach_Goal() const;

        virtual double get_tmin2() const;

        virtual std::vector<Agent *> extract_Agents(double t);

        virtual std::vector<State*> get_rawSto(std::vector<State*> S_from,
                                                             const std::vector<Agent *> &curr_agents, double t) const;

        virtual void update_Priority();

        virtual double get_duration(const Agent &agent) const;

        virtual bool
        check_Occupied(const Agent &agent, const long &v, const std::vector<State*> &Sto,
                       const std::vector<long> &constrain_list, bool in_pibt) const;

        virtual bool check_potential_deadlock(const long &v, const Agent &ag,
                                              const std::vector<State*> &Sfrom,
                                              const std::vector<State*> &Sto) const;

        virtual double get_makespan();

        virtual double get_Soc();

        virtual void add_lastStep();


        void extract_policy();


        /* integration of pibt and push_required and push_possible
        // not using anymore
        std::tuple<double, std::unordered_map<double, std::vector<State*>>>
        asynchronous_Pibt(const Agent &agent, std::vector<State*> &Sto,
                          const std::vector<State*> &Sfrom, const std::vector<Agent *> &curr_agents,
                          double tmin2, double curr_t, const std::vector<long> &constrain_list,
                          bool in_pibt);
        */

        int solve();

        virtual double re_soc() ;

        virtual double re_makespan() ;

        virtual CostVec GetPlanCost(long nid=-1) override ;

        virtual TimePathSet GetPlan(long nid=-1) override ;

        virtual double GetRuntime(long nid = -1) {return _runtime;}

        virtual int Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) override ;

        virtual std::unordered_map<std::string, double> GetStats() override ;

        virtual int _Solve(std::vector<long>& starts, std::vector<long>& goals, double time_limit,std::vector<double> duration);

        virtual std::tuple<double, std::unordered_map<double, std::vector<State*>>> push_possible
                (const Agent &agent, std::vector<State*> &Sto,
                 const std::vector<State*> &Sfrom, const std::vector<Agent *> &curr_agents,
                 double tmin2, double curr_t, const std::vector<long> &constrain_list);

        virtual Agent*
        push_required(const std::vector<Agent *> &curr_agents, const Agent &agent, const long &v,
                      const std::vector<State*> &Sfrom, const std::vector<State*> &Sto) const;

        virtual int asy_pibt(const Agent &agent, std::vector<State*> &Sto,
                             const std::vector<State*> &Sfrom, const std::vector<Agent *> &curr_agents,
                             double tmin2, double curr_t);


        virtual Agent* swap_required_possible(const std::vector<Agent *> &curr_agents,const Agent &agent,
                                              const std::vector<State*> &Sfrom,std::vector<State*> &Sto,
                                              std::vector<long> &C);

        virtual bool swap_required(const Agent &pusher,const Agent &puller,const std::vector<State*> &Sfrom,
                                   std::vector<State*> &Sto,long v_pusher_init,long v_puller_init);

        virtual bool swap_possible(const std::vector<State*> &Sfrom,std::vector<State*> &Sto,
                                   long v_pusher_init,long v_puller_init);

        virtual Agent* Check_occupied_forSwap(const std::vector<Agent*>& curr_agents,
                                              const long& u,
                                              const std::vector<State*>& Sfrom,
                                              const std::vector<State*>& Sto, bool curr_A_required);

        virtual int asy_pibt_swap(Agent &agent, std::vector<State*> &Sto,
                             const std::vector<State*> &Sfrom, const std::vector<Agent *> &curr_agents,
                             double tmin2, double curr_t);

        virtual std::tuple<double, std::unordered_map<double, std::vector<State*>>> push_possible_swap
                (Agent &agent, std::vector<State*> &Sto,
                 const std::vector<State*> &Sfrom, const std::vector<Agent *> &curr_agents,
                 double tmin2, double curr_t, const std::vector<long> &constrain_list, Agent &pusher);


    private:

        void insert_policy(const std::vector<std::tuple<Agent, State*>> &agent_state_list,
                           std::unordered_map<double, std::vector<State*>> &new_policy) const;

        void
        merge_policy(const std::unordered_map<double, std::vector<State*>> &new_policy, double curr_t);

        void update(const std::vector<Agent *> &curr_agents, std::vector<State*> Sto);

        std::vector<std::unordered_map<long,int>> generate_distable();

        std::unordered_map<long,int> generate_single_dis_table(int agent, double duration);

        int get_h(const Agent &agent, const long &coord);

        void set_agents();

        std::vector<std::vector<State*>> set_initPolicy();

        State generate_state(const long &v, const Agent &agent,
                             const std::vector<State*> &Sfrom,
                             double* tmin2) const;

        std::vector<long> _Sinit;
        std::vector<long> _Send;
        std::vector<double> _duration;
        std::vector<std::unordered_map<long,int>> _dis_table;
        std::priority_queue<double, std::vector<double>, std::greater<double>> _time_list;
        // Set to keep track of the timestamps present in time_list
        std::unordered_set<double> _time_set;

        std::unordered_map<double, std::vector<State*>> _t_policy;
        std::vector<Agent*> _agents;
        mutable std::vector<std::vector<State*>> _policy;
        double _min_duration;
        double _soc;
        double _makespan;
        double _time_limit;
        double _runtime;
        std::mt19937 _rng;
        std::unordered_map<std::string, double> _stats;
        TimePathSet _paths;
    };
}

#endif //CPPRAPLAB_MAPFAA_LSRP_HPP
