
/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include "mapfaa_util.hpp"

namespace raplab{
    MAPFAAPlanner::MAPFAAPlanner() {};

    MAPFAAPlanner::~MAPFAAPlanner() {};

    void MAPFAAPlanner::SetGraphPtr(raplab::PlannerGraph *g) {
        _graph = g;
        _starts.clear();
        _goals.clear();
    }

}