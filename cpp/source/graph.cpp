  
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "graph.hpp"
#include "vec_type.hpp"

namespace raplab{


// ############################################################
// ############################################################
// ############################################################

SparseGraph::SparseGraph() {};

SparseGraph::~SparseGraph() {};

bool SparseGraph::HasVertex(long v) {
  return v < _to.size();
};

bool SparseGraph::HasArc(long v, long u) {
  for (auto k : _to[v]){
    if (k == u) {return true;}
  }
  return false;
};

std::vector<long> SparseGraph::GetSuccs(long v) {
  if (!HasVertex(v)) {return std::vector<long>(); }
  return _to[v];
};

std::vector<long> SparseGraph::GetPreds(long v) {
  if (!HasVertex(v)) {return std::vector<long>(); }
  return _from[v];
};

CostVec SparseGraph::GetCost(long u, long v) {
  for (size_t idx = 0; idx < _to_cost[u].size(); idx++){
    if (_to[u][idx] == v) {
      return _to_cost[u][idx];
    }
  }
  return std::vector<double>();
};

std::vector< CostVec > SparseGraph::GetSuccCosts(long u) {
  if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
  return _to_cost[u];
};

std::vector< CostVec > SparseGraph::GetPredCosts(long u) {
  if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
  return _from_cost[u];
};

size_t SparseGraph::NumVertex() {
  return _to.size();
};

size_t SparseGraph::NumArc() {
  return _n_arc;
};

size_t SparseGraph::NumEdge() {
  if (_n_arc % 2 != 0) {
    std::cout << "[ERROR] SparseGraph::NumEdge is not an integer but a fraction" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::NumEdge is not an integer but a fraction");
  }
  return size_t(_n_arc / 2);
};

size_t SparseGraph::CostDim() {
  return _cdim ;
};

std::vector<long> SparseGraph::AllVertex() 
{
  std::vector<long> out;
  for (int i = 0; i < _to.size(); i++){
    out.push_back(i);
  }
  return out;
};

void SparseGraph::AddVertex(long v) {
  if (!HasVertex(v)) {
    _to.resize(v+1);
    _to_cost.resize(v+1);
    _from.resize(v+1);
    _from_cost.resize(v+1);
  }
  return;
};

void SparseGraph::AddEdge(long u, long v, std::vector<double> c) {
  AddArc(u,v,c);
  AddArc(v,u,c);
  return ;
};

void SparseGraph::AddArc(long u, long v, std::vector<double> c) {
  if (!HasVertex(u)) {
    AddVertex(u);
  }
  if (!HasVertex(v)) {
    AddVertex(v);
  }

  if (_cdim == 0) {
    _cdim = c.size();
  }else{
    if (_cdim != c.size()) {
      std::cout << "[ERROR] SparseGraph::AddArc cdim does not match: " << _cdim << " != " << c.size() << std::endl;
      throw std::runtime_error("[ERROR] SparseGraph::AddArc cdim does not match");
    }
  }

  bool updated = false;
  for (size_t idx = 0; idx < _to[u].size(); idx++){
    if (_to[u][idx] == v) {
      _to_cost[u][idx] = c;
      updated = true;
      break;
    }
  }
  if (!updated) {
    _to[u].push_back(v);
    _to_cost[u].push_back(c);
    _n_arc++;
  }

  updated = false;
  for (size_t idx = 0; idx < _from[v].size(); idx++){
    if (_from[v][idx] == u) {
      _from_cost[v][idx] = c;
      updated = true;
      break;
    }
  }
  if (!updated) {
    _from[v].push_back(u);
    _from_cost[v].push_back(c);
    _n_arc++;
  }
  return ;
};

void SparseGraph::CreateFromEdges(std::vector<long> sources, 
    std::vector<long> targets, std::vector<std::vector<double>> costs)
{
  _to.clear();
  _to_cost.clear();
  _from.clear();
  _from_cost.clear();
  long max_id = 0;
  for (size_t i = 0; i < sources.size(); i++){
    if (sources[i] > max_id) { max_id = sources[i]; }
    if (targets[i] > max_id) { max_id = targets[i]; }
  }
  _to.resize(max_id+1);
  _to_cost.resize(max_id+1);
  _from.resize(max_id+1);
  _from_cost.resize(max_id+1);
  for (size_t i = 0; i < sources.size(); i++) {
    _to[sources[i]].push_back(targets[i]);
    _to_cost[sources[i]].push_back(costs[i]);
    _from[targets[i]].push_back(sources[i]);
    _from_cost[targets[i]].push_back(costs[i]);
    _to[targets[i]].push_back(sources[i]);
    _to_cost[targets[i]].push_back(costs[i]);
    _from[sources[i]].push_back(targets[i]);
    _from_cost[sources[i]].push_back(costs[i]);
  }
  return ;
};

void SparseGraph::CreateFromArcs(std::vector<long> sources, 
    std::vector<long> targets, std::vector<std::vector<double>> costs)
{
  _to.clear();
  _to_cost.clear();
  _from.clear();
  _from_cost.clear();
  long max_id = 0;
  for (size_t i = 0; i < sources.size(); i++){
    if (sources[i] > max_id) { max_id = sources[i]; }
    if (targets[i] > max_id) { max_id = targets[i]; }
  }
  _to.resize(max_id+1);
  _to_cost.resize(max_id+1);
  _from.resize(max_id+1);
  _from_cost.resize(max_id+1);
  for (size_t i = 0; i < sources.size(); i++) {
    _to[sources[i]].push_back(targets[i]);
    _to_cost[sources[i]].push_back(costs[i]);
    _from[targets[i]].push_back(sources[i]);
    _from_cost[targets[i]].push_back(costs[i]);
  }
  return ;
};

void SparseGraph::ChangeCostDim(size_t new_cdim, double default_value) {
  for (size_t i = 0; i < _to.size(); i++) {
    for (size_t j = 0; j < _to[i].size(); j++) {
      _to_cost[i][j].resize(new_cdim, default_value);
    }
  }
  for (size_t i = 0; i < _from.size(); i++) {
    for (size_t j = 0; j < _from[i].size(); j++) {
      _from_cost[i][j].resize(new_cdim, default_value);
    }
  }
  _cdim = new_cdim;
  return ;
};

bool SparseGraph::SetArcCost(long u, long v, const std::vector<double>& new_cost) {
  if (new_cost.size() != _cdim){
    std::cout << "[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first." << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first.") ;
    return false;    
  }
  if ( (!HasVertex(u)) || (!HasVertex(v)) ) {
    std::cout << "[ERROR] SparseGraph::SetArcCost vertex does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost vertex does not exist!") ;
    return false;
  }
  if ( new_cost.size() != _cdim ) {
    std::cout << "[ERROR] SparseGraph::SetArcCost input vector size mismatch : " << new_cost.size() << " vs " << _cdim << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost input vector size mismatch!") ;
    return false;
  }

  bool found = false;
  for (int i = 0; i < _to[u].size(); i++){
    if (_to[u][i] == v) {
      _to_cost[u][i] = new_cost;
      found = true;
      break;
    }
  }
  if (!found) {
    std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
  }

  found = false;
  for (int i = 0; i < _from[v].size(); i++){
    if (_from[v][i] == u) {
      _from_cost[v][i] = new_cost;
      found = true;
      break;
    }
  }
  if (!found) {
    std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
    throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
  }
  return true;
};

std::string SparseGraph::ToStr() const {
  std::string out;
  out += "=== SparseGraph Begin ===\n |V| = " + std::to_string(_to.size()) + " outgoing edges \n";
  for (long v = 0; v < _to.size(); v++) {
    out += " -- " + std::to_string(v) + ":[";
    for (size_t idy = 0; idy < _to[v].size(); idy++){
      out += std::to_string(_to[v][idy]) + "(" + ToString(_to_cost[v][idy]) + "),";
    }
    out += "]\n";
  }
  out += " incoming edges \n";
  for (long v = 0; v < _to.size(); v++) {
    out += " -- " + std::to_string(v) + ":[";
    for (size_t idy = 0; idy < _from[v].size(); idy++){
      out += std::to_string(_from[v][idy]) + "(" + ToString(_from_cost[v][idy]) + "),";
    }
    out += "]\n";
  }
  out += "=== SparseGraph End ===";
  return out;
};

std::ostream& operator<<(std::ostream& os, const SparseGraph& c) {
  os << c.ToStr();
  return os;
};

// ############################################################
// ############################################################
// ############################################################

Grid2d::Grid2d() {
  SetKNeighbor(4); // by default
};

Grid2d::~Grid2d() {};

bool Grid2d::HasVertex(long v)
{
  if ( (_k2r(v) < _occu_grid_ptr->size()) && (_k2c(v) < _occu_grid_ptr->at(0).size()) ){
    return true;
  }
  return false;
};

bool Grid2d::HasArc(long v, long u) {
  // newly added on @2023-11
  // TODO, run some test.
  long r1 = _k2r(v);
  long c1 = _k2c(v);
  long r2 = _k2r(u);
  long c2 = _k2c(u);
  for (int i = 0; i < _act_r.size(); i++){
    bool b1 = (r2 == (r1+_act_r[i]));
    bool b2 = (c2 == (c1+_act_c[i]));
    if (b1 && b2) {return true;}
  }
  return false;
};

std::vector<long> Grid2d::GetSuccs(long v)
{
  std::vector<long> out;
  long r = _k2r(v);
  long c = _k2c(v);
  for (size_t idx = 0; idx < _kngh; idx++) {
    long nr = r+_act_r[idx];
    if (nr >= _occu_grid_ptr->size()) {continue;}
    long nc = c+_act_c[idx];
    if (nc >= _occu_grid_ptr->at(0).size()) {continue;}
    if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // obstacle
    out.push_back(_rc2k(nr,nc));
  }
  return out;
};

std::vector<long> Grid2d::GetPreds(long v)
{
  return GetSuccs(v);
};
  

CostVec Grid2d::GetCost(long u, long v)
{
  // v is the target vertex of arc (u,v)
  std::vector<double> out;
  long r = _k2r(v);
  long c = _k2c(v);
  long r2 = _k2r(v);
  long c2 = _k2c(v);
  if ((r != r2) && (c != c2)) {
    out.push_back(1.4*_cost_scale);
    return out;
  }else{
    out.push_back(1.0*_cost_scale);
    return out;
  }
};

std::vector< CostVec > Grid2d::GetSuccCosts(long u)
{
  std::vector<std::vector<double>> out;
  long r = _k2r(u);
  long c = _k2c(u);
  for (size_t idx = 0; idx < _kngh; idx++) {
    long nr = r+_act_r[idx];
    // if (nr >= _occu_grid_ptr->size()) {continue;}
    long nc = c+_act_c[idx];
    // if (nc >= _occu_grid_ptr->at(0).size()) {continue;}
    if (! IsWithinBorder(nr, nc)) {continue;};
    if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // obstacle
    std::vector<double> c;
    if (idx <= 3) { 
      c.push_back(1.0*_cost_scale);
    }
    else if (idx <= 7) {
      c.push_back(1.4*_cost_scale);
    }
    else { throw std::runtime_error( "[ERROR], Grid2d _kngh > 8, not supported!" ); }
    out.push_back(c);
  }
  return out;
};

std::vector< CostVec > Grid2d::GetPredCosts(long u) {
  return GetSuccCosts(u);
};

size_t Grid2d::NumVertex() {
  if (_occu_grid_ptr->size() == 0) {return 0;}
  return _occu_grid_ptr->size() * _occu_grid_ptr->at(0).size() ;
} ;

size_t Grid2d::NumArc() {
  std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
  return 0;
};

size_t Grid2d::NumEdge() {
  std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
  return 0;
}

size_t Grid2d::CostDim() {
  return 1;
};

std::vector<long> Grid2d::AllVertex() 
{
  std::cout << "[ERROR], Grid2d::AllVertex not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], Grid2d::AllVertex not implemented, TODO." );

  std::vector<long> out;
  return out;
};

////////////

void Grid2d::SetOccuGridPtr(std::vector< std::vector<double> >* in)
{
  _occu_grid_ptr = in;
  return ;
};

std::vector< std::vector<double> >* Grid2d::GetOccuGridPtr()
{
  return _occu_grid_ptr;
};

void Grid2d::SetOccuGridObject(std::vector< std::vector<double> >& in)
{
  _mat_from_py = in; // make a local copy. To avoid pybind issue.
  SetOccuGridPtr(&_mat_from_py);
};

bool Grid2d::IsWithinBorder(long nr, long nc) {
    if (nr >= _occu_grid_ptr->size() || nr < 0) {return false;}
    if (nc >= _occu_grid_ptr->at(0).size() || nc < 0) {return false;}
    return true;
};

bool Grid2d::SetKNeighbor(int kngh) {
  if (kngh == 4 || kngh == 8) {
    _kngh = kngh;
    if (_kngh == 4) {
      _act_r = std::vector<long>({0,0,-1,1});
      _act_c = std::vector<long>({-1,1,0,0});
    }else if (_kngh == 8) {
      _act_r = std::vector<long>({ 0, 0,-1, 1, -1,-1, 1, 1});
      _act_c = std::vector<long>({-1, 1, 0, 0, -1, 1,-1, 1});
    }
    return true;
  }
  return false;
};

void Grid2d::SetCostScaleFactor(const double in) {
  _cost_scale = in;
};

long Grid2d::_rc2k(const long r, const long c) const 
{
  return r * _occu_grid_ptr->at(0).size() + c;
};

long Grid2d::_k2r(const long k) const
{
  return long(k / _occu_grid_ptr->at(0).size());
};

long Grid2d::_k2c(const long k) const 
{
  return k % _occu_grid_ptr->at(0).size();
};

/////////////////////////////////////////////////////////////////

HybridGraph2d::HybridGraph2d() {};

HybridGraph2d::~HybridGraph2d() {};

bool HybridGraph2d::HasVertex(long v) {
  if (v < 0) {return false;}
  if (v < _nid_ends.back()) {return true;}
  return false;
};

bool HybridGraph2d::HasArc(long v, long u) {
  if (!HasVertex(v)) {return false;}
  if (!HasVertex(u)) {return false;}
  if (_find_subgraph(v) == _find_subgraph(u)) {return true;}
  for (int i = 0; i < _ig_arc_srcs.size(); i++){
    if (v == _ig_arc_srcs[i] && u == _ig_arc_tgts[i]){return true;}
  }
  return false;
};

std::vector<long> HybridGraph2d::GetSuccs(long v) {
  // for (int i = 0; i < _nid_starts.size(); i++){
  //   std::cout << " a, b = " << _nid_starts[i] << ", " << _nid_ends[i] << " ,,, " << _index_map[i] << std::endl;
  // }
  std::vector<long> out;
  int idx = _find_subgraph(v);
  if (idx < 0) {return out;}
  long nid = _g2l_nid(v);
  int kk = _index_map[idx];
  std::cout << " idx = " << idx << " nid = " << nid << " kk = " << kk << std::endl;
  if (kk >= 0){
    out = _grids[kk]->GetSuccs(nid);
  }else{
    std::cout << " -kk-1 = " << -kk-1 << std::endl;
    out = _roadmaps[-kk-1]->GetSuccs(nid);
  }
  for (int j = 0; j < out.size(); j++){
    out[j] += _nid_starts[idx];
  }
  //
  for (int j = 0; j < _ig_arc_srcs.size(); j++){
    if (_ig_arc_srcs[j] == v){
      out.push_back(_ig_arc_tgts[j]);
    }
  }
  return out;
};

std::vector<long> HybridGraph2d::GetPreds(long v) {
  std::vector<long> out;
  int idx = _find_subgraph(v);
  if (idx < 0) {return out;}
  long nid = _g2l_nid(v);
  int kk = _index_map[idx];
  if (kk >= 0){
    out = _grids[kk]->GetPreds(nid);
  }else{
    out = _roadmaps[-kk-1]->GetPreds(nid);
  }
  for (int j = 0; j < out.size(); j++){
    out[j] += _nid_starts[idx];
  }
  for (int j = 0; j < _ig_arc_tgts.size(); j++){
    if (_ig_arc_tgts[j] == v){
      out.push_back(_ig_arc_srcs[j]);
    }
  }
  return out;
};

CostVec HybridGraph2d::GetCost(long u, long v) {
  std::vector<double> out;
  int idx = _find_subgraph(u);
  int idy = _find_subgraph(v);
  if (idx < 0 || idy < 0) {return out;}
  if (idx == idy){
    long uu = _g2l_nid(u);
    long vv = _g2l_nid(v);
    // std::cout << " uu = " << uu << ", vv = " << vv << std::endl;
    int kk = _index_map[idx];
    if (kk >= 0){
      return _grids[kk]->GetCost(uu,vv);
    }else{
      return _roadmaps[-kk-1]->GetCost(uu,vv);
    }
  }
  for (int j = 0; j < _ig_arc_srcs.size(); j++){
    // std::cout << " _ig_arc = " << _ig_arc_srcs[j] << ", " << _ig_arc_tgts[j] << std::endl;
    // std::cout << " u = " << u << ", v = " << v << std::endl;
    if (_ig_arc_srcs[j] == u && _ig_arc_tgts[j] == v){
      return _ig_costs[j];
    }
  }
  return out;
};

std::vector< CostVec > HybridGraph2d::GetSuccCosts(long u) {
  std::vector< CostVec > out;
  int idx = _find_subgraph(u);
  if (idx < 0) {return out;}
  long nid = _g2l_nid(u);
  int kk = _index_map[idx];
  if (kk >= 0){
    out = _grids[kk]->GetSuccCosts(nid);
  }else{
    out = _roadmaps[-kk-1]->GetSuccCosts(nid);
  }
  for (int j = 0; j < _ig_arc_srcs.size(); j++){
    if (_ig_arc_srcs[j] == u){
      out.push_back(_ig_costs[j]);
    }
  }
  return out;
};

std::vector< CostVec > HybridGraph2d::GetPredCosts(long u) {
  std::vector< CostVec > out;
  int idx = _find_subgraph(u);
  if (idx < 0) {return out;}
  long nid = _g2l_nid(u);
  int kk = _index_map[idx];
  if (kk >= 0){
    out = _grids[kk]->GetPredCosts(nid);
  }else{
    out = _roadmaps[-kk-1]->GetPredCosts(nid);
  }
  for (int j = 0; j < _ig_arc_tgts.size(); j++){
    if (_ig_arc_tgts[j] == u){
      out.push_back(_ig_costs[j]);
    }
  }
  return out;
};

size_t HybridGraph2d::NumVertex() {
  if (_nid_ends.size() == 0) {
    return 0;
  }
  return _nid_ends.back();
};

size_t HybridGraph2d::NumArc() {
  size_t out = 0;
  for (auto p : _grids){
    out += p->NumArc();
  }
  for (auto p : _roadmaps){
    out += p->NumArc();
  }
  out += _ig_arc_srcs.size();
  return out;
};

size_t HybridGraph2d::NumEdge() {
  size_t n_arcs = NumArc();
  if (n_arcs % 2 != 0) {
    std::cout << "[ERROR] HybridGraph2d::NumEdge is not an integer but a fraction" << std::endl;
    throw std::runtime_error("[ERROR] HybridGraph2d::NumEdge is not an integer but a fraction");
  }
  return size_t(n_arcs / 2);
};

size_t HybridGraph2d::CostDim() {
  // assume all sub-grids and sub-roadmaps have the same cost dim.
  for (auto p : _grids){
    return p->CostDim();
  }
  for (auto p : _roadmaps){
    return p->CostDim();
  }
  for (auto c : _ig_costs){
    return c.size();
  }
  return 0;
};

std::vector<long> HybridGraph2d::AllVertex() {
  std::cout << "[ERROR], HybridGraph2d::AllVertex not implemented, TODO." << std::endl;
  throw std::runtime_error( "[ERROR], HybridGraph2d::AllVertex not implemented, TODO." );

  std::vector<long> out;
  return out;
};

void HybridGraph2d::AddGrid2d(Grid2d* g) {
  _grids.push_back(g);
  if (_nid_starts.size() == 0) {
    _nid_starts.push_back(0);
    _nid_ends.push_back(g->NumVertex());
    _index_map.push_back(0);
    return ;
  }
  _nid_starts.push_back(_nid_ends.back());
  _nid_ends.push_back(_nid_starts.back()+g->NumVertex());
  _index_map.push_back(_grids.size()-1);
  return ;
};

void HybridGraph2d::AddSparseGraph(SparseGraph* g) {
  _roadmaps.push_back(g);
  if (_nid_starts.size() == 0) {
    _nid_starts.push_back(0);
    _nid_ends.push_back(g->NumVertex());
    _index_map.push_back(-1);
    return ;
  } 
  _nid_starts.push_back(_nid_ends.back());
  _nid_ends.push_back(_nid_starts.back()+g->NumVertex());
  _index_map.push_back(-_roadmaps.size()); // _roadmaps is already increased by one due to the push_back.
  return ;
};

void HybridGraph2d::AddExtraEdge(long u, long v, CostVec c) {
  _ig_arc_srcs.push_back(u);
  _ig_arc_tgts.push_back(v);
  _ig_costs.push_back(c);
  return ;
};

int HybridGraph2d::_find_subgraph(long v) {
  for (int i = 0; i < _nid_ends.size(); i++){
    if (v >= _nid_starts[i] && v < _nid_ends[i]){
      return i;
    }
  }
  return -1;
};

long HybridGraph2d::_g2l_nid(long v) {
  for (int i = 0; i < _nid_ends.size(); i++){
    if (v >= _nid_starts[i] && v < _nid_ends[i]){
      return v - _nid_starts[i];
    }
  }
  return -1;
};


} // end namespace zr

