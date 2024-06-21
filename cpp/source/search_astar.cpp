
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_astar.hpp"
#include <set>
#include <limits>

namespace raplab{

Astar::Astar() {
  _class_name = "Astar";
};

Astar::~Astar() {

};

double Astar::_heuristic(long v) {
  return 0;
};

void Astar::_add_open(long u, double dist_u) {
  auto f = dist_u + _heuristic(u);
  auto temp_pair = std::make_pair(f,u);
  _open.insert(temp_pair); // re-insert
};


////////////////////////////


AstarGrid2d::AstarGrid2d() {
  _class_name = "AstarGrid2d";
};

AstarGrid2d::~AstarGrid2d() {

};

double AstarGrid2d::_heuristic(long v) {
  auto gg = dynamic_cast<Grid2d*>( _graph );
  auto r = gg->_k2r(v);
  auto c = gg->_k2c(v);
  return abs(r - _vd_r) + abs(c - _vd_c);
};

void AstarGrid2d::_init_more() {
  auto gg = dynamic_cast<Grid2d*>( _graph );
  _vd_r = gg->_k2r(_vg);
  _vd_c = gg->_k2c(_vg);
};

} // end namespace zr
