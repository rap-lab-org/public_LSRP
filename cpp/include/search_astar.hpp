
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_BASIC_SEARCH_ASTAR_H_
#define ZHONGQIANGREN_BASIC_SEARCH_ASTAR_H_

#include "search_dijkstra.hpp"

#define DEBUG_ASTAR 0

namespace raplab{

/**
 * @brief
 * 
 * CAVEAT: The graph vertex ID should be within range [0,N], since std::vector is used as the underlying storage.
 * NOTE: This implementation assumes the entire graph is available.
 */
class Astar : public Dijkstra
{
public:
	/**
	 *
	 */
	Astar() ;
	/**
	 *
	 */
	virtual ~Astar() ;
	/**
	 *
	 */
	virtual void SetHeuWeight(double w) ;

protected:
	/**
	 * @brief A new function to be override in the derived classes.
	 */
	virtual double _heuristic(long v) ;
	/**
	 *
	 */
	virtual void _add_open(long u, double dist_u) ;

	double _wh = 1.0;
};

/**
 *
 */
class AstarGrid2d : public Astar
{
public:
	/**
	 *
	 */
	AstarGrid2d();
	/**
	 *
	 */
	~AstarGrid2d();
	
protected:
	/**
	 * @brief 
	 */
	virtual double _heuristic(long v) override ;
	/**
	 * @brief 
	 */
	virtual void _init_more() override;

	long _vd_r;
	long _vd_c;
};

} // end namespace zr

#endif  // ZHONGQIANGREN_BASIC_SEARCH_ASTAR_H_
