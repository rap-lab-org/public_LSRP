
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_BASIC_UNIONFIND_H_
#define ZHONGQIANGREN_BASIC_UNIONFIND_H_

// #include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

namespace raplab{

/**
 * @brief Find operation in Union-find data structure. 
 *  Assume a is in dic. No safety check. You should do that outside.
 */
template<typename DataType>
DataType UFFind(std::unordered_map<DataType,DataType>* dic, DataType a) ;
/**
 * @brief Union the sets that contains element a and b.
 *  Returned value int indicates the running status of this func.
 */
template<typename DataType>
int UFUnion(std::unordered_map<DataType,DataType>* dic, const DataType& a, const DataType& b) ;

} // end rzq


#endif  // ZHONGQIANGREN_BASIC_UNIONFIND_H_

