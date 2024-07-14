
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

namespace raplab {

/**
 * @brief Find operation in Union-find data structure. 
 *  Assume a is in dic. No safety check. You should do that outside.
 */
    template<typename DataType>
    DataType UFFind(std::unordered_map<DataType, DataType> *dic, DataType a) {
        // if (dic->find(a) == dic->end()) {return -1;}
        while ((*dic)[a] != a) {
            (*dic)[a] = (*dic)[(*dic)[a]]; // path compression
            a = (*dic)[a];
        }
        return a;
    };

/**
 * @brief Union the sets that contains element a and b.
 *  Returned value int indicates the running status of this func.
 */
    template<typename DataType>
    int UFUnion(std::unordered_map<DataType, DataType> *dic, const DataType &a, const DataType &b) {
        if (dic->find(a) == dic->end()) { return -1; }
        if (dic->find(b) == dic->end()) { return -2; }
        auto rooti = UFFind(dic, a);
        auto rootj = UFFind(dic, b);
        if (rooti == rootj) {
            return 0;
        } else {
            if (rooti > rootj) {
                (*dic)[rootj] = rooti;
            } else {
                (*dic)[rooti] = rootj;
            }
            return 1;
        }
    } // end rzq

}
#endif  // ZHONGQIANGREN_BASIC_UNIONFIND_H_
