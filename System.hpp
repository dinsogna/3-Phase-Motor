//
//  System.hpp
//  CoupledObject
//
//  Created by Andrew Chen on 8/24/20.
//  Copyright © 2020 Andrew Chen. All rights reserved.
//

#ifndef System_hpp
#define System_hpp
#include "constants.h"
#include <vector>

class System{
public:
    System(double init1, double init2){
        state_type init(N);
        init<<init1, init2;
        values.push_back(init);
    }
    virtual state_type calculate(const state_type X, const double tor)=0;
    virtual state_type rk4_step(state_type state, double dt, double &tor)=0;
    void addState(state_type x){values.push_back(x);}
    state_type getState(int x) {return values[x];}
private:
    std::vector<state_type> values;
};

#endif /* System_hpp */
