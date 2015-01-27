#ifndef GOTHROUGH_H
#define GOTHROUGH_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

#include <kractions/GoThrough.h>

using namespace std;

class GoThrough : public PetriNetPlans::PnpAction {
public:
    GoThrough(const string& door);	
    virtual void executeStep();
    virtual bool finished();

private:
    string door;
    bool sent;
    double initial_time;
    bwi_krexec::GoThrough* gothrough;
};


#endif // GOTHROUGH_H
