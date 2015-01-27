#ifndef APPROACHDOOR_H
#define APPROACHDOOR_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

#include <kractions/ApproachDoor.h>

using namespace std;

class ApproachDoor : public PetriNetPlans::PnpAction {
public:
    ApproachDoor(const string& door);	
    virtual void executeStep();
    virtual bool finished();

private:
    string door;
    bool sent;
    double initial_time;
    bwi_krexec::ApproachDoor* approach;
};


#endif // APPROACHDOOR_H
