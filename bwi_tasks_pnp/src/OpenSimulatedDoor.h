#ifndef OPENSIMULATEDDOOR_H
#define OPENSIMULATEDDOOR_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

#include <kractions/OpenSimulatedDoor.h>

using namespace std;

class OpenSimulatedDoor : public PetriNetPlans::PnpAction {
public:
    OpenSimulatedDoor(const string& door);	
    virtual void executeStep();
    virtual bool finished();

private:
    string door;
    bool sent;
    double initial_time;
    bwi_krexec::OpenSimulatedDoor* open;
};


#endif // OPENSIMULATEDDOOR_H
