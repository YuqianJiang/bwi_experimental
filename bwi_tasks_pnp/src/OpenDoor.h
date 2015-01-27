#ifndef OPENDOOR_H
#define OPENDOOR_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

#include <kractions/OpenDoor.h>

using namespace std;

class OpenDoor : public PetriNetPlans::PnpAction {
public:
    OpenDoor(const string& door);	
    virtual void executeStep();
    virtual bool finished();

private:
    string door;
    bool sent;
    double initial_time;
    bwi_krexec::OpenDoor* open;
};


#endif // OPENDOOR_H
