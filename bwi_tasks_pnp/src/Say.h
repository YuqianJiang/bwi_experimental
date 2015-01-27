#ifndef SAY_H
#define SAY_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

using namespace std;

class Say : public PetriNetPlans::PnpAction {
public:
    Say(const string& sentence);	
    virtual void executeStep();
    virtual bool finished();

private:
    string sentence;
    static ros::Publisher soundPublisher;
    bool sent;
    static bool pub_set;
    double initial_time;
};


#endif // SAY_H