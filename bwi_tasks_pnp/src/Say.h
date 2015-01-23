#ifndef SAY_H
#define SAY_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

using namespace std;

class Say : public PetriNetPlans::PnpAction {
public:
    Say(const string& sentence, ros::Publisher& soundPublisher);	
    virtual void executeStep();
    virtual bool finished();

private:
    string sentence;
    ros::Publisher& soundPublisher;
    bool sent;
    double initial_time;
};


#endif // SAY_H