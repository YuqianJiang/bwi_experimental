#include "GoThrough.h"
#include "TaskCondition.h"

GoThrough::GoThrough(const string& door) : door(door), sent(false) {}

void GoThrough::executeStep() {

    if (!sent) {
        gothrough = new bwi_krexec::GoThrough(door);
        initial_time = ros::Time::now().toSec();
        sent = true;
    }

    if (!gothrough->hasFinished()) {
        gothrough->run();
    }
        

}

bool GoThrough::finished() {

    if ((sent) && (gothrough->hasFinished())) {

        if (gothrough->hasFailed()) {
            ROS_INFO("Failed!");
            this->fail();
        } 
        else {
            ROS_INFO("Succeeded!");
            double reward = initial_time - ros::Time::now().toSec();
            ROS_INFO("Reward is %f",reward);
            TaskCondition::setReward(reward);
        }
        return true;

    }
    else {
        return false;
    }
}