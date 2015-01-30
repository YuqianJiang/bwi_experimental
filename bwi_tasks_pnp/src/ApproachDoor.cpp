#include "ApproachDoor.h"
#include "TaskCondition.h"

ApproachDoor::ApproachDoor(const string& door) : door(door), sent(false) {}

void ApproachDoor::executeStep() {

    if (!sent) {
        initial_time = ros::Time::now().toSec();
        approach = new bwi_krexec::ApproachDoor(door);
        sent = true;
    }

    if (!approach->hasFinished()) {
        approach->run();
    }
        

}

bool ApproachDoor::finished() {

    if ((sent) && (approach->hasFinished())) {

        if (approach->hasFailed()) {
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