#include "OpenSimulatedDoor.h"
#include "TaskCondition.h"

OpenSimulatedDoor::OpenSimulatedDoor(const string& door) : door(door), sent(false) {}

void OpenSimulatedDoor::executeStep() {

    if (!sent) {
        initial_time = ros::Time::now().toSec();
        open = new bwi_krexec::OpenSimulatedDoor(door);
        sent = true;
    }

    if (!open->hasFinished()) {
        open->run();
    }
        

}

bool OpenSimulatedDoor::finished() {

    if ((sent) && (open->hasFinished())) {

        if (open->hasFailed()) {
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