#include "Print.h"

#include "TaskCondition.h"

Print::Print(const string& sentence, ros::ServiceClient& printClient) : sentence(sentence), printClient(printClient), done(false) {}

void Print::executeStep() {
	req.request.type = segbot_gui::QuestionDialogRequest::DISPLAY;
	req.request.message = sentence;

	printClient.call(req);

    done = true;
}

bool Print::finished() {

    if (done) {
        ROS_INFO("Succeeded!");
        double reward = initial_time - ros::Time::now().toSec();
        ROS_INFO("Reward is %f",reward);
        TaskCondition::setReward(reward);
        return true;
    }
    else {
        return false;
    }
}