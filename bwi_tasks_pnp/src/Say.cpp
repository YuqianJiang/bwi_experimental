#include "Say.h"
#include <sound_play/sound_play.h>
#include "TaskCondition.h"

Say::Say(const string& sentence, ros::Publisher& soundPublisher) : sentence(sentence), soundPublisher(soundPublisher), sent(false) {}

void Say::executeStep() {
    if (!sent) {
        //speak
        sound_play::SoundRequest sound_req;
        sound_req.sound = sound_play::SoundRequest::SAY;
        sound_req.command = sound_play::SoundRequest::PLAY_ONCE;

        sound_req.arg = sentence;
        soundPublisher.publish(sound_req);

        sent = true;
    }
}

bool Say::finished() {

    if (sent) {
        ROS_INFO("Succeeded!");
        double reward = initial_time - ros::Time::now().toSec();
        ROS_INFO("Reward is %f",reward);
        TaskCondition::setReward(reward);
        return true;
    }
    else
        return false;
}