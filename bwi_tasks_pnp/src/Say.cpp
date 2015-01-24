#include "Say.h"
#include <sound_play/sound_play.h>
#include "TaskCondition.h"

Say::Say(const string& sentence) : sentence(sentence), sent(false), pub_set(false) {}
ros::Publisher Say::soundPublisher;

void Say::executeStep() {
    if (!sent) {

        ros::NodeHandle n;
        if (!pub_set) {
            soundPublisher = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
            pub_set = true;
        }
        
        //speak

        if (soundPublisher.getNumSubscribers() == 0) return; 
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
    else {
        return false;
    }
}