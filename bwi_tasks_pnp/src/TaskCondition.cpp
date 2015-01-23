#include "TaskCondition.h"
#include "bwi_kr_execution/CurrentStateQuery.h"

#include <iostream>
#include <fstream>
#include <ros/package.h>

#include <dirent.h>

using namespace std;

double TaskCondition::rew;

TaskCondition::TaskCondition(Client& client, ros::ServiceClient& currentClient) : client(client), currentClient(currentClient) {}

bool TaskCondition::evaluateAtomicExternalCondition (const std::string& atom ) {
    if (atom.compare(0, 10, "IsDoorOpen") == 0) {
        
        currentClient.waitForExistence();

        bwi_kr_execution::AspFluent openFluent;
        openFluent.name = "open";
        openFluent.timeStep = 0;
        openFluent.variables.push_back(atom.substr(atom.find_first_not_of("- ", 10), atom.length()));

        bwi_kr_execution::AspRule rule;
        rule.head.push_back(openFluent);

        bwi_kr_execution::CurrentStateQuery csq;
        csq.request.query.push_back(rule);

        currentClient.call(csq);

        return csq.response.answer.satisfied;
    }
    string taskTypeName = ros::package::getPath("bwi_tasks_pnp")+"/task_types/"+ atom + ".txt";
    ofstream taskTypeFile(taskTypeName.c_str());
    
    if ((! taskTypeFile) || (! taskTypeFile.is_open())) {
        ROS_INFO_STREAM("No task type with name " << atom);
        return false;
    }

    ros::Duration(1).sleep();

    string taskPath = ros::package::getPath("bwi_tasks_pnp")+"/tasks/";
    
    DIR *dp;
    dp = opendir(taskPath.c_str());
    struct dirent *dirp;

    while (((dirp = readdir(dp)) != NULL) && ((string(dirp->d_name).find(atom.c_str())) == std::string::npos));

    closedir(dp);

    if (dirp == NULL) {
        return false;
    } else {

        string taskName = taskPath + string(dirp->d_name);
        
        ifstream taskFile(taskName.c_str());
        if (! taskFile) {
            if (! taskFile.is_open())
                remove(taskName.c_str());
            return false;
        }

        //ROS_INFO_STREAM(string(dirp->d_name));

        taskTypeFile << taskFile.rdbuf();
        taskFile.close();
        taskTypeFile.close();
        remove(taskName.c_str());
        return true;

    }
}

double TaskCondition::reward() {
    double r = rew;
    rew = 0.;
    //ROS_INFO("reward %F", r);
    return r;
}

void TaskCondition::setReward(double reward) {
    rew = reward;
}
