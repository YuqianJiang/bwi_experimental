#include "bwi_kr_execution/ExecutePlanAction.h"
#include "bwi_kr_execution/CurrentStateQuery.h"
#include "segbot_gui/QuestionDialog.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <ros/package.h>

#include <pnp/pnp_executer.h>
#include <pnp/basic_plan/basic_plan.h>

#include "TaskInstantiator.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;
using namespace PetriNetPlans;

int main(int argc, char**argv) {
    ros::init(argc, argv, "pnp_tasks");
    ros::NodeHandle n;

    ros::NodeHandle privateNode("~");
    string planName;
    privateNode.param<string>("planName",planName,"single_task");

    ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
    Client client("/action_executor/execute_plan", true);
    ros::ServiceClient printClient = n.serviceClient<segbot_gui::QuestionDialog> ( "question_dialog" );
    
    currentClient.waitForExistence();
    client.waitForServer();
    printClient.waitForExistence();

    TaskInstantiator *inst = new TaskInstantiator(ros::package::getPath("bwi_tasks_pnp")+"/plans/", client, currentClient, printClient);
    
    PnpExecuter<PnpPlan> executor(inst);
    executor.setMainPlan(planName);
    
    while((ros::ok()) && (!executor.goalReached()))
        executor.execMainPlanStep();
	
    return 0;
}
