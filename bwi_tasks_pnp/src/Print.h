#ifndef PRINT_H
#define PRINT_H

#include "ros/ros.h"
#include <pnp/pnp_action.h>

#include <segbot_gui/QuestionDialogRequest.h>
#include "segbot_gui/QuestionDialog.h"

using namespace std;

class Print : public PetriNetPlans::PnpAction {
public:
    Print(const string& sentence);	
    virtual void executeStep();
    virtual bool finished();

private:
    string sentence;
    bool done;
    double initial_time;
    segbot_gui::QuestionDialog req;
};


#endif // PRINT_H
