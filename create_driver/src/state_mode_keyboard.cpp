#include <ros/ros.h>
#include "States.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <create_driver/ChangeStateModeAction.h>

using namespace create_driver;

typedef actionlib::SimpleActionClient<ChangeStateModeAction> Client;

class StateModeKeyboard
{
protected:
  ChangeStateModeGoal goal;
  int mutex;

public:
  StateModeKeyboard() : 
    ac_("team6", true)
  {
    // Set defaults
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Server started, awaiting input.");
  }

  void run()
  {
    goal.state_mode_id = FOLLOW_OWNER;
    
    do {
      mutex = 1;
      ROS_INFO("Sending goal: State Mode: %d", goal.state_mode_id);
      ac_.sendGoal(goal,
      boost::bind(&StateModeKeyboard::done, this, _1, _2),
      Client::SimpleActiveCallback(),
      Client::SimpleFeedbackCallback());

      // Wait for previous action to complete (makes menus prettier)
      while (mutex);

      // Prompt for New Mode
      std::cout << "Select State Mode:\n";
      std::cout << "0 - Follow Owner\n";
      std::cout << "1 - Find Ball\n";
      std::cout << "2 - Fetch Ball\n";
      std::cout << ":";
      std::cin >> goal.state_mode_id;
        
    } while(ros::ok());
  }

  void done(const actionlib::SimpleClientGoalState& state,
              const ChangeStateModeResultConstPtr& result)
  {
    ROS_INFO("Goal finished in state [%s]", state.toString().c_str());
    mutex = 0;
  }

private:
  Client ac_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_mode_keyboard");
  StateModeKeyboard my_node;
  my_node.run();
  ros::spin();
  return 0;
}
