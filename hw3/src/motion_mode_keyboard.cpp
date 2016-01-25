#include <ros/ros.h>
#include "VideoModes.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <hw3/ChangeVideoModeAction.h>
using namespace hw3;

typedef actionlib::SimpleActionClient<ChangeVideoModeAction> Client;

class MotionModeKeyboard
{
protected:
  ChangeVideoModeGoal goal; 
  int mutex;

public:
  MotionModeKeyboard() : ac("motion_node", true)
  {
    // Set defaults
    ROS_INFO("Waiting for motion_node server to start.");
    ac.waitForServer();
    ROS_INFO("motion_mode server started, awaiting input.");
  }

  void run()
  {
    // Initial setup, no bounding boxes, standard video
    goal.video_mode_id = MODE_RAW_VIDEO;
    goal.visualization_mode_id = VISUALIZATION_OFF;
	
    do {
        mutex = 1;
        ROS_INFO("Sending goal: Video Mode: %d  Visulization Mode: %d", goal.video_mode_id, goal.visualization_mode_id);
        ac.sendGoal(goal,
			boost::bind(&MotionModeKeyboard::done, this, _1, _2),
			Client::SimpleActiveCallback(),
			Client::SimpleFeedbackCallback());

        // Wait for previous action to complete (makes menus prettier)
        while (mutex); 
	
		// Prompt for New Mode
        std::cout << "Select Video Mode:\n";
        std::cout << "0 - Raw Feed\n";
        std::cout << "1 - Farneback\n";
        std::cout << "2 - MOG2\n";
        std::cout << ":";
        std::cin >> goal.video_mode_id;
        
        std::cout << "Select Visualization Mode:\n";
        std::cout << "0 - None\n";
        std::cout << "1 - Bounding Box\n";
        std::cout << ":";
        std::cin >> goal.visualization_mode_id;

    } while(ros::ok());
  }

  void done(const actionlib::SimpleClientGoalState& state,
              const ChangeVideoModeResultConstPtr& result)
  {
    ROS_INFO("Goal finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Result for video mode change is %d", result->video_mode_changed);
    //ROS_INFO("Result for visualization mode change is %d", result->visualization_mode_changed);
    mutex = 0;
  }

private:
  Client ac;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_mode_keyboard");
  MotionModeKeyboard my_node;
  my_node.run();
  ros::spin();
  return 0;
}
