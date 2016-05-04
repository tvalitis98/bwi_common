
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>
#include <sstream>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

bool resumeCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

bool executing;

using namespace std;

int main(int argc, char**argv) {
  ros::init(argc, argv, "between_doors_interruptible");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  string locationA;
  privateNode.param<string>("a",locationA,"d3_414b1");

  string locationB;
  privateNode.param<string>("b",locationB,"d3_414b2");

  
  std::stringstream serviceName;
  serviceName << ros::this_node::getName() << "/resume";
  ros::ServiceServer resumeService = n.advertiseService(serviceName.str(), resumeCallback);

  Client client("/action_executor/execute_plan", true);
  client.waitForServer();
  
  executing = true;
  bool fromAtoB = true;
  
  ros::Rate r(10);

  while (ros::ok()) {
      
      // If the node was interrupted, wait for the 
      if(!executing) {
          r.sleep();
          continue;
      }

    string location = (fromAtoB)? locationB : locationA;

    fromAtoB = !fromAtoB;

    ROS_INFO_STREAM("going to " << location);

    bwi_kr_execution::ExecutePlanGoal goal;

    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";

    fluent.variables.push_back(location);

    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client.sendGoalAndWait(goal);

    if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Aborted");
      executing = false;
    } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      ROS_INFO("Preempted");
      executing = false;
    }

    else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Succeeded!");
    } else {
      ROS_INFO("Terminated");
      executing = false;
    }
  }

  return 0;
}

bool resumeCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    executing = true;
}