#include <ros/ros.h>
#include <pal_locomotion_msgs/ActionWithParameters.h>
#include <pal_locomotion_msgs/PushActions.h>
#include <property_bag/property_bag.h>
#include <pal_locomotion/step.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <property_bag/serialization/property_bag_boost_serialization.h>

using namespace pal_locomotion;

int main(int argc, char **argv)
{ 
  // Set up ROS.
  ros::init(argc, argv, "move_com_action_start");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<pal_locomotion_msgs::PushActions>(
      "/biped_walking_dcm_controller/push_actions");

  pal_locomotion_msgs::PushActions push_actions_request;

  pal_locomotion_msgs::ActionWithParameters new_action;
  new_action.action_type = "pal_locomotion::COMAction";
  property_bag::PropertyBag parameters;
  parameters.addProperty("target_com_position", eVector3(0.01, 0.05, 0.0));
  parameters.addProperty("duration", 2.0);

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << parameters;
  new_action.action_parameters = ss.str();
  push_actions_request.request.actions.push_back(new_action);

  if (client.call(push_actions_request))
  {
    ROS_INFO("Succesfully called service");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
}
