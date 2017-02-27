#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//List of goals excluding the first goal set before while loop
double goalList[4][7] = {
  {27.53, -6.76, 0, 0, 0, 0.68,  0.73},
  {27.13, 1.1,   0, 0, 0, 1,    -0.02},
  {13.02, 0.183, 0, 0, 0, -0.7,  0.71},
  {13.31, -3.4,  0, 0, 0, -0.66, 0.75}
};
int cnt = 0;
double epsilon = 0.3;

void switchGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal){
    //TODO: make this safe for out of bounds
    if(cnt > 3) return;
    ac.stopTrackingGoal();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x    = goalList[cnt][0];
    goal.target_pose.pose.position.y    = goalList[cnt][1];
    goal.target_pose.pose.position.z    = goalList[cnt][2];
    goal.target_pose.pose.orientation.x = goalList[cnt][3];
    goal.target_pose.pose.orientation.y = goalList[cnt][4];
    goal.target_pose.pose.orientation.z = goalList[cnt][5];
    goal.target_pose.pose.orientation.w = goalList[cnt][6];
    ROS_INFO("Sending Goal [%d]!", cnt+1);
    ac.sendGoal(goal);
    cnt++;
}

using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  tf::TransformListener listener;
  //ros::NodeHandle nh;
  //ros::Rate loop_rate(40);
  //tell the action client that we want to spin a thread by default
  double xPos,yPos;

  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }
  bool a = (ac.isServerConnected());
  cout << "Server conneceted: [%d]" << a << endl;

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 13.7;
  goal.target_pose.pose.position.y = -7.4;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending 1st goal");
  ac.sendGoal(goal);
  while(ros::ok()){
          //wait for the action server to come up

          ROS_INFO("%s", ac.getState().toString().c_str());
          //ac.waitForResult();
          listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
	        tf::StampedTransform transform;

          try
          	{
          		listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
          		xPos = transform.getOrigin().x();
          		yPos = transform.getOrigin().y();
          		ROS_INFO("Current position: (%f, %f, )\n", xPos,yPos);
          	}
          	catch(tf::TransformException &ex)
          	{
          		ROS_ERROR("%s", ex.what());
          		ros::Duration(1.0).sleep();
          }
          ROS_INFO("eps: [%f]",sqrt(pow((goal.target_pose.pose.position.x - xPos),2)
                + pow((goal.target_pose.pose.position.y - yPos),2)));
          // calculate dist. between goal and current pos.
          if(sqrt(pow((goal.target_pose.pose.position.x - xPos),2)
              + pow((goal.target_pose.pose.position.y - yPos),2)) < epsilon)
            {
                switchGoal(ac, goal);
                ROS_INFO("SWITCHING!");
            }


          /*if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("%s", ac.getState().toString().c_str());
            ac.stopTrackingGoal();
          }*/
              //ROS_INFO("Failed to reach 1st goal!");


  }

  ros::spin();
  return 0;
}
