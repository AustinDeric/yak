#include "nbv_planner/exploration_controller_node.hpp"

const std::string link_ = "camera_link";
static const std::string PLANNING_GROUP = "eef_group";

Explorer::Explorer(ros::NodeHandle &nh) {
//  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//  move_group_ = move_group;
  nbv_client_ = nh.serviceClient<nbv_planner::GetNBV>("/get_nbv");
  node_handle_ = nh;
//  exploration_server_ = nh.advertiseService<std_srvs::Empty>("do_exploration", &Explorer::MoveToNBVs, this);

}

bool Explorer::MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group)
{
  /*
   * While "finished" criteria is False:
   * - Get next best view given current information
   * - Move to view
   */
  ros::service::waitForService("/get_nbv");
  while (true)
  {
    ROS_INFO_STREAM("====STARTING EXPLORATION CONTROLLER=====");
    // Make a new call to the GetNBV service to get a list of potentially-good poses to move towards.
    nbv_planner::GetNBV srv;
    nbv_client_.call(srv);
    if (srv.response.exploration_done) {
      ROS_INFO("Exploration reasonably completed");
      break;
    }
    geometry_msgs::PoseArray move_targets = srv.response.bestViewPose;
    // Start with the most highly ranked pose
    int currentPoseIndex = 0;

    // Get the position of the TSDF volume to set constraint on camera orientation.
    tfListener_.waitForTransform("base_link", "volume_pose", ros::Time::now(), ros::Duration(0.5));

    // Test poses until we can find one that's reachable in constrained space.
    bool success = false;
    while(!success)
    {
      ROS_INFO_STREAM("Pose index: " << currentPoseIndex);
      if (currentPoseIndex >= move_targets.poses.size())
      {
        ROS_ERROR("Couldn't reach any of the provided poses in free space!");
        break;
      }

      // Get next best pose and try to plan a path to it in free space
      geometry_msgs::PoseStamped target_pose;
      geometry_msgs::PoseStamped input_pose;
      input_pose.header = move_targets.header;
      input_pose.pose = move_targets.poses[currentPoseIndex];
      ROS_INFO_STREAM("FRAME ID " << move_targets.header.frame_id);
      tfListener_.transformPose("base_link", input_pose, target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      move_group.setPoseTarget(target_pose);
      ROS_INFO_STREAM("Trying next best pose: " << target_pose);

      if (move_group.plan(my_plan))
      {
        // If it's reachable in free space, try to move to it in constrained space.
        ROS_INFO("Found a path in free space!");
        move_group.setPoseTarget(target_pose);
        if (!move_group.move())
        {
          // If not reachable in constrained space, try the next pose in free space.
          ROS_INFO("Couldn't move to this pose in constrained space");
          currentPoseIndex++;
        }
        else
        {
          // If reachable in constrained space, move to next step of NBV
          ROS_INFO("Moved to the pose in constrained space!");
          success = true;
        }
      }
      else
      {
        // If not reachable in free space, try the next pose
        ROS_ERROR("Couldn't compute a free-space trajectory");
        currentPoseIndex++;
      }
    }
  }
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "exploration_controller_node");
    ros::NodeHandle nh;

    //todo replace with arg from launch file
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlanningTime(60.0);

    Explorer explorer(nh);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    explorer.MoveToNBVs(move_group);

    return 0;
}
