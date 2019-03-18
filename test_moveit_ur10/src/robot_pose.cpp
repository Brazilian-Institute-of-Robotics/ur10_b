#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Point.h>

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg);

double X, Y, Z;
double A, B, C, W;
static bool cout = false;
    
int main(int argc, char** argv){

     ros::init(argc, argv, "robot_pose");
     ros::NodeHandle nh;
     
     ros::AsyncSpinner spinner(0);
     spinner.start();
     
     std::vector<double> joint_positions;
     collision_detection::CollisionRequest collision_req;
     collision_detection::CollisionResult collision_res;
     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

     moveit_msgs::CollisionObject collision_object;

   
     static const std::string PLANNING_GROUP = "manipulator";
     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

     //define planning group
     geometry_msgs::PoseStamped now_frame;
    
     //get goal as parameters from launch file

    //retrive robot_model by its pointer 'RobotModelPtr'
     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
     robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

     planning_scene::PlanningScene planning_scene(robot_model);

     planning_scene.checkSelfCollision(collision_req, collision_res);
     ROS_INFO_STREAM("Test 1: Current state is " << (collision_res.collision ? "in" : "not in") << " self collision");
    
     robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

     collision_object.header.frame_id = move_group.getPlanningFrame();

     collision_object.id = "box1";

     shape_msgs::SolidPrimitive primitive;
     primitive.type = primitive.BOX;
     primitive.dimensions.resize(3);
     primitive.dimensions[0] = 1.9; //width
     primitive.dimensions[1] = 1.9; //large
     primitive.dimensions[2] = 0.001; //hight

     geometry_msgs::Pose box_pose;
     box_pose.orientation.w = 1.0;
     box_pose.position.x = 0.0;
     box_pose.position.y = 0.0;
     box_pose.position.z = 0.0;


     collision_object.primitives.push_back(primitive);
     collision_object.primitive_poses.push_back(box_pose);
     collision_object.operation = collision_object.ADD;

     std::vector<moveit_msgs::CollisionObject> collision_objects;
     collision_objects.push_back(collision_object);
    
     //add collision object into planning_scene
     planning_scene_interface.addCollisionObjects(collision_objects);
     ROS_INFO_NAMED("tutorial", "Add an object into the world");   

     //Visualization:
     namespace rvt = rviz_visual_tools;
     moveit_visual_tools::MoveItVisualTools visual_tools("wrist_3_link");
     visual_tools.trigger();
     ros::Subscriber sub = nh.subscribe("goal", 1, chatterCallback);
     ros::Rate rate(10);
     while(ros::ok()) {
          if(cout) {
               tf::Pose t_goal;
               t_goal.setOrigin( tf::Vector3(X, Y, Z) );
               t_goal.setRotation( tf::Quaternion(A, B, C, W));

               geometry_msgs::Pose goal;

               tf::poseTFToMsg(t_goal, goal);

               //specify tolerances
               move_group.setGoalPositionTolerance(0.01);
               move_group.setGoalOrientationTolerance(0.01);

               move_group.setPoseTarget(goal); 
               //          /----------------------------------------------------------
               //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
               //bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
               //ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCESS" : "FAILED");

                    //if(success == true){
                         sleep(2.5);

            //execute the trajectory
                         move_group.move();
                    //}
               cout = false;
          }
          ros::spinOnce();     
          rate.sleep();
     }
}

void chatterCallback(const geometry_msgs::Pose::ConstPtr& msg){
     cout = true;
     X = msg->position.x;
     Y = msg->position.y;
     Z = msg->position.z;
     A = msg->orientation.x;
     B = -msg->orientation.y;
     C = msg->orientation.z;
     W = msg->orientation.w;
}
