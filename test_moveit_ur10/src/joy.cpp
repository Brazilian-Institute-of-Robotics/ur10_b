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

#include <math.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

void chatterCallback(const sensor_msgs::JoyConstPtr& msg);

double X, Y, Z;
static bool cout = false;
ros::Publisher Axis;
double A;
int UpDown = 0;
int LeftRight = 0;
int initPose = 0;
    
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
     moveit_visual_tools::MoveItVisualTools visual_tools(move_group.getPlanningFrame());
     visual_tools.trigger();

     Axis = nh.advertise<geometry_msgs::Pose>("goal", 1);
     ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, chatterCallback);
     ros::Rate rate(10);
     while(ros::ok()) {
          if(cout) {
               tf::Pose t_goal;
               t_goal.setOrigin( tf::Vector3(X, Y, Z) );
               t_goal.setRotation( tf::Quaternion(-0.5832, 0.6325, 0.41627, 0.41628));

               geometry_msgs::Pose goal;

               tf::poseTFToMsg(t_goal, goal);

               //specify tolerances
               move_group.setGoalPositionTolerance(0.01);
               move_group.setGoalOrientationTolerance(0.01);

               move_group.setPoseTarget(goal); 
               //          /----------------------------------------------------------
                    

            //execute the trajectory
                         move_group.move();
               
               cout = false;
          }
          ros::spinOnce();     
          rate.sleep();
     }
}

void chatterCallback(const sensor_msgs::JoyConstPtr& msg) {
     cout = true;
     geometry_msgs::Pose pose;
     //pose.position.x = msg->axes[0]*0.8;
     //pose.position.y = msg->axes[1]*0.8;
     //pose.position.z = msg->axes[4]*0.8;
     UpDown = msg->axes[7]; // +1/-1
     LeftRight = msg->axes[6]; // +1/-1
     initPose = msg->buttons[1];

     if(initPose == 1){ // Up
          X = 0.5;
          Y = 0.5;
          Z = 0.4;
     }

     if(UpDown == 1){ // Up
          X = 0.5;
          Y = 0.5;
          Z = Z + 0.2;
          if(Z > 0.9) Z = 0.9; 
          
     }
     else if(UpDown == -1){ // Down
          X = X;
          Y = Y;
          Z = Z - 0.15;
          if(Z < 0.2) Z = 0.2; 
     }
     else if(LeftRight == 1){ // Left Y > X
          if (X > 0){
               X = X - 0.1;
               Y = Y + 0.1;
               Z = Z;
               if (Y > 0.8) Y = 0.8;
          }
          else if(X < 0){
          X = X - 0.1;
          Y = Y - 0.1;
          Z = Z;
               if (X < -0.8) X = -0.8;
          }
          else if(Y < 0){
          X = X + 0.1;
          Y = Y - 0.1;
          Z = Z;
               if (Y < -0.8) Y = -0.8;
          }
     }
     else if(LeftRight == -1){ // Right X > Y
          if (Y > 0){
               X = X + 0.1;
               Y = Y - 0.1;
               Z = Z;
               if (X > 0.9) X = 0.8;
          }
          else if(X < 0){
          X = X - 0.1;
          Y = Y + 0.1;
          Z = Z;
               if (X < -0.9) X = -0.8;
          }
     }

     pose.orientation.x = 0;
     pose.orientation.y = 0;
     pose.orientation.z = 0;
     pose.orientation.w = 1.0;
     //X = msg->position.x;
     //Y = msg->position.y;
     //Z = msg->position.z;
     Axis.publish(pose);
     
}