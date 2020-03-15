#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h> 

//OSQP includes
#include "osqp.h"
#include "workspace_15_2.h" 
#include "interface_osqp_ros_mpc.h"
#include "matrices_osqp_traj_15_2.h"


//Iris Parameters
double iris_KT = 0.000015670;
double mass = 1.37;
double g = 9.81;

//Iris Variables
double iris_state[6];
double iris_state_vel[6];
double iris_state_ref_command[4];
double rotor_vel[4];
double n_filter = 4;
double iris_zpp = 0;
double empuxo = 0;
double iris_zpp_points[4];
double sum_zpp = 0;
double old_z_vel = 0;

//Controller Variables
double control_action[2];
double control_state_array[6];
double xy_ref[2];
double sin_phi, tg_theta_den, tg_theta_num;

//Aux Variables
int count_tick = 0;
int count_ref = 0;
int sample_time = 10;
double time_10m = 0;
double x_ini = 0, y_ini = 0;
int flag_ini = 0;

//ROS Variables
ros::Publisher topic; 
geometry_msgs::Pose ref_command;


//Topic names
std::string name_topic_state;
std::string name_topic_ref;


//Functions Prototype
void chatterCallback(const sensor_msgs::ImuConstPtr& current_state);
void publish_setpoint_ref(const ros::Publisher& topic_pub);
void update_iris_state(const sensor_msgs::ImuConstPtr& current_state);
void calculate_control();
void state_array_mpc();
void control_action_to_angular_ref();
void update_trajetory();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "iris_trajectory");
  ros::NodeHandle n;//("~");

  if (ros::param::get(ros::this_node::getName() + "/name_topic_state", name_topic_state)){
  	std::cout << name_topic_state << "\n\n";
  }
  else{
  	name_topic_state = "/gazebo_client/iris_state";
  	std::cout << "Nao tem\n\n";
  }

  if (ros::param::get(ros::this_node::getName() + "/name_topic_ref", name_topic_ref)){
  	std::cout << name_topic_ref << "\n\n";
  }
  else{
  	name_topic_ref= "/gazebo_client/iris_ref";
  	std::cout << "Nao tem\n\n";
  }

  std::cout << ros::this_node::getName() << "\n\n";
  //std::string hue_test;
  //hue_test = "sasasa" std::+ "sasas";
  //std::cout << hue_test <<"\n\n";

  /*
  if (n.getParam("name_topic_state", name_topic_state))
  //if (ros::param::get("name_topic_state", name_topic_state))	
	std::cout << name_topic_state <<"\n\n";
  else
  {
  	name_topic_state = "/gazebo_client/iris_state";
  	std::cout << "Nao tem\n\n";
  }

  if (n.getParam("name_topic_ref", name_topic_ref))
  //if (ros::param::get("name_topic_ref", name_topic_ref))
	std::cout << name_topic_ref <<"\n\n";
  else
  {
  	name_topic_ref  = "/my_iris/iris_ref";
  	std::cout << "Nao tem\n\n";
  }*/

  ros::Publisher topic_pub = n.advertise<geometry_msgs::Pose>(name_topic_ref, 1000);
  topic = topic_pub;
  ros::Subscriber sub = n.subscribe(name_topic_state, 1000, chatterCallback);


  /*
  ros::Publisher topic_pub = n.advertise<geometry_msgs::Pose>("/my_iris/iris_ref", 1000);
  topic = topic_pub;
  ros::Subscriber sub = n.subscribe("/gazebo_client/iris_state", 1000, chatterCallback);

  std::string globalname;
  if (n.getParam("globalname", globalname))
	std::cout << globalname <<"\n\n";
  else
  {
  	std::cout << "Nao tem\n\n";
  }*/

  //OSQP Setup
  osqp_solve(&workspace);
  int flag_bounds = osqp_update_bounds(&workspace, ldata, udata);

  iris_state_ref_command[0] = 1;
  iris_state_ref_command[3] = 0;

  ros::spin();

  return 0;
}

void chatterCallback(const sensor_msgs::ImuConstPtr& current_state)
{
  time_10m = time_10m + 0.01;


  //Calculate zpp
  sum_zpp = 0;
  for(int i = 0; i < (n_filter-1); i++){
    iris_zpp_points[i+1] = iris_zpp_points[i];
    sum_zpp = sum_zpp + iris_zpp_points[i];
  }
  iris_zpp_points[0] = (current_state->angular_velocity_covariance[2] - old_z_vel)/0.01;
  sum_zpp = sum_zpp + iris_zpp_points[0];
  iris_zpp = sum_zpp/(n_filter);
  old_z_vel = current_state->angular_velocity_covariance[2];

  //iris_state_ref_command[2] = 1;
  //ref_command.position.z = iris_state_ref_command[2];

  if (count_tick > (sample_time-1)){
    count_tick = 0;

    /*if (flag_ini == 0){
      flag_ini = 1;
      update_iris_state(current_state);
      x_ini = iris_state[0];
      y_ini = iris_state[1];
      std::cout <<  "Posicao inicial: " << x_ini << " " << y_ini << "\n\n";
    }*/

    //Update Iris State
    update_iris_state(current_state);
    /*
    //for(int i = 0; i < 6; i++){
   //   iris_state[i] = current_state->orientation_covariance[i];
   //   iris_state_vel[i] = current_state->angular_velocity_covariance[i];
   // }
      
    //calculate_control();
    //publish_setpoint_ref();
    
    if (count_ref > (sample_time-1)){
      std::cout << iris_state[2]  << "\n";
      count_ref = 0; 
      //Pub New Reference
      iris_state_ref[2]++;
      ref_command.position.z = iris_state_ref[2];
      publish_setpoint_ref(topic);
    }
    else{
      count_ref++;
    }
    */


    update_trajetory();

    calculate_control();

    publish_setpoint_ref(topic);

   // std::cout <<  xy_ref[0] << " " << xy_ref[1] << "\n";
   // std::cout <<  iris_state[0] << " " << iris_state[1] << "\n\n";

  }
  else{
    count_tick++;
  }


  // print test
  //iris_state[2] = current_state->orientation_covariance[2];
  //std::cout << iris_state[2]  << endl;
  //ROS_INFO("x = %.4f, y = %.4f theta = %.4f", pc[x],pc[y],pc[theta]);

}

void publish_setpoint_ref(const ros::Publisher& topic_pub)
{

    ref_command.position.z = iris_state_ref_command[0];
    ref_command.orientation.x = iris_state_ref_command[1];
    ref_command.orientation.y = iris_state_ref_command[2];
    ref_command.orientation.z = iris_state_ref_command[3];
    topic_pub.publish(ref_command);
}

void update_iris_state(const sensor_msgs::ImuConstPtr& current_state){
  for(int i = 0; i < 6; i++){
    //Update Iris State
    iris_state[i] = current_state->orientation_covariance[i];
    iris_state_vel[i] = current_state->angular_velocity_covariance[i];
  }

  rotor_vel[0] = current_state->orientation.x;
  rotor_vel[1] = current_state->orientation.y;
  rotor_vel[2] = current_state->orientation.z;
  rotor_vel[3] = current_state->orientation.w;
  //std::cout << rotor_vel[0] <<" "<< rotor_vel[1] <<" "<< rotor_vel[2] <<" "<< rotor_vel[3] << "\n\n";
}

void calculate_control(){
  state_array_mpc();

  update_OSQP_vectors(control_state_array, W, E, FT, number_control_states, number_control_actions, number_opt_var, number_ineq, &workspace);

  calculate_control_OSQP(control_action, control_state_array, number_control_states, number_control_actions, &workspace);

  //Convert control action to angular reference 
  control_action_to_angular_ref();
}


void state_array_mpc(){
    //xp
    control_state_array[0] = iris_state_vel[0];
    //x
    control_state_array[1] = iris_state[0];
    //yp
    control_state_array[2] = iris_state_vel[1];
    //y
    control_state_array[3] = iris_state[1];
    //xref
    control_state_array[4] = xy_ref[0];
    //yref
    control_state_array[5] = xy_ref[1];
   
}


void control_action_to_angular_ref(){
  empuxo = iris_KT*(pow(rotor_vel[0], 2) + pow(rotor_vel[1], 2) + pow(rotor_vel[2], 2) + pow(rotor_vel[3], 2));
  //std::cout <<  empuxo << "\n\n";

  //std::cout <<  iris_zpp << "\n\n";

  //control_action[0] = 0;
  //control_action[1] = 0;

 // std::cout <<  control_action[0] << " " << control_action[1] << "\n\n";

  sin_phi = -(control_action[0]*sin(iris_state[5]) + control_action[1]*cos(iris_state[5]))/empuxo;

  tg_theta_num = (control_action[0]*cos(iris_state[5]) + control_action[1]*sin(iris_state[5]));

  tg_theta_den = mass*(g - iris_zpp);

  //iris_state_ref[0] = 1;
  //iris_state_ref[3] = 0;

  if(sin_phi  > 1) sin_phi = 1;
  else if(sin_phi  < -1) sin_phi = -1;

  iris_state_ref_command[0] = 1;//0.5*sin(3.14*time_10m/4) + 1;
  iris_state_ref_command[1] = asin(sin_phi);
  iris_state_ref_command[2] = atan2(tg_theta_num, tg_theta_den);

  //std::cout <<  iris_state[5] << " " << sin_phi << "\n";
  //std::cout <<  iris_state_ref_command[1] << " " << iris_state_ref_command[2] << "\n\n";

}

void update_trajetory(){
  double time_traj = 15;

  if (flag_ini == 0){
      flag_ini = 1;
      x_ini = iris_state[0];
      y_ini = iris_state[1];
      std::cout <<  "Posicao inicial: " << x_ini << " " << y_ini << "\n\n";
  }


  xy_ref[0] = (16*pow(sin(2*3.14*time_10m/time_traj),3))/4;//3*cos(3.14*time_10m/4) - 3;
  xy_ref[1] = (13*cos(2*3.14*time_10m/time_traj) - 5*cos(2*2*3.14*time_10m/time_traj) - 2*cos(3*2*3.14*time_10m/time_traj) - cos(4*2*3.14*time_10m/time_traj) - 5)/4;//3*sin(3.14*time_10m/4);
  
  //iris_state_ref_command[0] = 2 - cos(2*3.14*time_10m/time_traj);

  //xy_ref[0] = x_ini;// + 2*cos(2*3.14*time_10m/time_traj) - 2;
  //xy_ref[1] = y_ini + 2*sin(2*3.14*time_10m/time_traj); 
  //std::cout << time_10m << "\n";
}
