#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <stdlib.h>
#include <math.h>

class ObstDetect
{
public:
    ObstDetect();

private:
    void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void bumpedCallback(const nav_msgs::Odometry::ConstPtr& msg);

    std_msgs::Float64 nearest_angle_;
    double nearest_range_;
    double old_pos_x_, old_pos_y_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odom_;
};

ObstDetect::ObstDetect(){
    // publish the angle location of the obstacle the robot has bumped into
    pub_ = nh_.advertise<std_msgs::Float64>("bump", 1000);

    // subscribe to laser and odometry information coming from stage_ros
    sub_laser_ = nh_.subscribe("base_scan", 1000, &ObstDetect::processScanCallback, this);
    // odometry information is sampled by the node odom_drop in order to not be overwhelmed by large amount of information
    sub_odom_ = nh_.subscribe("odom_drop", 1000, &ObstDetect::bumpedCallback, this);
}

void ObstDetect::processScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg){
    // iterate laser information to find the distance to the closest obstacle
    int n_ranges = msg->ranges.size();
    std::vector<float>::const_iterator min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    nearest_range_ = *min_it;

    // position of the closest obstacle in the laser array
    int arr_pos = std::distance(msg->ranges.begin(), min_it);

    // get angle knowing the array position of the closest obstacle
    nearest_angle_.data = (180/M_PI)*(arr_pos*(msg->angle_max - msg->angle_min)/n_ranges - msg->angle_max);

    ROS_INFO_STREAM("Nearest obstacle at range: " << nearest_range_ << " with bearing: " << nearest_angle_.data);
}

void ObstDetect::bumpedCallback(const nav_msgs::Odometry::ConstPtr &msg){

    // extract velocity and position
    double vel_x = msg->twist.twist.linear.x;
    double vel_z = msg->twist.twist.angular.z;
    double pos_x = msg->pose.pose.position.x;
    double pos_y = msg->pose.pose.position.y;

    // check if the robot wants to make a move
    bool move_query = (vel_x != 0 || vel_z != 0);

    // check if robot is stopped by comparing its position with the previous one
    bool stopped = (old_pos_x_ == pos_x &&
                    old_pos_y_ == pos_y);

    // check if there's an obstacle near and inside our region of movement (-100, 100)    
    bool obst_near = (nearest_range_ <= 0.6 &&
                      nearest_angle_.data < 100 &&
                      nearest_angle_.data > -100);

    // if the robot wants to make a move but there's an obstacle near that's keepint it still,
    // then the robot has bumped the wall
    if (move_query & stopped & obst_near){
        ROS_INFO_STREAM("Hit the wall at angle: " << nearest_angle_.data);
        pub_.publish(nearest_angle_);
    }

    old_pos_x_ = pos_x;
    old_pos_y_ = pos_y;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser");
    ObstDetect laser;

    ros::spin();

    return 0;
}
