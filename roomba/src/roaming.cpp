#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

class Roaming
{
public:
    Roaming();

    std::string state;

    void spiral();
    void forward();
    //follow_wall(); // TODO: implement a state in which the robot follows the wall once bumped

private:
    void bumpCallback(const std_msgs::Float64::ConstPtr& bearing);

    double linear_, angular_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    ros::Rate roam_rate_;
};

Roaming::Roaming():
    state("spiral"),
    linear_(0),
    angular_(0),
    roam_rate_(5)
{
    // publish the velocity command
    pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    // subscribe to get the angle location of the obstacle the robot has bumped into
    sub_ = nh_.subscribe("bump", 1, &Roaming::bumpCallback, this);

    srand(time(0));
}

void Roaming::spiral() {
    //ROS_INFO("Spiral motion");

    geometry_msgs::Twist msg;

    // in order to obtain a spiral motion:
    // keep a constant angular velocty and increase the linear velocity
    angular_ = 1.1; linear_ = linear_ + 0.05;
    msg.linear.x = linear_;
    msg.angular.z = angular_;

    ROS_INFO_STREAM("Spiral motion with linear = " << linear_ << " and angular = " << angular_);

    pub_.publish(msg);
    roam_rate_.sleep();
}

void Roaming::forward() {
    ROS_INFO("Forward motion");

    geometry_msgs::Twist msg;

    linear_=0.5; angular_=0;
    msg.linear.x = linear_;
    msg.angular.z = angular_;

    pub_.publish(msg);
    roam_rate_.sleep();
}

void Roaming::bumpCallback(const std_msgs::Float64::ConstPtr& bearing) {
    ROS_INFO("Bump. Changing direction");

    geometry_msgs::Twist msg;

    // go back two steps
    for (int i = 0; i < 2; i++){
        linear_=-0.5; angular_=0;
        msg.linear.x = linear_;
        msg.angular.z = angular_;

        pub_.publish(msg);
        roam_rate_.sleep();
    }

    // depending on the sign of minimal angle, turn right, left, or random
    if (bearing->data < -10){
        linear_=0; angular_=3;
    }
    else if (bearing->data > 10){
        linear_=0; angular_=-3;
    }
    else{
        linear_=0;
        angular_=3*(round (2* ((double)rand() / (double)RAND_MAX)) - 1);
        ROS_INFO_STREAM("Random angular velocity: " << angular_);
    }
    msg.linear.x = linear_;
    msg.angular.z = angular_;

    pub_.publish(msg);
    roam_rate_.sleep();
    pub_.publish(msg); // turn twice to assure avoidance
    roam_rate_.sleep();

    state = "forward";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roaming");
    Roaming roaming;

    while(ros::ok()){
        if (roaming.state == "spiral"){
            roaming.spiral();
        }
        else if (roaming.state == "forward"){
            roaming.forward();
        }

        ros::spinOnce();
    }

    return 0;
}
