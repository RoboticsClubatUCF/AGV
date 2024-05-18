#include <ros/ros.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <tf/transform_datatypes.h>

class OdomPublisher {

    public:
        OdomPublisher();
        // Callback for encoder data 
        void encoder_callback(const sensor_msgs::JointState::ConstPtr& msg);

    private:     
        ros::NodeHandle nh;
        ros::Subscriber encoder_sub;
        ros::Publisher  odom_pub;

        // Epoch time of last measurement in nanoseconds
        uint64_t last_timestamp = -1;

        // Accumulated position estimate (x,y,theta)
        std::vector<float> accumulator;
};
