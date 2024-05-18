#include "otto_odom.h"

// SI units only 
#define WHEEL_RADIUS 0.1016 
#define WHEELBASE 0.61 

OdomPublisher::OdomPublisher(){
    this->encoder_sub = nh.subscribe("otto/encoders", 1, &OdomPublisher::encoder_callback, this);
    this->odom_pub = nh.advertise<nav_msgs::Odometry>("otto/wheel_odom", 1, false);
    this->accumulator = std::vector<float>(3, (float)0.0);
}

void OdomPublisher::encoder_callback(const sensor_msgs::JointState::ConstPtr &ptr){

    sensor_msgs::JointState msg = *ptr;
    
    if(this->last_timestamp = -1){
        this->last_timestamp = msg.header.stamp.nsec;
        return;
    }

    // Diff drive model 
    float yaw_prime = (WHEEL_RADIUS/WHEELBASE) * (msg.velocity[0] - msg.velocity[1]);
    float v_prime = (WHEEL_RADIUS/2)*(msg.velocity[0] + msg.velocity[1]);

    float delta_t =  (msg.header.stamp.nsec - this->last_timestamp) * 1e-9;
    this->last_timestamp = msg.header.stamp.nsec;

    // Orientation data 
    this->accumulator[2] += delta_t * yaw_prime;
 
    //position data
    this->accumulator[0] += delta_t * (v_prime *cos(this->accumulator[2]));
    this->accumulator[1] += delta_t * (v_prime *sin(this->accumulator[2]));
    
    // Odom data 
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    
    odom.pose.pose.position.x = this->accumulator[1];
    odom.pose.pose.position.y = this->accumulator[0];
    odom.pose.pose.position.x = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->accumulator[2]);
    
    odom.twist.twist.linear.x = v_prime * cos(this->accumulator[2]);
    odom.twist.twist.linear.y = v_prime * sin(this->accumulator[2]);
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.z = yaw_prime;

    this->odom_pub.publish(odom);
     
}

int main(int argc, char** argv){

	ros::init(argc, argv, "wheel_odom_publisher");
	OdomPublisher odompub = OdomPublisher();
	
	while(ros::ok()){
		ros::spin();
	}	
}
