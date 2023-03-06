#include <ros.h>
#include <geometry_msgs/Twist.h>

// callback function for the cmd_vel subscriber
void msg_cb(const geometry_msgs::Twist &msg);

// create a rosserial node
ros::NodeHandle nh;

// create a subscriber
ros::Subscriber<geometry_msgs::Twist> jetson_sub("/cmd_vel", &msg_cb);

void setup()
{
    Serial.begin(115200);
    //initialize the node
    nh.initNode();
    nh.subscribe(jetson_sub);
}

void loop()
{
    nh.spinOnce();
    delay(100);
}

void msg_cb(const geometry_msgs::Twist &msg)
{
    float linear_x = msg.linear.x;
    float linear_y = msg.linear.y;
    float linear_z = msg.linear.z;
    Serial.print(linear_x);
    Serial.print("  ");
    Serial.print(linear_y);
    Serial.print("  ");
    Serial.println(linear_z);
}
