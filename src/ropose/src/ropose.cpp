#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose2D.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ropose");

    ros::NodeHandle node;

    ros::Publisher pose_pub = node.advertise<geometry_msgs::Pose2D>("/ropose", 1);

    tf::TransformListener tf_listener;

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        tf::StampedTransform tf;

        try
        {
            tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), tf);
        }

        catch (tf::TransformException& ex)
        {
            rate.sleep();
            continue;
        }

        geometry_msgs::Pose2D pose;
        pose.x = tf.getOrigin().x();
        pose.y = tf.getOrigin().y();

        pose.theta = tf.getRotation().getAngle();

        pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
}
