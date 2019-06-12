/* Provided by zsaigol on http://answers.ros.org/question/189159/bad-maps-produced-by-gmapping-in-simulation-with-feature-poor-environments/
Prints, the Gazebo model which I think is the ground truth, via topic /gazebo/model_states, the odometry published by Gazebo and the robot pose published by gmapping */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>

tf::TransformListener * listener;
bool received_gazebo_message = false;
geometry_msgs::Pose model_pose;

void receivedGazeboMessage(const gazebo_msgs::ModelStatesPtr& model_states)
{
    if (!received_gazebo_message)
    {
        std::cout << "Gazebo model info received!" << std::endl;
    }
    for (unsigned int i = 0; i < model_states->name.size(); ++i)
    {
        if (model_states->name[i] == "mobile_base")
        {
            model_pose = model_states->pose[i];
            received_gazebo_message = true;
            break;
        }
    }
}

void printPoses(const ros::TimerEvent&)
{
    if (received_gazebo_message)
    {
        tf::StampedTransform odom_transform;
        tf::StampedTransform gmap_transform;
        try
        {
            listener->lookupTransform("/odom", "/base_footprint", ros::Time(0), odom_transform);
            listener->lookupTransform("/map", "/base_footprint", ros::Time(0), gmap_transform);
        }
        catch (tf::TransformException & ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3);
        std::cout << "* at " << odom_transform.stamp_.toSec() << ":" << std::endl;

        std::cout << "Model: " << model_pose.position.x << ", " << model_pose.position.y <<
                " / " << tf::getYaw(model_pose.orientation) << std::endl;
        std::cout << "Odom : " << odom_transform.getOrigin()[0] << ", " <<
                odom_transform.getOrigin()[1] <<
                " / " << tf::getYaw(odom_transform.getRotation()) << std::endl;
        std::cout << "Gmapg: " << gmap_transform.getOrigin()[0] << ", " <<
                gmap_transform.getOrigin()[1] <<
                " / " << tf::getYaw(gmap_transform.getRotation()) << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_printer");
    ros::NodeHandle n;
    listener = new tf::TransformListener();

    ros::Subscriber gazebo_ground_truth_sub =
                n.subscribe("/gazebo/model_states", 1, &receivedGazeboMessage);
    ros::Timer timer1 = n.createTimer(ros::Duration(1.0), printPoses);

    ros::spin();
    return 0;
}
