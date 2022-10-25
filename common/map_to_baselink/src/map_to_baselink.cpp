#include "map_to_baselink.h"
geometry_msgs::Pose M2B::transformPose(const geometry_msgs::Pose &pose,
                                       const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::PoseStamped orig_pose;
    orig_pose.pose = pose;
    tf2::doTransform(orig_pose, transformed_pose, transform);

    return transformed_pose.pose;
}

geometry_msgs::TransformStamped M2B::getTransform(const std::string &target,
                                                  const std::string &source)
{
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = _tf_buffer->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
    }
    catch (const tf2::LookupException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return tf;
}

void M2B::callbackTimerPublishTF(const ros::TimerEvent &e)
{
    getPose();
    publishTF();
}

void M2B::getPose()
{
    gazebo_msgs::GetModelState model_pose;
    model_pose.request.model_name = "racebot";
    if (_srv_get_pose.call(model_pose))
    {
        // ROS_INFO("get model_pose");
        _base_pose_ground_truth.pose.pose = model_pose.response.pose;
    }
    else
    {
    }
}

void M2B::publishTF()
{

    // map->base_footprint
    geometry_msgs::TransformStamped m2b_transform;

    m2b_transform.header.frame_id = map_frame;
    m2b_transform.header.stamp = ros::Time::now();

    m2b_transform.child_frame_id = base_link_frame;

    m2b_transform.transform.translation.x = _base_pose_ground_truth.pose.pose.position.x ;
    m2b_transform.transform.translation.y = _base_pose_ground_truth.pose.pose.position.y ;
    m2b_transform.transform.translation.z = _base_pose_ground_truth.pose.pose.position.z;

    double roll, pitch, yaw;
    tf2::getEulerYPR(_base_pose_ground_truth.pose.pose.orientation, yaw, pitch, roll);

    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    m2b_transform.transform.rotation = q;

    _tf_broadcaster.sendTransform(m2b_transform);

    //广播结束
}

M2B::M2B() : _nh(""), _private_nh("~")
{
    _private_nh.param<std::string>("map_frame", map_frame, "map");
    _private_nh.param<std::string>("base_link_frame", base_link_frame, "base_footprint");

    _srv_get_pose = _nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

    _timer_tf = _nh.createTimer(ros::Duration(0.02), &M2B::callbackTimerPublishTF, this);

    _tf_buffer = std::make_shared<tf2_ros::Buffer>();
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_baselink");

    M2B obj;

    ros::spin();

    return 0;
}