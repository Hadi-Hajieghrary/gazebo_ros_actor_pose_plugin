#include "gazebo_ros_actor_pose.h"

#include <iostream>

using namespace gazebo;

void ActorPosePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf){

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("ActorPosePlugin", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

    this->_sdf = sdf;
    this->_actor = boost::dynamic_pointer_cast<physics::Actor>(model);
    this->_world = this->_actor->GetWorld();

    this->_link_name = std::string(this->_actor->GetName() + "_pose");
    this->_link = model->GetLink(this->_link_name);
    this->_frame_name = this->_link_name;

    this->_last_update_time = this->_world->SimTime();

    // Load Parameters of the plugin
    if (!(this->_sdf->HasElement("topicName"))){
        this->_topic_name = this->_link_name;
    }
    else{
        this->_topic_name = this->_sdf->GetElement("topicName")->Get<std::string>();
    }

    if (!_sdf->HasElement("updateRate")){
        ROS_DEBUG_NAMED("ActorPosePlugin", "ActorPose plugin missing <updateRate>, defaults to 0.0: as fast as possible!");
        this->_update_rate = 0;
    }
    else{
        this->_update_rate = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->_rosnode = new ros::NodeHandle();
    
    // publish multi queue
    this->_pub_multi_queue.startServiceThread();
    
    if (this->_topic_name != ""){
        this->_pub_queue = this->_pub_multi_queue.addPub<geometry_msgs::PoseStamped>();
        this->_pub = this->_rosnode->advertise<geometry_msgs::PoseStamped>(this->_topic_name, 1);
        this->_callback_queue_thread = boost::thread(boost::bind(&ActorPosePlugin::_PoseQueueThread, this));
    }
    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->_update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ActorPosePlugin::_OnUpdate, this));
}

void ActorPosePlugin::_OnUpdate(){

    if (!this->_link){
        return;
    }

    common::Time cur_time = this->_world->SimTime();

    if (cur_time < this->_last_update_time){
        ROS_WARN_NAMED("ActorPosePlugin", "Simulation time is not moving forward");
        this->_last_update_time = cur_time;
    }
    // Time to the update rate
    auto elapsed_time = cur_time - this->_last_update_time;
    if ( (this->_update_rate > 0) && (elapsed_time.Double() < (1.0 / this->_update_rate)) ){
        return;
    }
 
    std::scoped_lock lock(_mtx);        
    ignition::math::Pose3d cur_pose = this->_actor->WorldPose();

    this->_pose_msg->header.frame_id = this->_reference_frame_name;
    this->_pose_msg->header.stamp.sec = cur_time.sec;
    this->_pose_msg->header.stamp.nsec = cur_time.nsec;

    this->_pose_msg->pose.position.x    = cur_pose.Pos().X();
    this->_pose_msg->pose.position.y    = cur_pose.Pos().Y();
    this->_pose_msg->pose.position.z    = cur_pose.Pos().Z();

    this->_pose_msg->pose.orientation.x = cur_pose.Rot().X();
    this->_pose_msg->pose.orientation.y = cur_pose.Rot().Y();
    this->_pose_msg->pose.orientation.z = cur_pose.Rot().Z();
    this->_pose_msg->pose.orientation.w = cur_pose.Rot().W();

    this->_last_update_time = cur_time;
   
    // publish to ros
    this->_pub_queue->push(*(this->_pose_msg), this->_pub);

}

void ActorPosePlugin::_PoseQueueThread()
{
    static const double timeout = 0.01;
    while (this->_rosnode->ok()){
        this->_actor_pose_queue.callAvailable(ros::WallDuration(timeout));
    }

}
