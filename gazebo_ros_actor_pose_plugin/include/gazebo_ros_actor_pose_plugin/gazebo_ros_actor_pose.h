#ifndef _ACTOR_POSE_H_
#define _ACTOR_POSE_H_

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <gazebo_plugins/PubQueue.h>


namespace gazebo
{
  class GAZEBO_VISIBLE ActorPosePlugin : public ModelPlugin
  {
    public: 
    
        ActorPosePlugin() : ModelPlugin(){

        }

        ~ActorPosePlugin(){

          this->_update_connection.reset();
          // Finalize the controller
          this->_rosnode->shutdown();
          this->_actor_pose_queue.clear();
          this->_actor_pose_queue.disable();
          this->_callback_queue_thread.join();
          delete this->_rosnode;
          this->_rosnode = nullptr;
        }


        /*
         \brief Load the actor plugin.
         \param[in] model Pointer to the parent model.
         \param[in] sdf Pointer to the plugin's SDF elements.
        */
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    private:

        /*
         \brief Function that is called every update cycle.
         \param[in] _info Timing information
        */
        void _OnUpdate();

        void _PoseQueueThread();

        //brief pointer to ros node
        ros::NodeHandle* _rosnode{nullptr};
        ros::Publisher _pub{};
        geometry_msgs::PoseStamped::Ptr _pose_msg{new geometry_msgs::PoseStamped};
        
        std::string _topic_name{};
        std::string _frame_name{};
        
        std::string _robot_namespace{};
        // ros publish multi queue, prevents publish() blocking
        PubMultiQueue _pub_multi_queue{};
        PubQueue<geometry_msgs::PoseStamped>::Ptr _pub_queue{nullptr};
        ros::CallbackQueue _actor_pose_queue{};
        boost::thread _callback_queue_thread{};
        
        
        //brief Pointer to the parent actor.
        physics::ActorPtr _actor{nullptr};
        //brief Pointer to the world
        physics::WorldPtr _world{nullptr};
        //brief Pointer to the sdf element.
        sdf::ElementPtr _sdf{nullptr};
        physics::LinkPtr _link{nullptr};
        //brief Velocity of the actor
        ignition::math::Vector3d _velocity{};
        //brief Pointer to the update event connection
        event::ConnectionPtr _update_connection{nullptr};
        //brief Time of the last update.
        common::Time _last_update_time{};
        //brief Rate of publisher update
        double _update_rate{0.0};

        std::mutex _mtx;
        std::string _link_name;
        const std::string _reference_frame_name{"world"};


  };
  GZ_REGISTER_MODEL_PLUGIN(ActorPosePlugin)
}

#endif