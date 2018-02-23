#ifndef _MYBOT_PLUGIN_HH_
#define _MYBOT_PLUGIN_HH_

// #include <thread>
// #include "ros/ros.h"
// #include "ros/callback_queue.h"
// #include "ros/subscribe_options.h"
// #include "std_msgs/Float32.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control a Mybot sensor.
  class MybotPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: MybotPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, Mybot plugin not loaded\n";
            return;
        }


        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint = _model->GetJoints()[3];

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0, 0);

        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);

        // // Default to zero velocity
        // double velocity = 0;
        // double position = 0;
        // double force = 0;

        // // Check that the velocity element exists, then read the value
        // if (_sdf->HasElement("velocity"))
        //     velocity = _sdf->Get<double>("velocity");
        // if (_sdf->HasElement("position"))
        //     velocity = _sdf->Get<double>("position");            
        // if (_sdf->HasElement("force"))
        //     velocity = _sdf->Get<double>("force");

        // this->SetVelocity(velocity);
        // this->SetPosition(position);
        // this->SetForce(force);

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        std::string velTopicName = "~/" + this->model->GetName() + "/vel_cmd";
        std::string posTopicName = "~/" + this->model->GetName() + "/pos_cmd";        
        std::string forceTopicName = "~/" + this->model->GetName() + "/force_cmd";        

        // Subscribe to the topic, and register a callback
        this->sub_vel = this->node->Subscribe(velTopicName, &MybotPlugin::OnVelMsg, this);
        this->sub_pos = this->node->Subscribe(posTopicName, &MybotPlugin::OnPosMsg, this);        
        this->sub_force = this->node->Subscribe(forceTopicName, &MybotPlugin::OnForceMsg, this);

        // // Initialize ros, if it has not already bee initialized.
        // if (!ros::isInitialized())
        // {
        // int argc = 0;
        // char **argv = NULL;
        // ros::init(argc, argv, "gazebo_client",
        //     ros::init_options::NoSigintHandler);
        // }

        // // Create our ROS node. This acts in a similar manner to
        // // the Gazebo node
        // this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // // Create a named topic, and subscribe to it.
        // ros::SubscribeOptions so =
        // ros::SubscribeOptions::create<std_msgs::Float32>(
        //     "/" + this->model->GetName() + "/vel_cmd",
        //     1,
        //     boost::bind(&MybotPlugin::OnRosMsg, this, _1),
        //     ros::VoidPtr(), &this->rosQueue);
        // this->rosSub = this->rosNode->subscribe(so);

        // // Spin up the queue helper thread.
        // this->rosQueueThread =
        // std::thread(std::bind(&MybotPlugin::QueueThread, this));
    }

    /// \brief Set the velocity of the Mybot
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
        // Set the joint's target velocity.
        this->model->GetJointController()->SetVelocityTarget(
            this->joint->GetScopedName(), _vel);
    }

    public: void SetPosition(const double &_pos)
    {
        double deg = _pos * M_PI / 180.0;
        // Set the joint's target pos.
        this->model->GetJointController()->SetPositionTarget(
            this->joint->GetScopedName(), deg);
    }
    public: void SetForce(const double &_force)
    {
        this->joint->SetForce(0, _force);
    }
        
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnVelMsg(ConstVector3dPtr &_msg)
    {
        this->SetVelocity(_msg->x());
    }    
    private: void OnPosMsg(ConstVector3dPtr &_msg)
    {
        this->SetPosition(_msg->x());
    }    
    private: void OnForceMsg(ConstVector3dPtr &_msg)
    {
        this->SetForce(_msg->x());
    }        

    // /// \brief Handle an incoming message from ROS
    // /// \param[in] _msg A float value that is used to set the velocity
    // /// of the mybot.
    // public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    // {
    //     //this->SetVelocity(_msg->data);
    //     this->SetPosition(_msg->data * M_PI/180.0);
    // }

    // /// \brief ROS helper function that processes messages
    // private: void QueueThread()
    // {
    //     static const double timeout = 0.01;
    //     while (this->rosNode->ok())
    //     {
    //         this->rosQueue.callAvailable(ros::WallDuration(timeout));
    //     }
    // }    

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub_vel;    
    private: transport::SubscriberPtr sub_pos;    
    private: transport::SubscriberPtr sub_force;        

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    // /// \brief A node use for ROS transport
    // private: std::unique_ptr<ros::NodeHandle> rosNode;

    // /// \brief A ROS subscriber
    // private: ros::Subscriber rosSub;

    // /// \brief A ROS callbackqueue that helps process messages
    // private: ros::CallbackQueue rosQueue;

    // /// \brief A thread the keeps running the rosQueue
    // private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MybotPlugin)
}

#endif // _MYBOT_PLUGIN_HH_