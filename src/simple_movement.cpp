#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace gazebo
{
    class BasicMovement : public ModelPlugin
    {
        ignition::math::Pose3<double> actual_position;
        // geometry_msgs::PoseStamped actual_goal;
        ignition::math::Pose3<double> actual_goal;
        ignition::math::Pose3<double> pose;
        std::string name;
        ros::Subscriber sub; //Subscriber for the topic

    public:
        void subCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            actual_goal.Pos().X() = msg->pose.position.x;
            actual_goal.Pos().Y() = msg->pose.position.y;
            actual_goal.Pos().Z() = msg->pose.position.z;
            actual_goal.Rot().W() = msg->pose.orientation.w;
            actual_goal.Rot().X() = msg->pose.orientation.x;
            actual_goal.Rot().Y() = msg->pose.orientation.y;
            actual_goal.Rot().Z() = msg->pose.orientation.z;

            ROS_INFO("I heard %s", msg);
        }

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            this->model = _parent;
            //Setup of this drone Agent
            name = this->model->GetName();

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BasicMovement::OnUpdate, this));

            ros::NodeHandle n;
            sub = n.subscribe("/set_position", 1000, &BasicMovement::subCallback, this);
        }
        // Called by the world update start event
        void OnUpdate()
        {
            actual_position = this->model->WorldPose();
            if (actual_position.Pos().Distance(actual_goal.Pos()) > 0.001)
            {
                pose = this->model->WorldPose();

                //Calcolo il delta spazio ad ogni delta t e li sommo

                ignition::math::Vector3d velocity;
                velocity = (actual_goal.Pos() - actual_position.Pos()).Normalize() * 5;
                this->model->SetLinearVel(velocity);
            }
            else
            {
                this->model->SetLinearVel(actual_goal.Pos() * 0);
            }
            ros::spinOnce();
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BasicMovement)
} // namespace gazebo