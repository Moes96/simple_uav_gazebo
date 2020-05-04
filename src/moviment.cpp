#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <string>

namespace gazebo
{
    class BasicMoviment : public ModelPlugin
    {
        ignition::math::Vector3d actual_position;
        ignition::math::Pose3<double> pose;
        std::string name;
        ros::Publisher chatter_pub;
    public:

        void subCallback(const std_msgs::String::ConstPtr &msg)
        {
            ROS_INFO("I heard: [%s]", msg->data.c_str());
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
                std::bind(&BasicMoviment::OnUpdate, this));

            ros::NodeHandle n;
            ros::Subscriber sub = n.subscribe("set_position", 1000, &BasicMoviment::subCallback, this);
            chatter_pub = n.advertise<geometry_msgs::Pose>("set_position", 1000);

        }
        // Called by the world update start event
        void OnUpdate()
        {
            geometry_msgs::Pose pose{};
            pose.position.x=1;
            ROS_INFO("%s", pose);
            chatter_pub.publish(pose);



            // if (actual_position.Distance(final_position) > 0.1)
            // {
            //     pose = this->model->WorldPose();
            //     actual_position = pose.Pos();

            //     //Calcolo il delta spazio ad ogni delta t e li sommo

            //     ignition::math::Vector3d velocity;
            //     velocity = (final_position - actual_position).Normalize() * 50;
            //     this->model->SetLinearVel(velocity);
            // }
            // else
            // {
            //     this->model->SetLinearVel(final_position * 0);
            // }
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BasicMoviment)
} // namespace gazebo