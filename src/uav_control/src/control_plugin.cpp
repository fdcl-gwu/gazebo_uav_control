#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "uav_control/states.h"

namespace gazebo
{

class UavControlPlugin : public ModelPlugin
{

public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        this->link = this->model->GetLink((std::string) "quadrotor::link");

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin( \
            std::bind(&UavControlPlugin::update, this));

       // Start the ros publisher node.
        this->pub_state = n.advertise<uav_control::states>( \
            "states", 10);
    }


    void update(void)
    {
        // The "state" message is defined in "/msg" directory, and will be 
        // converted into "states.h" inside "devel/include" when you call the
        // "catkin_make".

        state.header.stamp = ros::Time::now();
        state.header.frame_id = (std::string) "uav_states";

        // "pose" include both position and the rotation.
        ignition::math::Pose3d pose = this->model->WorldPose();
        vector_to_msg(pose.Pos(), state.x);

        ignition::math::Quaterniond rot = pose.Rot();
        state.R.x = rot.Yaw();
        state.R.y = rot.Pitch();
        state.R.z = rot.Roll();

        // Get other states of the model.
        vector_to_msg(this->model->WorldLinearVel(), state.v);
        vector_to_msg(this->model->WorldAngularVel(), state.W);
        vector_to_msg(this->model->WorldLinearAccel(), state.a);

        // Publish all the states of the model to the ros publisher.
        this->pub_state.publish(state);

        // Define desired states.
        ignition::math::Vector3d xd(5.0, 0.0, 0.5);
        ignition::math::Vector3d vd(0.0, 0.0, 0.0);

        // Calculate errors.
        ex = pose.Pos() - xd;
        ev = this->model->WorldLinearVel() - vd;
        ei = ei + ex * 0.001;

        // Simple PID control.
        force = - 10.0 * ex - 8.0 * ev - 5.0 * ei;
        double m = 1.0;
        double g = 9.8;
        force[2] = force[2] + m * g;

        // Apply the control force determined with PID controller.
        this->link->SetForce(force);

        // // std::cout << ex << std::endl;
    }


    void vector_to_msg(
            ignition::math::Vector3d vec, geometry_msgs::Vector3 &msg_vec
    )
    {
       msg_vec.x = vec[0];
       msg_vec.y = vec[1];
       msg_vec.z = vec[2];
    } 


private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    
    event::ConnectionPtr update_connection;
    ros::Publisher pub_state; 
    ros::NodeHandle n;
    uav_control::states state;

    ignition::math::Vector3d ex;
    ignition::math::Vector3d ev;
    ignition::math::Vector3d ei;
    ignition::math::Vector3d force;
};


// Register this plugin with the simulator.
GZ_REGISTER_MODEL_PLUGIN(UavControlPlugin)


}  // End of namespace gazebo.

