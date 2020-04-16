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
#include "fdcl/common_types.hpp"


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

        // Get inertial properties.
        physics::InertialPtr uav_inertia = this->link->GetInertial();
        m = uav_inertia->Mass();
        J(0, 0) = uav_inertia->IXX();
        J(0, 1) = uav_inertia->IXY();
        J(0, 2) = uav_inertia->IXZ();
        J(1, 0) = uav_inertia->IXY();
        J(1, 1) = uav_inertia->IYY();
        J(1, 2) = uav_inertia->IYZ();
        J(2, 0) = uav_inertia->IXZ();
        J(2, 1) = uav_inertia->IYZ();
        J(2, 2) = uav_inertia->IZZ();

        std::cout << "\nUAV properties:"
            << "\n m: " << m
            << "\n J: " << J
            << std::endl;
        
        // Set gain matrices.
        kP(0, 1) = kp;
        kP(1, 0) = -kp;
        
        kD(0, 1) = kd;
        kD(1, 0) = -kd;

        kX << kx, 0.0, 0.0,
           0.0, kx, 0.0,
           0.0, 0.0, kx;
        
        kV << kv, 0.0, 0.0,
           0.0, kv, 0.0,
           0.0, 0.0, kv;
    }


    void update(void)
    {
        // The "state" message is defined in "/msg" directory, and will be 
        // converted into "states.h" inside "devel/include" when you call the
        // "catkin_make".

        state.header.stamp = ros::Time::now();
        state.header.frame_id = (std::string) "uav_states";

        update_states();
        update_ros_message();

        // Wait a few seconds for the Gazebo to start. If not, the simulation
        // might be running while the Gazebo GUI is not completely loaded.
        ros::Time t = ros::Time::now();
        ros::Duration t_elapsed = t - t0;
        if (t_elapsed.toSec() < 2.0) 
        {
            return;
        }

        // Define desired states.
        xd << 1.0, 1.0, 1.0;
        vd = (x - xd) / 2.0;

        // Calculate errors.
        ex = x - xd;
        ev = v - vd;

        // Calculate desired attitude.
        Rd = kP * ex + kD * ev;
        Rd(0) = saturate(Rd(0), -M_PI/3, M_PI/3); 
        Rd(1) = saturate(Rd(1), -M_PI/3, M_PI/3); 
        Wd << 0.0, 0.0, 0.0;       

        // Calculate attitude errors.
        eR = R - Rd;
        eW = W - Wd;

        // Simple PID control.
        fdcl::Vector3 e3(0.0, 0.0, 1.0);
        fdcl::Vector3 M = - kR * eR - kW * eW;
        fdcl::Vector3 F = - kX * ex - kV * ev + m * g * e3;

        // Get rotation matrix from Euler angles.
        fdcl::Matrix3 rot_mat;
        euler_to_R(R, rot_mat);

        // F is in world coordinates. Determine the force normal to the motor
        // plane of the UAV.
        fdcl::Vector3 Re3 = rot_mat * e3;
        double f = Re3.transpose() * F;

        force << 0.0, 0.0, f;

        // Apply the control force determined with PID controller.
        // Force is applied in world frame in Gazebo. Convert the force from
        // body frame to world frame.
        ignition::math::Vector3d force_out, moment_out;
        fdcl::Vector3 force_world = rot_mat * force;
        eigen_to_ignition(force_world, force_out);
        this->link->SetForce(force_out);
       
        // Torque is applied in body frame. 
        eigen_to_ignition(M, moment_out);
        this->link->SetTorque(moment_out);

        // std::cout << "ex: " << ex.transpose() << std::endl;
    }


    void update_states(void)
    {
        // "pose" include both position and the rotation.
        ignition::math::Pose3d pose = this->model->WorldPose();
        ignition::math::Quaterniond rot = pose.Rot();
        
        ignition_to_eigen(pose.Pos(), x);
        ignition_to_eigen(pose.Rot().Euler(), R);

        // Get other states of the model.
        ignition_to_eigen(this->model->WorldLinearVel(), v);
        ignition_to_eigen(this->model->WorldLinearAccel(), a);
        ignition_to_eigen(this->model->WorldAngularVel(), W);
    }


    void update_ros_message(void)
    {
        // Update the ROS message.
        vector_to_msg(x, state.x);
        vector_to_msg(v, state.v);
        vector_to_msg(a, state.a);
        vector_to_msg(R, state.R);
        vector_to_msg(W, state.W);

        // Publish all the states of the model to the ros publisher.
        this->pub_state.publish(state);
    }


    void vector_to_msg(fdcl::Vector3 vec, geometry_msgs::Vector3 &msg_vec)
    {
        msg_vec.x = vec(0);
        msg_vec.y = vec(1);
        msg_vec.z = vec(2);
    } 


    void ignition_to_eigen(
        const ignition::math::Vector3d input, fdcl::Vector3 &output
    )
    {
        output(0) = input[0];
        output(1) = input[1];
        output(2) = input[2];
    } 


    void eigen_to_ignition(
        const fdcl::Vector3 input, ignition::math::Vector3d &output
    )
    {
        output[0] = input(0);
        output[1] = input(1);
        output[2] = input(2);
    } 


    void euler_to_R(const fdcl::Vector3 rpy, fdcl::Matrix3 &R)
    {
        const double r = rpy(0), p = rpy(1), y = rpy(2);
        const double cr = cos(r), sr = sin(r);
        const double cp = cos(p), sp = sin(p);
        const double cy = cos(y), sy = sin(y);

        R(0, 0) = cy * cp;
        R(1, 0) = cy * sp * sr + sy * cr;
        R(2, 0) = - cy * sp * cr + sy * sr;
        R(0, 1) = - sy * cp;
        R(1, 1) = - sy * sp * sr + cy * cr;
        R(2, 1) = sy * sp * cr + cy * sr;
        R(0, 2) = sp;
        R(1, 2) = - cp * sr;
        R(2, 2) = cp * cr;
    }


    double saturate(const double a, const double min_a, const double max_a)
    {
        if (a < min_a) return min_a;
        if (a > max_a) return max_a;
        return a;
    }


private:
    ros::Time t0 = ros::Time::now();

    physics::ModelPtr model;
    physics::LinkPtr link;
    
    event::ConnectionPtr update_connection;
    ros::Publisher pub_state; 
    ros::NodeHandle n;
    uav_control::states state;

    fdcl::Vector3 x = fdcl::Vector3::Zero();
    fdcl::Vector3 v = fdcl::Vector3::Zero();
    fdcl::Vector3 a = fdcl::Vector3::Zero();
    fdcl::Vector3 R = fdcl::Vector3::Zero();
    fdcl::Vector3 W = fdcl::Vector3::Zero();
    
    fdcl::Vector3 xd = fdcl::Vector3::Zero();
    fdcl::Vector3 vd = fdcl::Vector3::Zero();
    fdcl::Vector3 Rd = fdcl::Vector3::Zero();
    fdcl::Vector3 Wd = fdcl::Vector3::Zero();

    fdcl::Vector3 ex = fdcl::Vector3::Zero();
    fdcl::Vector3 ev = fdcl::Vector3::Zero();
    fdcl::Vector3 ei = fdcl::Vector3::Zero();
    fdcl::Vector3 eR = fdcl::Vector3::Zero();
    fdcl::Vector3 eW = fdcl::Vector3::Zero();

    fdcl::Vector3 force = fdcl::Vector3::Zero();

    double m = 1.0;
    double g = 9.8;
    fdcl::Matrix3 J = fdcl::Matrix3::Identity();

    // Gains
    const double kx = 10;
    const double kv = 10;
    const double kR = 5.0;
    const double kW = 2.5;
    const double kp = 0.1;
    const double kd = 0.1;

    fdcl::Matrix3 kP = fdcl::Matrix3::Zero();
    fdcl::Matrix3 kD = fdcl::Matrix3::Zero();
    fdcl::Matrix3 kX = fdcl::Matrix3::Zero();
    fdcl::Matrix3 kV = fdcl::Matrix3::Zero();
};


// Register this plugin with the simulator.
GZ_REGISTER_MODEL_PLUGIN(UavControlPlugin)


}  // End of namespace gazebo.

