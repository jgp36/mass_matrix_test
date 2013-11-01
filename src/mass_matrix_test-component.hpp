#ifndef OROCOS_MASS_MATRIX_TEST_COMPONENT_HPP
#define OROCOS_MASS_MATRIX_TEST_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <Eigen/Geometry>

#include <lwr_fri/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

using namespace RTT;

#define Matrix7x7d Eigen::Matrix<double,7,7>
#define Matrix6x7d Eigen::Matrix<double,6,7>
#define Matrix6x6d Eigen::Matrix<double,6,6>
#define Vector7d Eigen::Matrix<double,7,1>
#define Vector6d Eigen::Matrix<double,6,1>

class Mass_matrix_test : public RTT::TaskContext{
  public:
    Mass_matrix_test(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    //State ports and messages
    InputPort<sensor_msgs::JointState> port_joint_state; 
    sensor_msgs::JointState joint_state;
    InputPort<lwr_fri::FriJointState> port_fri_joint_state;
    lwr_fri::FriJointState fri_joint_state;
    InputPort<KDL::Frame> port_cart_frame;
    KDL::Frame cart_frame_kdl;

    //Dynamics and kinematics ports and messages
    InputPort<lwr_fri::MassMatrix> port_mass_matrix;
    lwr_fri::MassMatrix mass_matrix;
    Matrix7x7d A;
    InputPort<KDL::Jacobian> port_jacobian;
    KDL::Jacobian jacobian_kdl;
    Matrix6x7d jacobian;

    //Output ports and messages
    OutputPort<motion_control_msgs::JointEfforts> port_joint_efforts;
    motion_control_msgs::JointEfforts joint_efforts;
    OutputPort<motion_control_msgs::JointPositions> port_joint_pos_command;
    motion_control_msgs::JointPositions joint_pos_command;
    OutputPort<lwr_fri::FriJointImpedance> port_fri_joint_impedance;
    lwr_fri::FriJointImpedance fri_joint_impedance;

    //Nullspace computation and control variables
    Matrix7x7d N;
    Matrix6x6d lambda;
    Vector7d torque;
    float kp;
    float period;
    Vector7d offset;

    //Data display timers
    float t_disp;
    os::TimeService::ticks t_start;
    os::TimeService::Seconds t_cur;
    os::TimeService::Seconds t_last_disp;

    //Data logging
    std::vector< Vector7d > jpos_log;
    std::vector< Vector7d > des_torque_log;
    Vector7d act_torque;
    std::vector< Vector7d > act_torque_log;
    Vector6d t_ee;
    std::vector< Vector6d > des_ee_force_log;
    std::vector< KDL::Frame > act_ee_pos_log;
    std::string filename;
    float t_log;
    os::TimeService::Seconds t_last_log;
    
};
#endif
