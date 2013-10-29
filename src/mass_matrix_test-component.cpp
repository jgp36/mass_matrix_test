#include "mass_matrix_test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace Eigen;

Mass_matrix_test::Mass_matrix_test(std::string const& name) 
  : TaskContext(name), 
    A(Matrix7x7d::Zero(7,7)), 
    jacobian(Matrix6x7d::Zero(6,7)),
    N(Matrix7x7d::Zero(7,7)),
    lambda(Matrix6x6d::Zero(6,6)),
    t_out(1), t_last(0), t_disp(0) {

  //Add Ports
  this->addPort("JointState", port_joint_state);
  this->addPort("CartesianPositionFrame", port_cart_frame);

  this->addPort("MassMatrix", port_mass_matrix);
  this->addPort("Jacobian", port_jacobian);

  this->addPort("JointEffortCommand", port_joint_efforts);
  this->addPort("FriJointImpedance", port_fri_joint_impedance);
}

bool Mass_matrix_test::configureHook(){

  //Set joint impedance and initialize joint efforts
  for (size_t ii(0); ii < 7; ++ii) {
     fri_joint_impedance.stiffness[ii] = 0;
     fri_joint_impedance.damping[ii] = 0;
     joint_efforts.efforts.push_back(0);
  }

  return true;
}

bool Mass_matrix_test::startHook(){

  //Time start
  t_start = RTT::os::TimeService::Instance()->getTicks();

  return true;
}

void Mass_matrix_test::updateHook(){

  //Write joint impedance
  port_fri_joint_impedance.write(fri_joint_impedance);

  //Read the current time
  t_cur = RTT::os::TimeService::Instance()->getSeconds(t_start);

  //Read and update the state
  if (port_joint_state.read(joint_state) == NewData) {

     port_mass_matrix.read(mass_matrix);

     //Convert to Eigen for easier computations
     for (size_t ii(0); ii<7; ++ii) {
       for (size_t jj(0); jj<7; ++jj) {
         A(ii,jj) = mass_matrix.mass[7*ii+jj];
       }
     }

     port_jacobian.read(jacobian_kdl);

     //Convert to Eigen for easier computations
     for (size_t ii(0); ii<6; ++ii) {
       for (size_t jj(0); jj<7; ++jj) {
         jacobian(ii,jj) = jacobian_kdl(ii,jj);
       }
     }

     //Compute the operational space mass matrix assuming non-sinuglar pose
     lambda = (jacobian*A.inverse()*jacobian.transpose()).inverse();

     //Compute the dynamically consistent nullspace
     N = Matrix7x7d::Identity(7,7) - A.inverse()*jacobian.transpose()*lambda*jacobian;

     //Compute desired torque to follow the trajectory
     for(size_t ii(0); ii<7; ++ii) {
       torque(ii) = 0;
     }

     //Project into the nullspace
     torque = N.transpose()*torque;

     //Write to robot
     for (size_t ii(0); ii<7; ++ii) {
       joint_efforts.efforts[ii] = torque(ii);
     }
     port_joint_efforts.write(joint_efforts);
  }

  //Data display
  if (t_cur - t_disp > t_out) {
    std::cout << "Time: " << t_cur<< std::endl;
    std::cout << "Joint Position: " ;
    for (size_t ii(0); ii<7; ++ii) {
      std::cout << joint_state.position[ii] << " ";
    }
    std::cout << std::endl;
    std::cout << "Joint Torque: " << torque.transpose() << std::endl;

    std::cout << "End Effector Frame:" << std::endl;
    for (size_t ii(0); ii<4; ++ii) {
      for (size_t jj(0); jj<4; ++jj) {
        std::cout << cart_frame_kdl(ii,jj) << " ";
      }
      std::cout << std::endl;
    }

    //Theoretical Contribution to end effector pose
    Vector6d t_ee(jacobian*torque);
    std::cout << "Theoretical EE force: " << t_ee.transpose() << std::endl;

    std::cout << std::endl;
    t_disp = t_cur;
  }

}

void Mass_matrix_test::stopHook() {

  //Write zero torque
  for (size_t ii(0); ii < 7; ++ii) {
    joint_efforts.efforts[ii] = 0.0;
  }
  port_joint_efforts.write(joint_efforts);

}

void Mass_matrix_test::cleanupHook() {

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Mass_matrix_test)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Mass_matrix_test)
