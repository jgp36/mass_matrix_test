#include "mass_matrix_test-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>
#include <Eigen/SVD>

using namespace Eigen;

Mass_matrix_test::Mass_matrix_test(std::string const& name) 
  : TaskContext(name), 
    A(Matrix7x7d::Zero(7,7)), 
    jacobian(Matrix6x7d::Zero(6,7)),
    N(Matrix7x7d::Zero(7,7)),
    lambda(Matrix6x6d::Zero(6,6)),
    kp(1), period(1),
    t_disp(1), t_last_disp(0),
    t_ee(Vector6d::Zero(6,1)),
    t_log(0.1), t_last_log(0) {

  //Add Ports
  this->addPort("JointState", port_joint_state);
  this->addPort("FriJointState", port_fri_joint_state);
  this->addPort("CartesianPositionFrame", port_cart_frame);

  this->addPort("MassMatrix", port_mass_matrix);
  this->addPort("Jacobian", port_jacobian);

  this->addPort("JointPositionCommand", port_joint_pos_command);
  this->addPort("JointEffortCommand", port_joint_efforts);
  this->addPort("FriJointImpedance", port_fri_joint_impedance);
  
  this->addProperty("Filename", filename);
  
  this->addProperty("kp", kp);
  this->addProperty("period", period);
  
  offset = Vector7d::Zero(7,1);
  offset[1] =  45.0*M_PI/180.0;
  offset[3] = -90.0*M_PI/180.0;
  offset[5] =  45.0*M_PI/180.0;
}

bool Mass_matrix_test::configureHook(){

  //Set joint impedance and initialize joint efforts
  for (size_t ii(0); ii < 7; ++ii) {
     fri_joint_impedance.stiffness[ii] = 0.0;
     fri_joint_impedance.damping[ii] = 0.1;
     joint_efforts.efforts.push_back(0.0);
     joint_pos_command.positions.push_back(0.0);
  }


  return true;
}

bool Mass_matrix_test::startHook(){

  //Time start
  t_start = RTT::os::TimeService::Instance()->getTicks();
  
  port_fri_joint_impedance.write(fri_joint_impedance);

  return true;
}

void Mass_matrix_test::updateHook(){

  //Write joint impedance
  port_fri_joint_impedance.write(fri_joint_impedance);

  //Read the current time
  t_cur = RTT::os::TimeService::Instance()->getSeconds(t_start);

  //Read and update the state
  if (port_joint_state.read(joint_state) == NewData) {

     port_fri_joint_state.read(fri_joint_state);
     port_cart_frame.read(cart_frame_kdl);
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
     
     //Read actual joint torque for logging
     for (size_t ii(0); ii<7; ++ii) {
       act_torque[ii] = fri_joint_state.msrJntTrq[ii] - fri_joint_state.gravity[ii];
     }

     //Compute the operational space mass matrix assuming non-singular pose
     lambda = (jacobian*A.inverse()*jacobian.transpose()).inverse();

     //Compute the tranpose of thedynamically consistent nullspace
     N = Matrix7x7d::Identity(7,7) - jacobian.transpose()*(A.inverse()*jacobian.transpose()*lambda).transpose();

     //Compute desired torque to follow the trajectory
     for(size_t ii(0); ii<7; ++ii) {
       torque(ii) = kp*(offset[ii] + (M_PI/2)*sin(t_cur*2*M_PI/period) - joint_state.position[ii]);
     }

     //Project into the nullspace
     torque = N*torque;
     
     //Theoretical Contribution to end effector pose
     t_ee = (A.inverse()*jacobian.transpose()*lambda).transpose()*torque;

     //Write to robot
     for (size_t ii(0); ii<7; ++ii) {
       joint_efforts.efforts[ii] = torque(ii);
       joint_pos_command.positions[ii] = joint_state.position[ii];
     }
     port_joint_efforts.write(joint_efforts);
     port_joint_pos_command.write(joint_pos_command);
  }

  //Data display
  if (t_cur - t_last_disp > t_disp) {
    std::cout << "Time: " << t_cur<< std::endl;
    std::cout << "Joint Position: " ;
    for (size_t ii(0); ii<7; ++ii) {
      std::cout << joint_state.position[ii] << " ";
    }
    std::cout << std::endl;
    std::cout << "Desired Joint Torque: " << torque.transpose() << std::endl;
    
    std::cout << "Actual Joint Torque: " << act_torque.transpose() << std::endl;

    std::cout << "End Effector Frame:" << std::endl;
    for (size_t ii(0); ii<4; ++ii) {
      for (size_t jj(0); jj<4; ++jj) {
        std::cout << cart_frame_kdl(ii,jj) << " ";
      }
      std::cout << std::endl;
    }
    
    JacobiSVD<Matrix6x7d> svd(jacobian);
    std::cout << "Singular Values of J: " << svd.singularValues() << std::endl;

    std::cout << "Theoretical EE force: " << t_ee.transpose() << std::endl;

    std::cout << std::endl;
    t_last_disp = t_cur;
  }

  //Log
  if (t_cur - t_last_log > t_log) {
  
    Vector7d jpos;
    for (size_t ii(0); ii<7; ++ii) {
      jpos[ii] = joint_state.position[ii];
    }

    jpos_log.push_back(jpos);
    des_torque_log.push_back(torque);
    act_torque_log.push_back(act_torque);
    des_ee_force_log.push_back(t_ee);
    act_ee_pos_log.push_back(cart_frame_kdl);
    
    t_last_log = t_cur;
  }

}

void Mass_matrix_test::stopHook() {

  //Write zero torque
  for (size_t ii(0); ii < 7; ++ii) {
    joint_efforts.efforts[ii] = 0.0;
  }
  port_joint_efforts.write(joint_efforts);

  //Write out data
  std::ofstream f;
  f.open (filename.c_str());

  double x,y,z,w;

  for (size_t ii(0); ii < (size_t) jpos_log.size(); ++ii) {
    for (size_t jj(0); jj < (size_t) 7; ++jj) {
      f << jpos_log[ii][jj] << " ";
    }
    for (size_t jj(0); jj < (size_t) 7; ++jj) {
      f << des_torque_log[ii][jj] << " ";
    }
    for (size_t jj(0); jj < (size_t) 7; ++jj) {
      f << act_torque_log[ii][jj] << " ";
    }
    for (size_t jj(0); jj < (size_t) 6; ++jj) {
      f << des_ee_force_log[ii][jj] << " ";
    }
    f << act_ee_pos_log[ii].p.x() << " " << act_ee_pos_log[ii].p.y() << " "<< act_ee_pos_log[ii].p.z() << " ";
    act_ee_pos_log[ii].M.GetQuaternion(x,y,z,w);
    f << x << " " << y << " " << z << " " << w;
    f << "\n";
  }
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
