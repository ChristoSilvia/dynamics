#!/usr/bin/env python
# python imports
import numpy as np
import scipy.integrate 
import matplotlib.pyplot as plt

# ROS imports
import rospy

# baxter imports
import baxter_interface
import baxter_pykdl

class Integrator:
  def __init__(self, limb_name='left'):
    # initialize ros node
    rospy.init_node('integrator')
    rospy.loginfo("Initialized node Integrator")

    # initialize baxter interface with limb
    self.limb_name = limb_name
    self.limb = baxter_interface.Limb(self.limb_name)
    self.joint_names = self.limb.joint_names()
    rospy.loginfo("Initialized limb interface")

    # setup limb kinematics
    self.limb_kinematics = baxter_pykdl.baxter_kinematics(self.limb_name)
    
    # params
    self.eps = 1e-7

    rospy.loginfo("Getting Initial Conditions")
    rospy.loginfo("Initial Joint Positions")
    q_init = self.make_vec(self.limb.joint_angles)
    print q_init
    rospy.loginfo("Getting Joint Velocities")
    q_dot_init = self.make_vec(self.limb.joint_velocities)
    print q_dot_init 
    rospy.loginfo("Joint Momentums:")
    p_init = np.dot(np.array(self.limb_kinematics.inertia()), q_dot_init)
    print p_init
    x_init = np.zeros(14)
    x_init[:7] = q_init
    x_init[7:] = p_init
    print x_init


    T = np.linspace(0,4,1000)
    Q = scipy.integrate.odeint(self.f, x_init, T)
    for i in xrange(7):
      plt.plot(T,Q[:,i],label=self.joint_names[i])
    plt.legend()
    plt.show()



  def f(self, x, t):
    x_dot = np.empty(14)

    joint_dict = self.make_dict(x[:7])

    I = np.array(self.limb_kinematics.inertia(joint_values = joint_dict))
    x_dot[:7] = np.linalg.solve(I,x[7:])
    for i in xrange(7):
      delta_joint_dict = joint_dict
      delta_joint_dict[self.joint_names[i]] += self.eps
      dI_dqi = (np.array(self.limb_kinematics.inertia(joint_values = delta_joint_dict)) - I)/self.eps
      x_dot[7+i] = 0.5 * np.dot(x_dot[:7].T, np.dot(dI_dqi, x_dot[:7]))

    print("Energy: {0}".format(0.5*np.dot(x_dot[:7].T,np.dot(I, x_dot[:7]))))
    return x_dot

  def make_dict(self, attributes):
    return_dict = {}
    for attribute, name in zip(attributes, self.joint_names):
      return_dict[name] = attribute
    return return_dict

  def make_vec(self, getter_function):
    joint_angles_dict = getter_function()
    return np.array([ joint_angles_dict[joint_name] for joint_name in self.joint_names])



if __name__ == '__main__':
  try:
    integrator = Integrator()
  except rospy.ROSInterruptException:
    pass
