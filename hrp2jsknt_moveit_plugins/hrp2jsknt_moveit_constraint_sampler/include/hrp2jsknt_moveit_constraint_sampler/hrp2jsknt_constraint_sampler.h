/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Dave Coleman */

#ifndef HRP2JSKNT_MOVEIT_PLUGINS__HRP2JSKNT_CONSTRAINT_SAMPLER_
#define HRP2JSKNT_MOVEIT_PLUGINS__HRP2JSKNT_CONSTRAINT_SAMPLER_

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <random_numbers/random_numbers.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// debug: for publishing to rviz:
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

namespace hrp2jsknt_moveit_constraint_sampler
{

/**
 * \brief HRP2JSKNTConstraintSampler is a class that allows the sampling
 * of joints in a particular group of the robot, subject to a set of individual joint constraints.
 *
 * The set of individual joint constraint reduce the allowable bounds
 * used in the sampling.  Unconstrained values will be sampled within
 * their limits.
 *
 */
class HRP2JSKNTConstraintSampler : public constraint_samplers::ConstraintSampler
{
public:

  /**
   * Constructor
   *
   * @param [in] scene The planning scene used to check the constraint
   *
   * @param [in] group_name The group name associated with the
   * constraint.  Will be invalid if no group name is passed in or the
   * joint model group cannot be found in the kinematic model
   *
   */
  HRP2JSKNTConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
    const std::string &group_name) :
    ConstraintSampler(scene, group_name)
  {
    logInform("constructing HRP2JSKNTConstraintSampler");
  }
  /**
   * \brief Configures a joint constraint given a Constraints message.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest maximum value).  For the configuration to be
   * successful, the following condition must be met, in addition to
   * the conditions specified in \ref configure(const std::vector<kinematic_constraints::JointConstraint> &jc) :

   * \li The Constraints message must contain one or more valid joint
   * constraints (where validity is judged by the ability to configure
   * a \ref JointConstraint)
   *
   * @param [in] constr The message containing the constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  virtual bool configure(const moveit_msgs::Constraints &constr);

  /**
   * \brief Configures a joint constraint given a vector of constraints.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest_maximum value.  For the configuration to be
   * successful, the following conditions must be met:

   * \li The vector must contain one or more valid, enabled joint
   * constraints
   *
   * \li At least one constraint must reference a joint in the
   * indicated group.  If no additional bounds exist for this group,
   * then RobotState::setToRandomPositions() can be
   * used to generate a sample independently from the
   * constraint_samplers infrastructure.
   *
   * \li The constraints must allow a sampleable region for all
   * joints, where the most restrictive minimum bound is less than the
   * most restrictive maximum bound
   *
   * @param [in] jc The vector of joint constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  bool configure(const std::vector<kinematic_constraints::JointConstraint> &jc);

  virtual bool sample(robot_state::RobotState &state,
    const robot_state::RobotState &ks,
    unsigned int max_attempts);

  bool sampleJoints(robot_state::RobotState &state);

  bool sampleOrientationConstraints(robot_state::RobotState &state);

  /**
   * \brief Use two IK solvers to find the location of the robot's legs
   * \param state - the robot state as it has so far been sampled (upper body and torso)
   * \param max_attempts - number of times to run IK before giving up
   * \return true if it finds a valid location for both legs
   */
  bool calculateLegJoints(robot_state::RobotState &state, const robot_state::RobotState &reference_state, unsigned int max_attempts);

  virtual bool project(robot_state::RobotState &state,
    unsigned int max_attempts);

  /**
   * \brief Gets the number of constrained joints - joints that have an
   * additional bound beyond the joint limits.
   *
   *
   * @return The number of constrained joints.
   */
  std::size_t getConstrainedJointCount() const
  {
    return bounds_.size();
  }

  /**
   * \brief Gets the number of unconstrained joints - joint that have
   * no additional bound beyond the joint limits.
   *
   * @return The number of unconstrained joints.
   */
  std::size_t getUnconstrainedJointCount() const
  {
    return unbounded_.size();
  }

  /**
   * \brief Get the name of the constraint sampler, for debugging purposes
   * should be in CamelCase format.
   * \return string of name
   */
  virtual const std::string& getName() const
  {
    static const std::string SAMPLER_NAME = "HRP2JSKNTConstraintSampler";
    return SAMPLER_NAME;
  }

  void printVirtualJointPosition(const robot_state::RobotState &robot_state)
  {
    logInform("Virtual Joint Positions:");
    const double* positions = robot_state.getJointPositions("virtual_joint");
    std::cout << "X: " << positions[0] << std::endl;
    std::cout << "Y: " << positions[1] << std::endl;
    std::cout << "Z: " << positions[2] << std::endl;
    std::cout << "Qx: " << positions[3] << std::endl;
    std::cout << "Qy: " << positions[4] << std::endl;
    std::cout << "QZ: " << positions[5] << std::endl;
    std::cout << "Qw: " << positions[6] << std::endl;
  }

private:
  void displayRobotState(const robot_state::RobotState &robot_state);

  ros::Publisher robot_state_publisher_;
  moveit_msgs::DisplayRobotState display_robot_msg_;
  
protected:

  /// \brief An internal structure used for maintaining constraints on a particular joint
  struct JointInfo
  {
    /**
     * \brief Constructor
     *
     * @return
     */
    JointInfo()
    {
      min_bound_ = -std::numeric_limits<double>::max();
      max_bound_ = std::numeric_limits<double>::max();
    }

    /**
     * \brief Function that adjusts the joints only if they are more
     * restrictive.  This means that the min limit is higher than the
     * current limit, or the max limit is lower than the current max
     * limit.
     *
     * @param min The min limit for potential adjustment
     * @param max The max limit for potential adjustment
     */
    void potentiallyAdjustMinMaxBounds(double min, double max)
    {
      min_bound_ = std::max(min, min_bound_);
      max_bound_ = std::min(max, max_bound_);
    }

    double min_bound_;          /**< The most restrictive min value of those set */
    double max_bound_;          /**< The most restrictive max value of those set */
    std::size_t index_;         /**< The index within the joint state vector for this joint */
  };

  virtual void clear();

  random_numbers::RandomNumberGenerator           random_number_generator_; /**< \brief Random number generator used to sample */
  std::vector<JointInfo>                          bounds_; /**< \brief The bounds for any joint with bounds that are more restrictive than the joint limits */

  std::vector<const robot_model::JointModel*> unbounded_; /**< \brief The joints that are not bounded except by joint limits */
  std::vector<unsigned int>                       uindex_; /**< \brief The index of the unbounded joints in the joint state vector */
  std::vector<double>                             values_; /**< \brief Values associated with this group to avoid continuously reallocating */

  std::string sampler_name_; // used for debugging

  boost::shared_ptr<kinematic_constraints::OrientationConstraint> orientation_constraint_; /**< \brief Holds the orientation constraint for sampling */

  // Leg IK solvers
  const robot_model::JointModelGroup* left_leg_;
  const robot_model::JointModelGroup* right_leg_;

  // Store desired feet positions
  Eigen::Affine3d left_foot_position_;
  Eigen::Affine3d right_foot_position_;

  // Allocate memory for storing transforms of feet
  Eigen::Affine3d left_foot_position_new_;
  Eigen::Affine3d right_foot_position_new_;
};


// define the sampler allocator plugin interface
class HRP2JSKNTConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:

  virtual constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr &scene,
    const std::string &group_name, const moveit_msgs::Constraints &constr)
  {
    constraint_samplers::ConstraintSamplerPtr cs(new HRP2JSKNTConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
  }

  virtual bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
    const moveit_msgs::Constraints &constr) const
  {
    logInform("hrp2jsknt_constraint_sampler: checking constraint sampler %s", group_name.c_str());
    std::cerr << constr << std::endl;
    // do not use this sampler if there are any joint constraints, because then we are in the goal sampling stage
#if 0
    if (group_name == "whole_body")
      {
        if (constr.joint_constraints.size() == 0)
          {
            logInform("hrp2jsknt_constraint_sampler: Using custom constraint sampler");
            return true;
          }
      }
#endif
#if 1
    if (group_name == "whole_body")
      {
        if (constr.joint_constraints.size() == 0)
          {
            logInform("hrp2jsknt_constraint_sampler: Using custom constraint sampler");
            return true;
          }
        bool virtual_joint_flag = true;
        for (size_t i = 0; i < constr.joint_constraints.size(); i++) {
          if (constr.joint_constraints[i].joint_name.find("/trans_") == std::string::npos &&
              constr.joint_constraints[i].joint_name.find("/rot_") == std::string::npos ) {
            virtual_joint_flag = false;
            break;
          }
        }
        if (virtual_joint_flag) {
          logInform("hrp2jsknt_constraint_sampler: Using custom constraint sampler (only contain virtual joint)");
          return true;
        }
      }
#endif
    logInform("hrp2jsknt_constraint_sampler: Not using custom constraint sampler");
    return false;
  }

};

} // namespace

PLUGINLIB_EXPORT_CLASS(hrp2jsknt_moveit_constraint_sampler::HRP2JSKNTConstraintSamplerAllocator,
  constraint_samplers::ConstraintSamplerAllocator);


#endif
