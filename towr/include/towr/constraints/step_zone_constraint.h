// Copyright (c) 2023 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by suyoung on 12/11/23.
//

#ifndef RAISIM_TOWR_STEP_ZONE_CONSTRAINT_H
#define RAISIM_TOWR_STEP_ZONE_CONSTRAINT_H

#include <ifopt/constraint_set.h>

#include <towr/variables/nodes_variables_phase_based.h>
#include <towr/variables/cartesian_dimensions.h>

namespace towr
{

class StepZoneConstraint : public ifopt::ConstraintSet
{
public:
  using Vector2d = Eigen::Vector2d;
  using Vector3d = Eigen::Vector3d;

  StepZoneConstraint (const double &stepZoneSize,
                      const std::vector<Vector2d> &stepCenters,
                      const std::vector<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> &stepHalfSpaces,
                      const std::string &ee_motion);
  ~StepZoneConstraint () override = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;

  VectorXd GetValues () const override;
  VecBound GetBounds () const override;
  void FillJacobianBlock (std::string var_set, Jacobian &jac) const override;

private:
  [[nodiscard]] int GetClosestStepIdx (const Vector2d &p) const;
  [[nodiscard]] bool isInStepZone (const Vector2d &p) const;
  NodesVariablesPhaseBased::Ptr ee_motion_;

  int num_half_spaces_per_sample_;
  double step_zone_size_;
  std::vector<Vector2d> step_centers_;
  std::vector<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> step_half_spaces_;

  std::string ee_motion_id_;
  std::vector<int> node_ids_;
};

} // namespace towr

#endif //RAISIM_TOWR_STEP_ZONE_CONSTRAINT_H
