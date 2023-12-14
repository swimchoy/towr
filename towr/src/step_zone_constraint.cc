// Copyright (c) 2023 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by suyoung on 12/11/23.
//

#include <towr/constraints/step_zone_constraint.h>

namespace towr
{

StepZoneConstraint::StepZoneConstraint (const std::vector<Vector2d> &points,
                                        const std::vector<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> &stepHalfSpaces,
                                        const std::string &ee_motion)
    :ConstraintSet(kSpecifyLater, "step_zone-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
  step_centers_ = points;
  step_half_spaces_ = stepHalfSpaces;
  num_half_spaces_per_sample_ = static_cast<int>(step_half_spaces_.back().second.rows());
}

void
StepZoneConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_id_);

  // skip first node, b/c already constrained by initial stance
  for (int id=1; id<ee_motion_->GetNodes().size(); ++id)
    node_ids_.push_back(id);

  int num_nodes = node_ids_.size();
  SetRows(num_nodes * num_half_spaces_per_sample_);
}

Eigen::VectorXd
StepZoneConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector2d p = nodes.at(id).p().head(2);
    int idx = GetClosestStepIdx(p);

    g.segment(row, num_half_spaces_per_sample_) =
        step_half_spaces_[idx].first * p - step_half_spaces_[idx].second;
    row += num_half_spaces_per_sample_;
  }
  return g;
}

StepZoneConstraint::VecBound
StepZoneConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  int row = 0;
  for (int id : node_ids_) {
    if (ee_motion_->IsConstantNode(id))
      bounds.at(row) = ifopt::BoundSmallerZero;
    else
      bounds.at(row) = ifopt::NoBound;
    row += num_half_spaces_per_sample_;
  }

  return bounds;
}

void
StepZoneConstraint::FillJacobianBlock (std::string var_set, Jacobian &jac) const
{
  if (var_set == ee_motion_->GetName()) {
    auto nodes = ee_motion_->GetNodes();
    int row = 0;
    for (int id : node_ids_) {
      Vector2d p = nodes.at(id).p().head(2);
      int idx = GetClosestStepIdx(p);
      int x = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, X));
      int y = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Y));
      int z = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Z));

      for (int i = 0; i < num_half_spaces_per_sample_; ++i) {
        jac.coeffRef(row + i, x) = step_half_spaces_[idx].first(i, 0);
        jac.coeffRef(row + i, y) = step_half_spaces_[idx].first(i, 1);
        jac.coeffRef(row + i, z) = 0.0;
      }
      row += num_half_spaces_per_sample_;
    }
  }
}

int
StepZoneConstraint::GetClosestStepIdx (const Vector2d &p) const
{
  auto it = std::min_element(step_centers_.begin(), step_centers_.end(),
                             [&](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
                               return (a - p).norm() < (b - p).norm();
                             });
  return static_cast<int>(std::distance(step_centers_.begin(), it));
}

} // namespace towr
