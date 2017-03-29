/**
 @file    contact_load_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 27, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_

#include "../constraint.h"
#include "endeffector_load.h"
#include "endeffectors_motion.h"


namespace xpp {
namespace opt {

class ContactLoadConstraint : public Constraint {
public:
  using EELoadPtr   = std::shared_ptr<EndeffectorLoad>;
  using EEMotionPtr = std::shared_ptr<EndeffectorsMotion>;

  ContactLoadConstraint ();
  virtual ~ContactLoadConstraint ();

  void Init(const EEMotionPtr&, const EELoadPtr& ee_load);
  void UpdateConstraintValues () override;
  VecBound GetBounds () const override;

private:
  EELoadPtr ee_load_;
  EEMotionPtr ee_motion_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_CONTACT_LOAD_CONSTRAINT_H_ */
