#include <talos_wbc_actions/icp_control_utils.h>
#include <pal_geometry_tools/wykobi.hpp>
#include <math_utils/geometry_tools.h>

namespace pal_locomotion
{
void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped)
{
  control(bc, rate_limiter, targetDCM, targetDCM_vel, referenceCOP, use_rate_limited_dcm,
          bc->getActualCOMPosition(), bc->getActualCOMVelocity2d(),
          targetCOP_rate_limited_unclamped, targetCOP_unclamped);
}
void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm,
             const eVector3 &actualCOM, const eVector2 &actualCOM_vel,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped)
{
  Eigen::Vector2d actualCOM_2d(actualCOM.x(), actualCOM.y());
  Eigen::Vector2d actualCOM_vel_2d(actualCOM_vel.x(), actualCOM_vel.y());

  std::vector<eVector3> local_foot_description;
  double foot_length = bc->getParameters()->foot_description_.foot_length_;
  double foot_with = bc->getParameters()->foot_description_.foot_with_;

  local_foot_description.push_back(eVector3(foot_length / 2, foot_with / 2, 0.));
  local_foot_description.push_back(eVector3(foot_length / 2, -foot_with / 2, 0.));
  local_foot_description.push_back(eVector3(-foot_length / 2, foot_with / 2, 0.));
  local_foot_description.push_back(eVector3(-foot_length / 2, -foot_with / 2, 0.));

  std::vector<eVector3> foot_description;
  auto stance_feet = bc->getStanceLegIDs();
  for (size_t j = 0; j < stance_feet.size(); j++)
  {
    eMatrixHom pose = bc->getActualFootPose(stance_feet[j]);
    for (size_t i = 0; i < 4; ++i)
    {
      foot_description.push_back(pose * local_foot_description[i]);
    }
  }

  std::vector<eVector2> foot_description_2d(foot_description.size());
  pal::convert(foot_description, foot_description_2d);

  std::vector<eVector2> convexHullPoints;
  wykobi::polygon<double, 2> convex_hull;
  createConvexHull(foot_description_2d, convexHullPoints, convex_hull);

  // Compute control law
  double w = sqrt(bc->getParameters()->gravity_ / bc->getParameters()->z_height_);

  eVector2 targetDCM2D(targetDCM.x(), targetDCM.y());
  eVector2 targetDCM_vel2D(targetDCM_vel.x(), targetDCM_vel.y());

  Eigen::Vector2d actualDCM = actualCOM_2d + actualCOM_vel_2d / w;

  targetCOP_rate_limited_unclamped =
      actualDCM +
      rate_limiter->onlineFiltering(bc->getParameters()->icp_gain_ * (actualDCM - targetDCM2D)) -
      (1. / w) * targetDCM_vel2D;

  targetCOP_unclamped = actualDCM + bc->getParameters()->icp_gain_ * (actualDCM - targetDCM2D) -
                        (1. / w) * targetDCM_vel2D;

  Eigen::Vector2d targetCOP_clamped;
  if (use_rate_limited_dcm)
  {
    bool isInsideSupportPolygon =
        isPointInPolygon(targetCOP_rate_limited_unclamped, convex_hull);
    if (!isInsideSupportPolygon)
    {
      targetCOP_clamped =
          projectoPointClosestPointPolygon(targetCOP_rate_limited_unclamped, convex_hull);
    }
    else
    {
      targetCOP_clamped = targetCOP_rate_limited_unclamped;
    }
  }
  else
  {
    bool isInsideSupportPolygon = isPointInPolygon(targetCOP_unclamped, convex_hull);
    if (!isInsideSupportPolygon)
    {
      targetCOP_clamped = projectoPointClosestPointPolygon(targetCOP_unclamped, convex_hull);
    }
    else
    {
      targetCOP_clamped = targetCOP_unclamped;
    }
  }

  Eigen::Vector2d desiredCOM;
  Eigen::Vector2d desiredCOMd;
  Eigen::Vector2d desiredCOMdd;

  desiredCOMdd = pow(w, 2) * (actualCOM_2d - targetCOP_clamped);

  ros::Duration dt = bc->getControllerDt();
  desiredCOM =
      actualCOM_2d + actualCOM_vel_2d * dt.toSec() + desiredCOMdd * pow(dt.toSec(), 2) * 0.5;
  desiredCOMd = actualCOM_vel_2d + desiredCOMdd * dt.toSec();

  bc->setDesiredCOMPosition(eVector3(desiredCOM.x(), desiredCOM.y(), targetDCM.z()));
  bc->setDesiredCOMVelocity(eVector3(desiredCOMd.x(), desiredCOMd.x(), targetDCM_vel.z()));
  bc->setDesiredCOMAcceleration(eVector3(desiredCOMdd.x(), desiredCOMdd.y(), 0.));

  // Change back to global coordinates
  bc->setDesiredICP(eVector3(targetDCM.x(), targetDCM.y(), 0.));
  bc->setDesiredCOPReference(eVector3(referenceCOP.x(), referenceCOP.y(), 0.));
  bc->setDesiredCOPComputed(eVector3(targetCOP_clamped.x(), targetCOP_clamped.y(), 0.));
  bc->setActualICP(eVector3(actualDCM.x(), actualDCM.y(), 0.));  // ICP aka DCM
}

CubicInterpolator::CubicInterpolator(const ros::Time &start_time, const ros::Time &end_time, const eVector3 &start_pos, const eVector3 &end_pos, 
                    const eVector3 &start_vel, const eVector3 &end_vel,
                    const eVector3 &start_acc, const eVector3 &end_acc): end_time_(end_time), start_time_(start_time)
{

}
void CubicInterpolator::evaluate(const eVector3 &desired_pos, const eVector3 &desired_vel, const eVector3 &desired_acc)
{


}



} // namespace
