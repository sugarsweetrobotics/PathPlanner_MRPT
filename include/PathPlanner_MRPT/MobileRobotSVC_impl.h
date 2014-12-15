// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.h
 * @brief Service implementation header of MobileRobot.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSkel.h"

#ifndef MOBILEROBOTSVC_IMPL_H
#define MOBILEROBOTSVC_IMPL_H
 
#include <iostream>

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace std;


/*!
 * @class PathPlannerSVC_impl
 * Example class implementing IDL interface RTC::PathPlanner
 */
class PathPlannerSVC_impl
 : public virtual POA_RTC::PathPlanner,
   public virtual PortableServer::RefCountServantBase
{
 private:
	mrpt::poses::CPose2D start;
	mrpt::poses::CPose2D goal;
	float RADIUS;
	float PATH_LENGTH;

 public:
	 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * @brief standard constructor
   */
   PathPlannerSVC_impl();

  /*!
   * @brief destructor
   */
   virtual ~PathPlannerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE planPath(const RTC::PathPlanParameter & param ,RTC::Path2D_out path);
   
   	mrpt::poses::CPose2D getStart(){return start;}
	mrpt::poses::CPose2D getGoal(){return goal;}

	void setConfig(float radius, float pathLength){
		RADIUS = radius;
		PATH_LENGTH = pathLength;
	}
	float getRadius(){return RADIUS;}
	float getPathLength(){return PATH_LENGTH;}

	void setStart(const RTC::Pose2D & tp, const RTC::OGMap & map);
	void setGoal(const RTC::Pose2D & tp, const RTC::OGMap & map);
	void OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap);
};

#endif // MOBILEROBOTSVC_IMPL_H


