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
   RTC::RETURN_VALUE planPath(const RTC::OGMap & map,const RTC::TimedPose2D & currentPose,const RTC::TimedPose2D & targetGoal ,RTC::Path2D_out path);
   
   	mrpt::poses::CPose2D getStart(){return start;}
	mrpt::poses::CPose2D getGoal(){return goal;}

	void setStart(const RTC::TimedPose2D & tp, const RTC::OGMap & map){
		start.x(map.map.column + tp.data.position.x / map.config.yScale);
		start.y(map.map.row + tp.data.position.y / map.config.yScale);
		start.phi(tp.data.heading);
	}
	void setGoal(const RTC::TimedPose2D & tp, const RTC::OGMap & map){
		goal.x(map.map.column + tp.data.position.x / map.config.xScale);
		goal.y(map.map.row + tp.data.position.y / map.config.yScale);
		goal.phi(tp.data.heading);
	}
	void OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap);

};

#endif // MOBILEROBOTSVC_IMPL_H


