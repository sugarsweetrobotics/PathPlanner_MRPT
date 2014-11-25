// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"
#include <iostream>

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>
#include <mrpt/poses/CPose2D.h>


using namespace mrpt;
using namespace mrpt::slam;
using namespace std;


// End of example implementational code

/*
 * Example implementational code for IDL interface RTC::PathPlanner
 */


PathPlannerSVC_impl::PathPlannerSVC_impl()
{
	CPose2D start(0,0,0);
	CPose2D goal(0,0,0);
}


PathPlannerSVC_impl::~PathPlannerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */



RTC::RETURN_VALUE PathPlannerSVC_impl::planPath(const RTC::OGMap & map,const RTC::TimedPose2D & currentPose,const RTC::TimedPose2D & targetGoal ,RTC::Path2D_out path)
{
	RTC::RETURN_VALUE result;
	
	CPathPlanningCircularRobot pathPlanning;

	COccupancyGridMap2D gridmap;	
	//OGap -> COccupancyGridMap2Dに変換するコードを書く

	//TimedPose2d -> CPose2D 
	setStart(currentPose);
	setGoal(targetGoal);

	bool notFound = false;
	std::deque <TPoint2D> tPath;
		
	//plan path
	pathPlanning.computePath(gridmap, getStart(), getGoal(), tPath, notFound, 100.0f);

	//deque <TPoint2D>  -> Path2Dに変換するコードを書く

  return result;
}

// End of example implementational code



