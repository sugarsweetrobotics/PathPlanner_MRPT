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

void PathPlannerSVC_impl::OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap) {

	gridmap->setSize(0-ogmap.config.width/2, ogmap.config.width/2, 0-ogmap.config.height/2, ogmap.config.height/2, ogmap.config.xScale, 0.5f);
	for(int i=0; i<gridmap->getSizeX(); i++){
		for(int j=0; j<gridmap->getSizeY(); j++){	
			gridmap->setCell( i, j, ogmap.map.cells[i,j]);
		}
	}

	/*
	ogmap.config.width = map.getWidth();
	ogmap.config.height = map.getHeight();
	ogmap.config.xScale = map.getResolution();
	ogmap.config.yScale = map.getResolution();
	ogmap.config.origin.position.x = -map.getOriginX() * map.getResolution();
	ogmap.config.origin.position.y = -map.getOriginY() * map.getResolution();
	ogmap.config.origin.heading = 0.0;
	ogmap.map.width = map.getWidth();
	ogmap.map.height = map.getHeight();
	ogmap.map.row = map.getOriginX();
	ogmap.map.column = map.getOriginY();
	ogmap.map.cells.length(map.getWidth() * map.getHeight());
	for(uint32_t i = 0;i < map.getHeight();i++) {
		for(uint32_t j = 0;j < map.getWidth();j++) {
			ogmap.cells[(i)*map.getWidth() + (map.getWidth()-1-j)] = map.getCell(i, j);
		}
	}
	*/
}


RTC::RETURN_VALUE PathPlannerSVC_impl::planPath(const RTC::OGMap & map,const RTC::TimedPose2D & currentPose,const RTC::TimedPose2D & targetGoal ,RTC::Path2D_out path)
{
	RTC::RETURN_VALUE result;
	
	CPathPlanningCircularRobot pathPlanning;

	COccupancyGridMap2D gridmap;	
	OGMapToCOccupancyGridMap(map, &gridmap);
	

	//TimedPose2d -> CPose2D 
	setStart(currentPose);
	setGoal(targetGoal);

	bool notFound = false;
	std::deque <TPoint2D> tPath;

	pathPlanning.robotRadius = 0.30f;
		
	//plan path
	pathPlanning.computePath(gridmap, getStart(), getGoal(), tPath, notFound, 100.0f);


	//deque <TPoint2D>  -> Path2D_out‚É•ÏŠ·‚·‚é
	path = new RTC::Path2D(); //‚à‚µ‚©‚µ‚½‚çpath = new RTC::Path2D_out.ptr();‚©‚àH
	path->waypoints.length(tPath.size());
	for(int i = 0;i < tPath.size(); i++) {
		path->waypoints[i].target.position.x = tPath[i].x;
		path->waypoints[i].target.position.y = tPath[i].y;
	}

	std::cout<< path << endl;

  return result;
}

// End of example implementational code



