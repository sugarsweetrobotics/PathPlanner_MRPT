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


#include "PathPlanner_MRPT.h"

using namespace mrpt;
using namespace mrpt::slam;
using namespace std;
using namespace RTC;

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
void PathPlannerSVC_impl::setStart(const RTC::Pose2D & tp, const RTC::OGMap & map){
	start.x(tp.position.x);
	start.y(tp.position.y);
	start.phi(tp.heading);
	}
void PathPlannerSVC_impl::setGoal(const RTC::Pose2D & tp, const RTC::OGMap & map){
	goal.x(tp.position.x);
	goal.y(tp.position.y);
	goal.phi(tp.heading);
	}

void PathPlannerSVC_impl::OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap) {
	gridmap->setSize(
		-ogmap.config.origin.position.x,
		ogmap.map.width * ogmap.config.xScale - ogmap.config.origin.position.x,
		-(ogmap.map.height * ogmap.config.yScale + ogmap.config.origin.position.y),
		-ogmap.config.origin.position.y,
		ogmap.config.xScale);
		/*
		0-ogmap.map.width*ogmap.config.xScale/2,
		ogmap.map.width*ogmap.config.xScale/2,
		0-ogmap.map.height*ogmap.config.yScale/2,
		ogmap.map.height*ogmap.config.yScale/2,
		ogmap.config.xScale);*/


	int height = gridmap->getSizeY();
	int width =  gridmap->getSizeX();


	for(int i=0; i <height ; i++){
		for(int j=0; j <width ; j++){
			int cell = ogmap.map.cells[(height -i -1) * width + j];
	
			if(cell < 100){
				gridmap->setCell(j, i, 0.0);
			}
			else if(cell > 200){
				gridmap->setCell(j, i, 1.0);
			}
			else{
				gridmap->setCell(j, i, 1.0);
			}
		}
	}
	gridmap->saveAsBitmapFile("C:\\testout.bmp");
}
RTC::RETURN_VALUE PathPlannerSVC_impl::planPath(const RTC::PathPlanParameter& param, RTC::Path2D_out outPath)
{
	RETURN_VALUE result = RETVAL_OK;
	cout << "Start Path Planning..." << endl;
	cout << "  Start: " << param.currentPose.position.x <<","<<param.currentPose.position.y << endl;
	cout << "  Goal: " << param.targetPose.position.x <<","<<param.targetPose.position.y << endl;

	//TimedPose2d -> CPose2D	
	setStart(param.currentPose, param.map);
	setGoal(param.targetPose, param.map);
	
	//OGMap -> COccupancyGridMap2D
	COccupancyGridMap2D gridmap;
	OGMapToCOccupancyGridMap(param.map, &gridmap);
			
	//Plan path
	CPathPlanningCircularRobot pathPlanning;
	//PlannerSimple2D pathPlanning;
	pathPlanning.robotRadius = getRadius();
	bool notFound = true;
	std::deque <TPoint2D> tPath;

	pathPlanning.robotRadius = m_pRTC->m_robotRadius;
	pathPlanning.computePath(gridmap, getStart(), getGoal(), tPath, notFound, this->getPathLength());
	
	//Any path was not found
	if(notFound){
		cout << "No path was founded"<<endl;
		cout <<endl;
		outPath = new RTC::Path2D();
		
		result = RETVAL_INVALID_PARAMETER;
	}

	//deque <TPoint2D>  -> Path2D_out
	else{

		double distanceTolerance = m_pRTC->getPathDistnaceTolerance() < param.distanceTolerance ? m_pRTC->getPathDistnaceTolerance() : param.distanceTolerance;
		double angularTolerance = m_pRTC->getPathHeadingTolerance() < param.headingTolerance ? m_pRTC->getPathHeadingTolerance() : param.headingTolerance;
		double maxSpeedX = param.maxSpeed.vx;
		double maxSpeedY = param.maxSpeed.vy;
		double maxSpeedTh = param.maxSpeed.va;
		double goalDistanceTolerance = m_pRTC->getGoalDistnaceTolerance() < distanceTolerance ? m_pRTC->getGoalDistnaceTolerance() : distanceTolerance;
		double goalAngularTolerance = m_pRTC->getGoalHeadingTolerance() < angularTolerance ? m_pRTC->getGoalHeadingTolerance() : angularTolerance;
		cout << "Done."<<endl;

		outPath = new RTC::Path2D(); 
		outPath->waypoints.length(tPath.size());

		for(int i = 0;i < tPath.size(); i++) {
			outPath->waypoints[i].target.position.x = tPath[i].x ;
			outPath->waypoints[i].target.position.y = tPath[i].y ;
			if (i == tPath.size() -1) {
				//Goal
				outPath->waypoints[i].distanceTolerance = goalDistanceTolerance;
				outPath->waypoints[i].headingTolerance  = goalAngularTolerance;
			} else {
				outPath->waypoints[i].distanceTolerance = distanceTolerance;
				outPath->waypoints[i].headingTolerance  = angularTolerance;
			}
			outPath->waypoints[i].maxSpeed.vx = maxSpeedX;
			outPath->waypoints[i].maxSpeed.vy = maxSpeedY;
			outPath->waypoints[i].maxSpeed.va = maxSpeedTh;
		}
		std::cout << "  Path length:"<< outPath->waypoints.length() << endl;
		cout <<endl;
	}
	return result;
}









// End of example implementational code



