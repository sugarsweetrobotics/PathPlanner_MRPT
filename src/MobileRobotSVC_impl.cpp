// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"
#include <iostream>

//#include<fstream>

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>
#include <mrpt/poses/CPose2D.h>


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

void PathPlannerSVC_impl::OGMapToCOccupancyGridMap(RTC::OGMap ogmap, COccupancyGridMap2D *gridmap) {
	//gridmap->setSize(0, ogmap.map.width, 0, ogmap.map.height, ogmap.config.xScale, 0.5f);
	gridmap->setSize(0, ogmap.map.width, 0, ogmap.map.height, 1, 0.5f);
	int height = gridmap->getSizeY();
	int width =  gridmap->getSizeX();
	
	  ofstream ofs( "filename.csv");


	for(int i=0; i <height ; i++){
		for(int j=0; j <width ; j++){
			int index = i * width + j;
			int cell = ogmap.map.cells[index];
	
			if(cell < 100){
				gridmap->setCell(j, i, 0.0);
				//cout<<"occupy  "<< i <<"  "<< j <<"  "<<gridmap->getCell(i,j)<<endl;
				//ofs<<gridmap->getCell(i,j)<<endl;
			}
			else if(cell > 200){
				gridmap->setCell(j, i, 1.0);
				//cout<<"empty   "<< i <<"  "<< j <<"  "<<gridmap->getCell(i,j)<<endl;
			//	ofs<<gridmap->getCell(i,j)<<endl;
			}
			else{
				//gridmap->setCell(i, j, 0.5);
		//		cout<<"gray    "<< i <<"  "<< j <<"  "<<gridmap->getCell(i,j)<<endl;
			//	ofs<<gridmap->getCell(i,j)<<endl;
			}
		}
	}
}


RTC::RETURN_VALUE PathPlannerSVC_impl::planPath(const RTC::OGMap & map,const RTC::TimedPose2D & currentPose,const RTC::TimedPose2D & targetGoal ,RTC::Path2D_out path)
{
	RETURN_VALUE result = RETVAL_OK;
	cout << "Start Path Planning... " << endl;


	CPathPlanningCircularRobot pathPlanning;
	pathPlanning.robotRadius = 0.30f;//robot radius should be able to change by configuration
		
	COccupancyGridMap2D gridmap;
	OGMapToCOccupancyGridMap(map, &gridmap);
	//test
	gridmap.saveAsBitmapFile("map.png");

	//TimedPose2d -> CPose2D	
	setStart(currentPose);
	setGoal(targetGoal);

	cout << "Origin: " << getStart() << endl;
	cout << "Target: " << getGoal() << endl;

	bool notFound = false;
	std::deque <TPoint2D> tPath;
		
	//plan path
	pathPlanning.computePath(gridmap, getStart(), getGoal(), tPath, notFound, -1);//maxSearchPathLength should be able to change by configuration
	if(notFound){
		cout << "No path was founded"<<endl;
		cout <<endl;
		//return RETVAL_NOTFOUND_PATH;
		return result;
	}
	else{
		//deque <TPoint2D>  -> Path2D_out
		path = new RTC::Path2D(); //‚à‚µ‚©‚µ‚½‚çpath = new RTC::Path2D_out.ptr();‚©‚àH
		path->waypoints.length(tPath.size());
		for(int i = 0;i < tPath.size(); i++) {
			path->waypoints[i].target.position.x = tPath[i].x;
			path->waypoints[i].target.position.y = tPath[i].y;
		}

		std::cout << "path length:"<< path->waypoints.length() << endl;
		cout <<endl;
		return result;
	}
}

// End of example implementational code



