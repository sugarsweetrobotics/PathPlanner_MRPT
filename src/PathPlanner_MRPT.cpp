// -*- C++ -*-
/*!
 * @file  PathPlanner_MRPT.cpp
 * @brief Path Planner MRPT component
 * @date $Date$
 *
 * $Id$
 */

#include "PathPlanner_MRPT.h"

// Module specification
// <rtc-template block="module_spec">
static const char* pathplanner_mrpt_spec[] =
  {
    "implementation_id", "PathPlanner_MRPT",
    "type_name",         "PathPlanner_MRPT",
    "description",       "Path Planner MRPT component",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigatio",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    // Widget
    "conf.__widget__.debug", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PathPlanner_MRPT::PathPlanner_MRPT(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pathPlannerPort("pathPlanner")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PathPlanner_MRPT::~PathPlanner_MRPT()
{
}



RTC::ReturnCode_t PathPlanner_MRPT::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_pathPlannerPort.registerProvider("PathPlanner", "RTC::PathPlanner", m_pathPlanner);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_pathPlannerPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PathPlanner_MRPT::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PathPlanner_MRPT::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PathPlanner_MRPT::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PathPlanner_MRPT::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PathPlanner_MRPT::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_MRPT::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PathPlanner_MRPTInit(RTC::Manager* manager)
  {
    coil::Properties profile(pathplanner_mrpt_spec);
    manager->registerFactory(profile,
                             RTC::Create<PathPlanner_MRPT>,
                             RTC::Delete<PathPlanner_MRPT>);
  }
  
};


