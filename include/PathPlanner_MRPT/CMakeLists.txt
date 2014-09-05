set(hdrs PathPlanner_MRPT.h
    PARENT_SCOPE
    MobileRobotSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

