set(MOVEIT_LIB_NAME staro_moveit_constraint_sampler)

add_library(${MOVEIT_LIB_NAME} src/staro_constraint_sampler.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
