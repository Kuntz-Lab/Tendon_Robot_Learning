cmake_minimum_required(VERSION 3.5)
project(motion_learning LANGUAGES CXX VERSION 1.0.0)

option(TENDON_USE_OPENMP "Use OpenMP" ON)
option(TENDON_LINK_PYTHON_SCRIPTS "Link python scripts instead of copying" ON)
option(TENDON_USE_ROS "Use ROS libraries" ON)
option(TENDON_BUILD_TESTING "Build testing" ON)

set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING
  "Build type choices: None Debug Release RelWithDebInfo MinSizeRel tsan asan lsan msan ubsan"
  FORCE
  )

# uncomment to have build output timing for each file
# - for better metrics, use clang with -ftime-trace and put the json file into
#   Google Chrome at chrome://tracing
# - another option is to use gcc with -v -ftime-report -ftime-report-details,
#   but it's not as good as with clang and Chrome.
#set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
#set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CMAKE_COMMAND} -E time")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(TENDON_USE_ROS)
  find_package(ament_cmake)
  if(ament_cmake_FOUND)
    find_package(visualization_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
  endif()
else()
  set(ament_cmake_FOUND OFF)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl 0.5 EXACT REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams graph)
find_package(ITK REQUIRED)
find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

include(${ITK_USE_FILE})

find_package(PythonInterp 3)
find_package(PythonLibs 3)
find_package(pybind11)

if (TENDON_USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else()
    message(STATUS "OpenMP not found")
  endif()
endif()

if (CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
# Note: any one of these flags cause OMPL to crash inexplicably: -mavx -mavx2 -mfma
# Note: -ffast-math turns on -fno-math-errno, -funsafe-math-optimizations,
#       -ffinite-math-only, -fno-rounding-math, -fno-signaling-nans,
#       -fcx-limited-range and -fexcess-precision=fast
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG -mfpmath=sse -mtune=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG -mfpmath=sse -mtune=native")

option(TENDON_ENABLE_GPROF "Enable gprof" OFF)
if (TENDON_ENABLE_GPROF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
endif()

# sanitizers integrated via the suggestion from
#   http://www.stablecoder.ca/2018/02/01/analyzer-build-types.html
# could also use this CMake module
#   https://github.com/arsenm/sanitizers-cmake

# ThreadSanitizer
set(CMAKE_C_FLAGS_TSAN
    "-fsanitize=thread -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during ThreadSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_TSAN
    "-fsanitize=thread -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during ThreadSanitizer builds."
    FORCE)

# AddressSanitize
set(CMAKE_C_FLAGS_ASAN
    "-fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during AddressSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_ASAN
    "-fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during AddressSanitizer builds."
    FORCE)

# LeakSanitizer
set(CMAKE_C_FLAGS_LSAN
    "-fsanitize=leak -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C compiler during LeakSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_LSAN
    "-fsanitize=leak -fno-omit-frame-pointer -g -Ofast" #-O1"
    CACHE STRING "Flags used by the C++ compiler during LeakSanitizer builds."
    FORCE)

# MemorySanitizer
set(CMAKE_C_FLAGS_MSAN
    "-fsanitize=memory -fno-optimize-sibling-calls -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -g -Ofast" #-O2"
    CACHE STRING "Flags used by the C compiler during MemorySanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_MSAN
    "-fsanitize=memory -fno-optimize-sibling-calls -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -g -Ofast" #-O2"
    CACHE STRING "Flags used by the C++ compiler during MemorySanitizer builds."
    FORCE)

# UndefinedBehaviour
set(CMAKE_C_FLAGS_UBSAN
    "-fsanitize=undefined -fsanitize-minimal-runtime -fno-omit-frame-pointer -g -Ofast"
    CACHE STRING "Flags used by the C compiler during UndefinedBehaviourSanitizer builds."
    FORCE)
set(CMAKE_CXX_FLAGS_UBSAN
    "-fsanitize=undefined -fsanitize-minimal-runtime -fno-omit-frame-pointer -g -Ofast"
    CACHE STRING "Flags used by the C++ compiler during UndefinedBehaviourSanitizer builds."
    FORCE)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_upper)
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_${build_type_upper}: ${CMAKE_CXX_FLAGS_${build_type_upper}}")


set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src)
set(THIRD_PARTY_DIR ${SRC_DIR}/3rdparty)


# 3rd party
set(THIRD_PARTY_HEADERS)
set(NLOHMANN_HEADERS
  ${THIRD_PARTY_DIR}/nlohmann/json.hpp
  )
install(FILES ${NLOHMANN_HEADERS} DESTINATION include/nlohmann)
list(APPEND THIRD_PARTY_HEADERS ${NLOHMANN_HEADERS})
list(APPEND LIBTENDON_ALL_HEADERS ${THIRD_PARTY_HEADERS})

# util
set(UTIL_HEADERS
  ${SRC_DIR}/util/FunctionTimer.h
  ${SRC_DIR}/util/LifetimeTimer.h
  ${SRC_DIR}/util/SingleStopWatch.h
  ${SRC_DIR}/util/StopWatch.h
  ${SRC_DIR}/util/angles.h
  ${SRC_DIR}/util/eigen_ops.h
  ${SRC_DIR}/util/json_io.h
  ${SRC_DIR}/util/macros.h
  ${SRC_DIR}/util/ompl_logging.h
  ${SRC_DIR}/util/openfile_check.h
  ${SRC_DIR}/util/poly.h
  ${SRC_DIR}/util/string_ops.h
  ${SRC_DIR}/util/time_function_call.h
  ${SRC_DIR}/util/vector_ops.h
  )

# spline
set(SPLINE_HEADERS
  ${SRC_DIR}/spline/Cubic.h
  ${SRC_DIR}/spline/CubicSpline.h
  ${SRC_DIR}/spline/CubicSplineSequence.h
  )
install(FILES ${SPLINE_HEADERS} DESTINATION include/spline)
list(APPEND LIBTENDON_ALL_HEADERS ${SPLINE_HEADERS})

# cliparser
set(CLIPARSER_HEADERS ${SRC_DIR}/cliparser/CliParser.h)
install(FILES ${CLIPARSER_HEADERS} DESTINATION include/cliparser)
list(APPEND LIBTENDON_ALL_HEADERS ${CLIPARSER_HEADERS})

# csv
set(CSV_HEADERS ${SRC_DIR}/csv/Csv.h)
install(FILES ${CSV_HEADERS} DESTINATION include/csv)
list(APPEND LIBTENDON_ALL_HEADERS ${CSV_HEADERS})

# tip-controller
set(TIP_CONTROL_HEADERS
    ${SRC_DIR}/tip-control/tip_control.h
    ${SRC_DIR}/tip-control/Controller.h
    ${SRC_DIR}/tip-control/StreamingIK.h
    )
install(FILES ${TIP_CONTROL_HEADERS} DESTINATION include/tip-control)
list(APPEND LIBTENDON_ALL_HEADERS ${TIP_CONTROL_HEADERS})

# cpptoml
set(CPPTOML_HEADERS
  ${SRC_DIR}/cpptoml/cpptoml.h
  ${SRC_DIR}/cpptoml/toml_conversions.h
  )
install(FILES ${CPPTOML_HEADERS} DESTINATION include/cpptoml)
list(APPEND LIBTENDON_ALL_HEADERS ${CPPTOML_HEADERS})

# collision
set(COLLISION_HEADERS
  ${SRC_DIR}/collision/Capsule.h
  ${SRC_DIR}/collision/CapsuleSequence.h
  ${SRC_DIR}/collision/Mesh.h
  ${SRC_DIR}/collision/Point.h
  ${SRC_DIR}/collision/Sphere.h
  ${SRC_DIR}/collision/VoxelOctree.h
  ${SRC_DIR}/collision/OctomapWrap.h
  ${SRC_DIR}/collision/collision.h
  ${SRC_DIR}/collision/collision.hxx
  ${SRC_DIR}/collision/collision_primitives.h
  ${SRC_DIR}/collision/detail/TreeNode.h
  ${SRC_DIR}/collision/detail/TreeNode.hxx
  ${SRC_DIR}/collision/stl_io.h
  )
install(FILES ${COLLISION_HEADERS} DESTINATION include/collision)
list(APPEND LIBTENDON_ALL_HEADERS ${COLLISION_HEADERS})

if (ament_cmake_FOUND)
  # vistendon
  set(VISTENDON_HEADERS
    ${SRC_DIR}/vistendon/EnvironmentRvizPublisher.h
    ${SRC_DIR}/vistendon/ManualRvizMarkerArrayPublisher.h
    ${SRC_DIR}/vistendon/RvizMarkerArrayPublisher.h
    ${SRC_DIR}/vistendon/StreamingInterleaver.h
    ${SRC_DIR}/vistendon/StreamingRvizTipPublisher.h
    ${SRC_DIR}/vistendon/TendonBackboneRvizPublisher.h
    ${SRC_DIR}/vistendon/TendonRvizPublisher.h
    ${SRC_DIR}/vistendon/TrajectoryVisualizer.h
    ${SRC_DIR}/vistendon/VoxelRvizPublisher.h
    ${SRC_DIR}/vistendon/marker_array_conversions.h
    ${SRC_DIR}/vistendon/view_path.h
    )
  install(FILES ${VISTENDON_HEADERS} DESTINATION include/vistendon)
  list(APPEND LIBTENDON_ALL_HEADERS ${VISTENDON_HEADERS})
endif ()

# motion planning
set(MOTION_PLANNING_HEADERS
  ${SRC_DIR}/motion-planning/AStarGoalVisitor.h
  ${SRC_DIR}/motion-planning/AbstractValidityChecker.h
  ${SRC_DIR}/motion-planning/AbstractVoxelMotionValidator.h
  ${SRC_DIR}/motion-planning/AbstractVoxelValidityChecker.h
  ${SRC_DIR}/motion-planning/Environment.h
  ${SRC_DIR}/motion-planning/LazyPRMFixed.h
  ${SRC_DIR}/motion-planning/OctomapValidityChecker.h
  ${SRC_DIR}/motion-planning/Problem.h
  ${SRC_DIR}/motion-planning/RetractionSampler.h
  ${SRC_DIR}/motion-planning/StraightLinePlanner.h
  ${SRC_DIR}/motion-planning/StreamingPlanner.h
  ${SRC_DIR}/motion-planning/ValidityChecker.h
  ${SRC_DIR}/motion-planning/VoxelBackboneDiscreteMotionValidator.h
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidator.h
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidatorAndLogger.h
  ${SRC_DIR}/motion-planning/VoxelBackboneValidityChecker.h
  ${SRC_DIR}/motion-planning/VoxelCachedLazyPRM.h
  ${SRC_DIR}/motion-planning/VoxelEnvironment.h
  ${SRC_DIR}/motion-planning/VoxelValidityChecker.h
  ${SRC_DIR}/motion-planning/WSpaceGoal.h
  ${SRC_DIR}/motion-planning/ompl_planners.h
  ${SRC_DIR}/motion-planning/plan.h
  )
install(FILES ${MOTION_PLANNING_HEADERS} DESTINATION include/motion-planning)
list(APPEND LIBTENDON_ALL_HEADERS ${MOTION_PLANNING_HEADERS})

# design optimization
set(DESIGN_OPTIMIZATION_HEADERS
  ${SRC_DIR}/design-optimization/MultiGoalPlanningCostIK.h
  #${SRC_DIR}/design-optimization/MultiGoalPlanningCostGoalBias.h
  )
install(FILES ${DESIGN_OPTIMIZATION_HEADERS} DESTINATION include/design-optimization)
list(APPEND LIBTENDON_ALL_HEADERS ${DESIGN_OPTIMIZATION_HEADERS})

# tendon
set(CONFIG_IN ${SRC_DIR}/tendon/config.h.in)
set(CONFIG_OUT ${CMAKE_CURRENT_BINARY_DIR}/include/tendon/config.h)
configure_file("${CONFIG_IN}" "${CONFIG_OUT}")
set(TENDON_HEADERS
  ${SRC_DIR}/tendon/BackboneSpecs.h
  ${SRC_DIR}/tendon/TendonResult.h
  ${SRC_DIR}/tendon/TendonRobot.h
  ${SRC_DIR}/tendon/TendonSpecs.h
  ${SRC_DIR}/tendon/get_r_info.h
  ${SRC_DIR}/tendon/solve_initial_bending.h
  ${SRC_DIR}/tendon/tendon_deriv.h
  ${CONFIG_OUT}
  )
install(FILES ${TENDON_HEADERS} DESTINATION include/tendon)
list(APPEND LIBTENDON_ALL_HEADERS ${CONFIG_IN})
list(APPEND LIBTENDON_ALL_HEADERS ${TENDON_HEADERS})


# serialport
set(SERIAL_PORT_HEADERS ${SRC_DIR}/serialport/SerialPort.h)
install(FILES ${SERIAL_PORT_HEADERS} DESTINATION include/serialport)
list(APPEND LIBTENDON_ALL_HEADERS ${SERIAL_PORT_HEADERS})

#physical robot
set(PHYS_ROBOT_HEADERS ${SRC_DIR}/phys-robot/MotorControlSubscriber.h
                      ${SRC_DIR}/phys-robot/MotorControlPublisher.h
                      ${SRC_DIR}/phys-robot/trajectory_conversion.h
                      )
install(FILES ${PHYS_ROBOT_HEADERS} DESTINATION include/phys-robot)
list(APPEND LIBTENDON_ALL_HEADERS ${PHYS_ROBOT_HEADERS})

# haptic
if (ament_cmake_FOUND)
  set(HAPTIC_HEADERS
    ${SRC_DIR}/haptic/HapticQSubscriber.h
    ${SRC_DIR}/haptic/StreamingTransformer.h
    ${SRC_DIR}/haptic/HapticTransform.h
    )
  install(FILES ${HAPTIC_HEADERS} DESTINATION include/haptic)
  list(APPEND LIBTENDON_ALL_HEADERS ${HAPTIC_HEADERS})
endif ()

#magnetic tracker
set(MAGNETIC_TRACKER_HEADERS
    ${SRC_DIR}/mag-tracker/svd_reg.h
    ${SRC_DIR}/mag-tracker/StreamingSensor.h
    ${SRC_DIR}/mag-tracker/ndi/NdiManager.h
    ${SRC_DIR}/mag-tracker/ndi/ParseError.h
    ${SRC_DIR}/mag-tracker/ndi/ToolData.h
    ${SRC_DIR}/mag-tracker/ndi/char_parse.h
    )
install(FILES ${MAGNETIC_TRACKER_HEADERS} DESTINATION include/mag-tracker)
list(APPEND LIBTENDON_ALL_HEADERS ${MAGNETIC_TRACKER_HEADERS})

# qt_util
set(QT_UTIL_HEADERS
  ${SRC_DIR}/qt_util/StdinIO.h
  ${SRC_DIR}/qt_util/StdinWatcher.h
  ${SRC_DIR}/qt_util/asKeyValueRange.h
  ${SRC_DIR}/qt_util/streams.h
  )
install(FILES ${QT_UTIL_HEADERS} DESTINATION include/qt_util)
list(APPEND LIBTENDON_ALL_HEADERS ${QT_UTIL_HEADERS})






####################
#    libtendon     #
####################

set(TENDON_SOURCE
  ${SRC_DIR}/collision/Capsule.cpp
  ${SRC_DIR}/collision/CapsuleSequence.cpp
  ${SRC_DIR}/collision/Mesh.cpp
  ${SRC_DIR}/collision/Sphere.cpp
  ${SRC_DIR}/collision/VoxelOctree.cpp
  ${SRC_DIR}/collision/collision.cpp
  ${SRC_DIR}/collision/collision_primitives.cpp
  ${SRC_DIR}/cpptoml/cpptoml.cpp
  ${SRC_DIR}/cpptoml/toml_conversions.cpp
  ${SRC_DIR}/csv/Csv.cpp
  ${SRC_DIR}/design-optimization/MultiGoalPlanningCostIK.cpp
  ${SRC_DIR}/motion-planning/AbstractValidityChecker.cpp
  ${SRC_DIR}/motion-planning/Environment.cpp
  ${SRC_DIR}/motion-planning/LazyPRMFixed.cpp
  ${SRC_DIR}/motion-planning/Problem.cpp
  ${SRC_DIR}/motion-planning/StraightLinePlanner.cpp
  ${SRC_DIR}/motion-planning/StreamingPlanner.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidator.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneDiscreteMotionValidator.cpp
  ${SRC_DIR}/motion-planning/VoxelBackboneMotionValidatorAndLogger.cpp
  ${SRC_DIR}/motion-planning/VoxelCachedLazyPRM.cpp
  ${SRC_DIR}/motion-planning/VoxelEnvironment.cpp
  ${SRC_DIR}/motion-planning/ompl_planners.cpp
  ${SRC_DIR}/motion-planning/plan.cpp
  ${SRC_DIR}/tendon/BackboneSpecs.cpp
  ${SRC_DIR}/tendon/TendonResult.cpp
  ${SRC_DIR}/tendon/TendonRobot.cpp
  ${SRC_DIR}/tendon/TendonSpecs.cpp
  ${SRC_DIR}/tendon/get_r_info.cpp
  ${SRC_DIR}/tendon/solve_initial_bending.cpp
  ${SRC_DIR}/tendon/tendon_deriv.cpp
  ${SRC_DIR}/tip-control/Controller.cpp
  ${SRC_DIR}/tip-control/tip_control.cpp
  ${SRC_DIR}/util/eigen_ops.cpp
  ${SRC_DIR}/util/json_io.cpp
  ${SRC_DIR}/vistendon/view_path.cpp

  ${SRC_DIR}/serialport/SerialPort.cpp

  ${SRC_DIR}/vistendon/StreamingInterleaver.cpp
  ${SRC_DIR}/mag-tracker/ndi/NdiManager.cpp
  ${SRC_DIR}/mag-tracker/ndi/ToolData.cpp
  ${SRC_DIR}/qt_util/StdinIO.cpp
  ${SRC_DIR}/qt_util/StdinWatcher.cpp
  ${SRC_DIR}/qt_util/streams.cpp

  ${LIBTENDON_ALL_HEADERS}
  )
if (ament_cmake_FOUND)
  list(APPEND TENDON_SOURCE
    ${SRC_DIR}/vistendon/StreamingInterleaver.cpp
    ${SRC_DIR}/vistendon/view_path.cpp
    ${SRC_DIR}/haptic/HapticQSubscriber.cpp
    ${SRC_DIR}/haptic/HapticTransform.cpp
    )
endif ()
add_library(tendon SHARED ${TENDON_SOURCE})
target_include_directories(tendon PUBLIC
  ${SRC_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/include # for config.h
  ${THIRD_PARTY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${ITK_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  )
set_property(TARGET tendon PROPERTY CXX_STANDARD 17)
target_include_directories(tendon PUBLIC ${SRC_DIR})
if (ament_cmake_FOUND)
  ament_target_dependencies(tendon
    geometry_msgs
    rclcpp
    std_msgs
    visualization_msgs
    )
endif ()
target_link_libraries(tendon
  ${FCL_LIBRARIES}
  ${ITK_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
  Boost::filesystem
  Boost::iostreams
  Boost::system
  Eigen3::Eigen
  Qt5::Core
  Qt5::Gui
  Qt5::SerialPort
  levmar
  readline
  )

target_compile_definitions(tendon PUBLIC CPPTOML_USE_MAP)
install(TARGETS tendon DESTINATION lib/${PROJECT_NAME})


add_subdirectory(${THIRD_PARTY_DIR}/levmar-2.6 3rdparty/levmar-2.6)
if (ament_cmake_FOUND)
  add_subdirectory(${THIRD_PARTY_DIR}/libros2qt  3rdparty/libros2qt )
endif ()


####################
#   applications   #
####################

set(APP_SRC_DIR ${SRC_DIR}/apps)

# tendon shape example
add_executable(tendon_shape_example ${APP_SRC_DIR}/tendon_shape_example.cpp)
set_property(TARGET tendon_shape_example PROPERTY CXX_STANDARD 17)
target_link_libraries(tendon_shape_example tendon)
install(TARGETS tendon_shape_example DESTINATION lib/${PROJECT_NAME})

if (ament_cmake_FOUND)
  # view tendon example
  add_executable(view_tendon ${APP_SRC_DIR}/view_tendon.cpp)
  set_property(TARGET view_tendon PROPERTY CXX_STANDARD 17)
  target_link_libraries(view_tendon tendon)
  install(TARGETS view_tendon DESTINATION lib/${PROJECT_NAME})

  # view problem
  add_executable(view_problem ${APP_SRC_DIR}/view_problem.cpp)
  set_property(TARGET view_problem PROPERTY CXX_STANDARD 17)
  target_link_libraries(view_problem tendon)
  install(TARGETS view_problem DESTINATION lib/${PROJECT_NAME})

  # view voxel problem
  add_executable(view_voxel_problem ${APP_SRC_DIR}/view_voxel_problem.cpp)
  set_property(TARGET view_voxel_problem PROPERTY CXX_STANDARD 17)
  target_link_libraries(view_voxel_problem tendon)
  install(TARGETS view_voxel_problem DESTINATION lib/${PROJECT_NAME})

  # resolved_rate_problem
  add_executable(resolved_rate_problem ${APP_SRC_DIR}/resolved_rate_problem.cpp)
  set_property(TARGET resolved_rate_problem PROPERTY CXX_STANDARD 17)
  target_link_libraries(resolved_rate_problem tendon)
  install(TARGETS resolved_rate_problem DESTINATION lib/${PROJECT_NAME})
endif ()

# create plan
add_executable(create_plan ${APP_SRC_DIR}/create_plan.cpp)
set_property(TARGET create_plan PROPERTY CXX_STANDARD 17)
target_link_libraries(create_plan tendon)
install(TARGETS create_plan DESTINATION lib/${PROJECT_NAME})

if (ament_cmake_FOUND)
  # view plan
  add_executable(view_plan ${APP_SRC_DIR}/view_plan.cpp)
  set_property(TARGET view_plan PROPERTY CXX_STANDARD 17)
  target_link_libraries(view_plan tendon)
  install(TARGETS view_plan DESTINATION lib/${PROJECT_NAME})
endif ()

# plan cost
add_executable(plan_cost ${APP_SRC_DIR}/plan_cost.cpp)
set_property(TARGET plan_cost PROPERTY CXX_STANDARD 17)
target_link_libraries(plan_cost tendon)
install(TARGETS plan_cost DESTINATION lib/${PROJECT_NAME})

# fix stl
add_executable(fix_stl ${APP_SRC_DIR}/fix_stl.cpp)
set_property(TARGET fix_stl PROPERTY CXX_STANDARD 17)
target_link_libraries(fix_stl tendon)
install(TARGETS fix_stl DESTINATION lib/${PROJECT_NAME})

# query planner
add_executable(query_planner ${APP_SRC_DIR}/query_planner.cpp)
set_property(TARGET query_planner PROPERTY CXX_STANDARD 17)
target_link_libraries(query_planner tendon)
install(TARGETS query_planner DESTINATION lib/${PROJECT_NAME})

if (ament_cmake_FOUND)
  # tip controller example app
  add_executable(control_app ${APP_SRC_DIR}/control_app.cpp)
  set_property(TARGET control_app PROPERTY CXX_STANDARD 17)
  target_link_libraries(control_app tendon)
  install(TARGETS control_app DESTINATION lib/${PROJECT_NAME})
endif ()

# estimate tension limits
add_executable(estimate_tension_limits ${APP_SRC_DIR}/estimate_tension_limits.cpp)
set_property(TARGET estimate_tension_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_tension_limits tendon)
install(TARGETS estimate_tension_limits DESTINATION lib/${PROJECT_NAME})

# estimate length limits
add_executable(estimate_length_limits ${APP_SRC_DIR}/estimate_length_limits.cpp)
set_property(TARGET estimate_length_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_length_limits tendon)
install(TARGETS estimate_length_limits DESTINATION lib/${PROJECT_NAME})

# estimate length discretization
add_executable(estimate_length_discretization ${APP_SRC_DIR}/estimate_length_discretization.cpp)
set_property(TARGET estimate_length_discretization PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_length_discretization tendon)
install(TARGETS estimate_length_discretization DESTINATION lib/${PROJECT_NAME})

# gen tasks
add_executable(gen_tasks ${APP_SRC_DIR}/gen_tasks.cpp)
set_property(TARGET gen_tasks PROPERTY CXX_STANDARD 17)
target_link_libraries(gen_tasks tendon Boost::filesystem)
install(TARGETS gen_tasks DESTINATION lib/${PROJECT_NAME})

# sample configs
add_executable(sample_configs ${APP_SRC_DIR}/sample_configs.cpp)
set_property(TARGET sample_configs PROPERTY CXX_STANDARD 17)
target_link_libraries(sample_configs tendon Boost::filesystem)
install(TARGETS sample_configs DESTINATION lib/${PROJECT_NAME})

# sample tips
add_executable(sample_tips ${APP_SRC_DIR}/sample_tips.cpp)
set_property(TARGET sample_tips PROPERTY CXX_STANDARD 17)
target_link_libraries(sample_tips tendon)
install(TARGETS sample_tips DESTINATION lib/${PROJECT_NAME})

# profile fk
add_executable(profile_fk ${APP_SRC_DIR}/profile_fk.cpp)
set_property(TARGET profile_fk PROPERTY CXX_STANDARD 17)
target_link_libraries(profile_fk tendon)
install(TARGETS profile_fk DESTINATION lib/${PROJECT_NAME})

# profile problem
add_executable(profile_problem ${APP_SRC_DIR}/profile_problem.cpp)
set_property(TARGET profile_problem PROPERTY CXX_STANDARD 17)
target_link_libraries(profile_problem tendon Boost::filesystem)
install(TARGETS profile_problem DESTINATION lib/${PROJECT_NAME})

# voxelize sphere
add_executable(voxelize_sphere ${APP_SRC_DIR}/voxelize_sphere.cpp)
set_property(TARGET voxelize_sphere PROPERTY CXX_STANDARD 17)
target_link_libraries(voxelize_sphere tendon)
install(TARGETS voxelize_sphere DESTINATION lib/${PROJECT_NAME})

# nrrd 2 mesh
add_executable(nrrd2mesh ${APP_SRC_DIR}/nrrd2mesh.cpp)
set_property(TARGET nrrd2mesh PROPERTY CXX_STANDARD 17)
target_link_libraries(nrrd2mesh tendon)
install(TARGETS nrrd2mesh DESTINATION lib/${PROJECT_NAME})

# nrrd resample
add_executable(nrrdresample ${APP_SRC_DIR}/nrrdresample.cpp)
set_property(TARGET nrrdresample PROPERTY CXX_STANDARD 17)
target_include_directories(nrrdresample PUBLIC ${ITK_INCLUDE_DIRS})
target_link_libraries(nrrdresample ${ITK_LIBRARIES})
install(TARGETS nrrdresample DESTINATION lib/${PROJECT_NAME})

# prepare voxel env
add_executable(prepare_voxel_env ${APP_SRC_DIR}/prepare_voxel_env.cpp)
set_property(TARGET prepare_voxel_env PROPERTY CXX_STANDARD 17)
target_link_libraries(prepare_voxel_env tendon Boost::filesystem)
install(TARGETS prepare_voxel_env DESTINATION lib/${PROJECT_NAME})

# cheap inverse kinematics
add_executable(cheap_ik ${APP_SRC_DIR}/cheap_ik.cpp)
set_property(TARGET cheap_ik PROPERTY CXX_STANDARD 17)
target_link_libraries(cheap_ik tendon ${OMPL_LIBRARIES})
target_include_directories(cheap_ik PUBLIC ${OMPL_INCLUDE_DIRS})
install(TARGETS cheap_ik DESTINATION lib/${PROJECT_NAME})

# voxel convert
add_executable(voxel_convert ${APP_SRC_DIR}/voxel_convert.cpp)
set_property(TARGET voxel_convert PROPERTY CXX_STANDARD 17)
target_link_libraries(voxel_convert tendon)
install(TARGETS voxel_convert DESTINATION lib/${PROJECT_NAME})

# json pretty (print)
add_executable(json_pretty ${APP_SRC_DIR}/json_pretty.cpp)
set_property(TARGET json_pretty PROPERTY CXX_STANDARD 17)
target_link_libraries(json_pretty tendon)
install(TARGETS json_pretty DESTINATION lib/${PROJECT_NAME})

# json convert
add_executable(json_convert ${APP_SRC_DIR}/json_convert.cpp)
set_property(TARGET json_convert PROPERTY CXX_STANDARD 17)
target_link_libraries(json_convert tendon)
install(TARGETS json_convert DESTINATION lib/${PROJECT_NAME})

# create roadmap
add_executable(create_roadmap ${APP_SRC_DIR}/create_roadmap.cpp)
set_property(TARGET create_roadmap PROPERTY CXX_STANDARD 17)
target_link_libraries(create_roadmap tendon)
install(TARGETS create_roadmap DESTINATION lib/${PROJECT_NAME})

# roadmap to samples
add_executable(roadmap2samples ${APP_SRC_DIR}/roadmap2samples.cpp)
set_property(TARGET roadmap2samples PROPERTY CXX_STANDARD 17)
target_link_libraries(roadmap2samples tendon)
install(TARGETS roadmap2samples DESTINATION lib/${PROJECT_NAME})

# roadmap IK
add_executable(roadmap_ik ${APP_SRC_DIR}/roadmap_ik.cpp)
set_property(TARGET roadmap_ik PROPERTY CXX_STANDARD 17)
target_link_libraries(roadmap_ik tendon)
install(TARGETS roadmap_ik DESTINATION lib/${PROJECT_NAME})

# profile chained plan
add_executable(profile_chained_plan ${APP_SRC_DIR}/profile_chained_plan.cpp)
set_property(TARGET profile_chained_plan PROPERTY CXX_STANDARD 17)
target_link_libraries(profile_chained_plan tendon)
install(TARGETS profile_chained_plan DESTINATION lib/${PROJECT_NAME})

# roadmap chained plan
add_executable(roadmap_chained_plan ${APP_SRC_DIR}/roadmap_chained_plan.cpp)
set_property(TARGET roadmap_chained_plan PROPERTY CXX_STANDARD 17)
target_link_libraries(roadmap_chained_plan tendon)
install(TARGETS roadmap_chained_plan DESTINATION lib/${PROJECT_NAME})

# serial communication example
add_executable(robot_comm_example ${APP_SRC_DIR}/robot_comm_example.cpp)
set_property(TARGET robot_comm_example PROPERTY CXX_STANDARD 17)
target_link_libraries(robot_comm_example tendon Qt5::Core Qt5::SerialPort)
install(TARGETS robot_comm_example DESTINATION lib/${PROJECT_NAME})

# control physical robot
add_executable(robot_serial ${APP_SRC_DIR}/robot_serial.cpp)
set_property(TARGET robot_serial PROPERTY CXX_STANDARD 17)
target_link_libraries(robot_serial tendon)
install(TARGETS robot_serial DESTINATION lib/${PROJECT_NAME})

# publish motor controls
add_executable(tip_publish ${APP_SRC_DIR}/tip_publish.cpp)
set_property(TARGET tip_publish PROPERTY CXX_STANDARD 17)
target_link_libraries(tip_publish tendon)
install(TARGETS tip_publish DESTINATION lib/${PROJECT_NAME})

# haptic subscriber example
if (ament_cmake_FOUND)
  add_executable(haptic_subscriber_example ${APP_SRC_DIR}/haptic_subscriber_example.cpp)
  set_property(TARGET haptic_subscriber_example PROPERTY CXX_STANDARD 17)
  target_link_libraries(haptic_subscriber_example tendon libros2qt)
  install(TARGETS haptic_subscriber_example DESTINATION lib/${PROJECT_NAME})
endif ()

# haptic tip rviz viewer
add_executable(haptic_tip_rviz_viewer ${APP_SRC_DIR}/haptic_tip_rviz_viewer.cpp)
set_property(TARGET haptic_tip_rviz_viewer PROPERTY CXX_STANDARD 17)
target_link_libraries(haptic_tip_rviz_viewer tendon libros2qt)
install(TARGETS haptic_tip_rviz_viewer DESTINATION lib/${PROJECT_NAME})

# plan with haptic controller
if (ament_cmake_FOUND)
  add_executable(plan_with_haptic
    ${APP_SRC_DIR}/plan_with_haptic/main.cpp
    ${APP_SRC_DIR}/plan_with_haptic/GoalTracker.h
    ${APP_SRC_DIR}/plan_with_haptic/InteriorTracker.h
    )
  set_property(TARGET plan_with_haptic PROPERTY CXX_STANDARD 17)
  target_link_libraries(plan_with_haptic tendon libros2qt)
  install(TARGETS plan_with_haptic DESTINATION lib/${PROJECT_NAME})
endif ()

#control physical robot with haptic controller
add_executable(phys_robot_control
  ${APP_SRC_DIR}/phys_robot_control/phys_robot_control.cpp
  ${APP_SRC_DIR}/phys_robot_control/GoalTracker.h
  ${APP_SRC_DIR}/phys_robot_control/InteriorTracker.h
  )
set_property(TARGET phys_robot_control PROPERTY CXX_STANDARD 17)
target_link_libraries(phys_robot_control tendon libros2qt)
install(TARGETS phys_robot_control DESTINATION lib/${PROJECT_NAME})

# svd registration example
add_executable(reg_example ${APP_SRC_DIR}/reg_example.cpp)
set_property(TARGET reg_example PROPERTY CXX_STANDARD 17)
target_link_libraries(reg_example tendon)
install(TARGETS reg_example DESTINATION lib/${PROJECT_NAME})

# compute transformed points
add_executable(transform_points ${APP_SRC_DIR}/transform_points.cpp)
set_property(TARGET transform_points PROPERTY CXX_STANDARD 17)
target_link_libraries(transform_points tendon)
install(TARGETS transform_points DESTINATION lib/${PROJECT_NAME})

# ndi terminal
add_executable(ndi-terminal ${APP_SRC_DIR}/ndi-terminal.cpp)
set_property(TARGET ndi-terminal PROPERTY CXX_STANDARD 17)
target_link_libraries(ndi-terminal tendon
  )
install(TARGETS ndi-terminal DESTINATION lib/${PROJECT_NAME})

# available-serial-ports
add_executable(available-serial-ports
  ${APP_SRC_DIR}/available-serial-ports.cpp
  )
set_property(TARGET available-serial-ports PROPERTY CXX_STANDARD 17)
target_link_libraries(available-serial-ports Qt5::Core Qt5::SerialPort)
install(TARGETS available-serial-ports DESTINATION lib/${PROJECT_NAME})

# view a set of points in RViz(useful for tendon tip visualization)
add_executable(view_points ${APP_SRC_DIR}/view_points.cpp)
set_property(TARGET view_points PROPERTY CXX_STANDARD 17)
target_link_libraries(view_points tendon)
install(TARGETS view_points DESTINATION lib/${PROJECT_NAME})

# closed loop controller
add_executable(closed_loop_control_example
    ${APP_SRC_DIR}/closed_loop_control_example.cpp)
set_property(TARGET closed_loop_control_example PROPERTY CXX_STANDARD 17)
target_link_libraries(closed_loop_control_example tendon libros2qt)
install(TARGETS closed_loop_control_example DESTINATION lib/${PROJECT_NAME})

#data collector for DL based controller
add_executable(data_collector ${APP_SRC_DIR}/data_collector.cpp)
set_property(TARGET data_collector PROPERTY CXX_STANDARD 17)
target_link_libraries(data_collector tendon)
install(TARGETS data_collector DESTINATION lib/${PROJECT_NAME})

#data collector for DL based controller
add_executable(sensor_rviz_viewer ${APP_SRC_DIR}/sensor_rviz_viewer.cpp)
set_property(TARGET sensor_rviz_viewer PROPERTY CXX_STANDARD 17)
target_link_libraries(sensor_rviz_viewer tendon libros2qt)
install(TARGETS sensor_rviz_viewer DESTINATION lib/${PROJECT_NAME})



#######################
#   python bindings   #
#######################

if (${pybind11_FOUND})
  set(PYTHON_BINDINGS_SRC_DIR ${SRC_DIR}/python-bindings)
  pybind11_add_module(cpptendon
    ${PYTHON_BINDINGS_SRC_DIR}/module_cpptendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_collision.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_collision.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_design_optimization.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_design_optimization.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_motion_planning.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_motion_planning.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_ndi.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_ndi.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.h
    )
  target_link_libraries(cpptendon PUBLIC tendon)
  set_property(TARGET cpptendon PROPERTY CXX_STANDARD 17)
  install(TARGETS cpptendon DESTINATION lib/${PROJECT_NAME})
endif()



######################
#   python scripts   #
######################

set(PYTHON_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/python/src)

# creates the target for scriptfile without the .py at the end and copies it
# into the build directory.
function(add_python_script scriptfile)
  get_filename_component(filebase ${scriptfile} NAME_WE)
  set(filepath ${PYTHON_SRC_DIR}/${scriptfile})
  set(dest ${CMAKE_BINARY_DIR}/${scriptfile})
  if(TENDON_LINK_PYTHON_SCRIPTS)
    set(command ${CMAKE_COMMAND} -E create_symlink ${filepath} ${dest})
  else()
    #configure_file(${PYTHON_SRC_DIR}/${scriptfile} ${scriptfile} COPYONLY)
    set(command ${CMAKE_COMMAND} -E copy ${filepath} ${dest})
  endif(TENDON_LINK_PYTHON_SCRIPTS)
  add_custom_target(${filebase} ALL
    COMMAND ${command}
    SOURCES ${filepath}
    BYPRODUCTS ${dest}
    )
  install(PROGRAMS ${dest} DESTINATION lib/${PROJECT_NAME})
endfunction(add_python_script)

add_python_script(aggregate_roadmap_stats.py)
add_python_script(chain_plans.py)
add_python_script(clean_toml.py)
add_python_script(create_task_sequence.py)
add_python_script(csv_combine.py)
add_python_script(fix_milestones.py)
add_python_script(gen_tendon_region_samples.py)
add_python_script(plot_roadmap_chain_histogram.py)
add_python_script(roadmap_stats.py)

if (TARGET cpptendon)
  add_python_script(pc_sim.py)
  add_dependencies(pc_sim cpptendon)
  add_python_script(trajectory_gpt2.py)
  add_dependencies(trajectory_gpt2 cpptendon)
  add_python_script(CosseratRod.py)
  add_dependencies(CosseratRod cpptendon)
  add_python_script(len_to_tau_conversion.py)
  add_dependencies(len_to_tau_conversion cpptendon)
  add_python_script(pbf_traj_collection.py)
  add_dependencies(pbf_traj_collection cpptendon)
  add_python_script(traj_to_datalist.py)
  add_dependencies(traj_to_datalist cpptendon)
  add_python_script(test_sim_config.py)
  add_dependencies(test_sim_config cpptendon)
  add_python_script(sim_to_real_property.py)
  add_dependencies(sim_to_real_property cpptendon)
  add_python_script(test_hysteresis_config.py)
  add_dependencies(test_hysteresis_config cpptendon)
  add_python_script(pickle_to_list.py)
  add_dependencies(pickle_to_list cpptendon)
  add_python_script(learn_actual_pc.py)
  add_dependencies(learn_actual_pc cpptendon)
  add_python_script(csv_to_pkl.py)
  add_dependencies(csv_to_pkl cpptendon)
  add_python_script(emd.py)
  add_dependencies(emd cpptendon)
  add_python_script(learn_tendon_pc.py)
  add_dependencies(learn_tendon_pc cpptendon)
  add_python_script(learn_tendon_shape.py)
  add_dependencies(learn_tendon_shape cpptendon)
  add_python_script(test_config.py)
  add_dependencies(test_config cpptendon)
  add_python_script(pbf_data_collection.py)
  add_dependencies(pbf_data_collection cpptendon)
  add_python_script(simulated_data_collection.py)
  add_dependencies(simulated_data_collection cpptendon)
  add_python_script(gridgen.py)
  add_dependencies(gridgen cpptendon)
  add_python_script(measure_tip_poses.py)
  add_dependencies(measure_tip_poses cpptendon)
endif()


#############
#   tests   #
#############

if(TENDON_BUILD_TESTING AND ament_cmake_FOUND)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_link_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  set(TEST_FOLDER cpp/tests)

  #add_executable(all_tests
  ament_add_gtest(all_tests
    ${TEST_FOLDER}/collision/tst_Capsule.cpp
    ${TEST_FOLDER}/collision/tst_CapsuleSequence.cpp
    ${TEST_FOLDER}/collision/tst_Mesh.cpp
    ${TEST_FOLDER}/collision/tst_Sphere.cpp
    ${TEST_FOLDER}/collision/tst_VoxelOctree.cpp
    ${TEST_FOLDER}/collision/tst_collision.cpp
    ${TEST_FOLDER}/collision/tst_stl_io.cpp
    ${TEST_FOLDER}/design-optimization/tst_MultiGoalPlanningCostIK.cpp
    ${TEST_FOLDER}/cpptoml/tst_toml_conversions.cpp
    ${TEST_FOLDER}/motion-planning/tst_Environment.cpp
    ${TEST_FOLDER}/motion-planning/tst_Problem.cpp
    ${TEST_FOLDER}/spline/tst_Cubic.cpp
    ${TEST_FOLDER}/spline/tst_CubicSpline.cpp
    ${TEST_FOLDER}/spline/tst_CubicSplineSequence.cpp
    ${TEST_FOLDER}/tendon/tst_BackboneSpecs.cpp
    ${TEST_FOLDER}/tendon/tst_TendonResult.cpp
    ${TEST_FOLDER}/tendon/tst_TendonRobot.cpp
    ${TEST_FOLDER}/tendon/tst_TendonSpecs.cpp
    ${TEST_FOLDER}/tendon/tst_tendon_accuracy.cpp
    ${TEST_FOLDER}/tip-control/tst_Controller.cpp
    ${TEST_FOLDER}/util/tst_FunctionTimer.cpp
    ${TEST_FOLDER}/util/tst_poly.cpp
    ${TEST_FOLDER}/util/tst_vector_ops.cpp
    ${TEST_FOLDER}/util/tst_angles.cpp
    )
  set_property(TARGET all_tests PROPERTY CXX_STANDARD 17)
  target_include_directories(all_tests PRIVATE ${SRC_DIR})
  target_link_libraries(all_tests
    #gtest gtest_main
    GTest::GTest
    GTest::Main
    pthread
    Eigen3::Eigen
    tendon
    )
  gtest_discover_tests(all_tests)
endif()

if (${ament_cmake_FOUND})
  ament_package()
endif ()
