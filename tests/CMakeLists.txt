cmake_minimum_required(VERSION 3.14)
project(recovery_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ORT_ROOT "${REPO_ROOT}/onnxruntime-linux-x64-1.22.0")
add_executable(recovery_state_test recovery_state_test.cpp
    "${REPO_ROOT}/src/FSM/FSMState.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmpRecovery.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmp.cpp"
    "${REPO_ROOT}/src/FSM/State_Loco.cpp"
    "${REPO_ROOT}/src/FSM/State_FixedStand.cpp"
    "${REPO_ROOT}/src/FSM/State_Passive.cpp")
target_include_directories(recovery_state_test PRIVATE "${REPO_ROOT}/include" "${ORT_ROOT}/include")
target_compile_definitions(recovery_state_test PRIVATE PROJECT_ROOT_DIR="${REPO_ROOT}")
target_link_libraries(recovery_state_test PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json
    "${ORT_ROOT}/lib/libonnxruntime.so.1.22.0")
enable_testing()
add_test(NAME recovery_state COMMAND recovery_state_test)

add_library(fsm_test_runtime STATIC
    "${REPO_ROOT}/src/FSM/State_LocoRecovery.cpp"
    "${REPO_ROOT}/src/FSM/FSM.cpp"
    "${REPO_ROOT}/src/FSM/FSMState.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmpRecovery.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmp.cpp"
    "${REPO_ROOT}/src/FSM/State_Amp.cpp"
    "${REPO_ROOT}/src/FSM/State_Loco.cpp"
    "${REPO_ROOT}/src/FSM/State_WBC.cpp"
    "${REPO_ROOT}/src/FSM/State_FixedStand.cpp"
    "${REPO_ROOT}/src/FSM/State_Passive.cpp")
target_include_directories(fsm_test_runtime PUBLIC "${REPO_ROOT}/include" "${ORT_ROOT}/include")
target_compile_definitions(fsm_test_runtime PUBLIC PROJECT_ROOT_DIR="${REPO_ROOT}")
target_link_libraries(fsm_test_runtime PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json
    "${ORT_ROOT}/lib/libonnxruntime.so.1.22.0")
add_executable(recovery_fsm_test recovery_fsm_test.cpp)
target_link_libraries(recovery_fsm_test PRIVATE fsm_test_runtime)
add_test(NAME recovery_fsm COMMAND recovery_fsm_test)

add_executable(loco_recovery_fsm_test loco_recovery_fsm_test.cpp)
target_link_libraries(loco_recovery_fsm_test PRIVATE fsm_test_runtime)
add_test(NAME loco_recovery_fsm COMMAND loco_recovery_fsm_test)

add_executable(startup_test startup_test.cpp)
target_include_directories(startup_test PRIVATE "${REPO_ROOT}/include")
add_test(NAME startup COMMAND startup_test)

add_executable(loco_recovery_state_test loco_recovery_state_test.cpp
    "${REPO_ROOT}/src/FSM/FSMState.cpp"
    "${REPO_ROOT}/src/FSM/State_Loco.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmpRecovery.cpp"
    "${REPO_ROOT}/src/FSM/State_LocoRecovery.cpp")
target_include_directories(loco_recovery_state_test PRIVATE "${REPO_ROOT}/include" "${ORT_ROOT}/include")
target_compile_definitions(loco_recovery_state_test PRIVATE PROJECT_ROOT_DIR="${REPO_ROOT}")
target_link_libraries(loco_recovery_state_test PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json
    "${ORT_ROOT}/lib/libonnxruntime.so.1.22.0")
add_test(NAME loco_recovery_state COMMAND loco_recovery_state_test)

option(ENABLE_MUJOCO_RECOVERY_TEST "Build the isolated G1 physical fall/get-up/return regression" OFF)
if(ENABLE_MUJOCO_RECOVERY_TEST)
    set(MUJOCO_ROOT "${REPO_ROOT}/../unitree_mujoco/simulate/mujoco" CACHE PATH "MuJoCo installation")
    set(RECOVERY_TEST_SCENE "${REPO_ROOT}/../unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
        CACHE FILEPATH "G1 scene for the headless recovery test")
    find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h HINTS "${MUJOCO_ROOT}/include")
    find_library(MUJOCO_LIBRARY mujoco HINTS "${MUJOCO_ROOT}/lib")
    if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
        message(FATAL_ERROR "MuJoCo not found; specify MUJOCO_ROOT or disable ENABLE_MUJOCO_RECOVERY_TEST")
    endif()
    add_executable(loco_recovery_physics_test loco_recovery_physics_test.cpp)
    target_include_directories(loco_recovery_physics_test PRIVATE "${MUJOCO_INCLUDE_DIR}")
    target_compile_definitions(loco_recovery_physics_test PRIVATE RECOVERY_TEST_SCENE="${RECOVERY_TEST_SCENE}")
    target_link_libraries(loco_recovery_physics_test PRIVATE fsm_test_runtime "${MUJOCO_LIBRARY}")
    add_test(NAME loco_recovery_physics COMMAND loco_recovery_physics_test)
endif()
