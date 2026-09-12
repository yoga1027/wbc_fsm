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

add_executable(recovery_fsm_test recovery_fsm_test.cpp
    "${REPO_ROOT}/src/FSM/FSM.cpp"
    "${REPO_ROOT}/src/FSM/FSMState.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmpRecovery.cpp"
    "${REPO_ROOT}/src/FSM/State_MJAmp.cpp"
    "${REPO_ROOT}/src/FSM/State_Amp.cpp"
    "${REPO_ROOT}/src/FSM/State_Loco.cpp"
    "${REPO_ROOT}/src/FSM/State_WBC.cpp"
    "${REPO_ROOT}/src/FSM/State_FixedStand.cpp"
    "${REPO_ROOT}/src/FSM/State_Passive.cpp")
target_include_directories(recovery_fsm_test PRIVATE "${REPO_ROOT}/include" "${ORT_ROOT}/include")
target_compile_definitions(recovery_fsm_test PRIVATE PROJECT_ROOT_DIR="${REPO_ROOT}")
target_link_libraries(recovery_fsm_test PRIVATE Eigen3::Eigen nlohmann_json::nlohmann_json
    "${ORT_ROOT}/lib/libonnxruntime.so.1.22.0")
add_test(NAME recovery_fsm COMMAND recovery_fsm_test)
