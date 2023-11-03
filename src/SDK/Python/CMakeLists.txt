# CMakeLists.txt

cmake_minimum_required(VERSION 3.0)
project(hightorque_biped_py_sdk)
SET(CMAKE_CXX_FLAGS "-std=c++14")

add_subdirectory(lib/hightorque_biped_sdk_core)
add_subdirectory(lib/pybind11)

# set(PYTHON_INCLUDE_DIRS "/usr/include/python3.10" CACHE INTERNAL "")
# # 可以通过指令 python -c "import sysconfig; print(sysconfig.get_path('stdlib'))" 获得PYTHON_LIBRARIES，并填入<path-to-python-include-dir>中
# set(PYTHON_LIBRARIES "/usr/lib/python3.10" CACHE INTERNAL "")

# # 可以通过指令 which python 获得PYTHON_EXECUTABLE，并填入<path-to-python-include-dir>中
# set(PYTHON_EXECUTABLE "/usr/bin/python")

# # 可以通过指令 python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))" 获得PYTHON_LIBRARY，并填入<path-to-python-include-dir>中
# set(PYTHON_LIBRARY "/usr/lib/aarch64-linux-gnu")

set(SOURCE_DIR "src/hightorque_biped_sdk")
set(SOURCES "${SOURCE_DIR}/Livelybot_Driver_bind.cpp")
pybind11_add_module(Livelybot_Driver_bind ${SOURCES})
target_link_libraries(Livelybot_Driver_bind PRIVATE hightorque_biped_sdk_core)