cmake_minimum_required(VERSION 2.8.3)
project(trabalho_final)

set(fuzzylite ~/fuzzylite/fuzzylite)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        sensor_msgs
        std_msgs
        nav_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs nav_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${fuzzylite}/release/bin
        ${fuzzylite}
)

link_directories(
        ${fuzzylite}/release/bin
        ${fuzzylite}
)

set(H_FILES
         include/Node.h
         include/trabalho_final/ControladorDeDeslocamento.h
         include/trabalho_final/EvitadorDeColisoes.h
         include/trabalho_final/Conversor.h
)

set(CPP_FILES
         src/Node.cpp
         src/trabalho_final/ControladorDeDeslocamento.cpp
         src/trabalho_final/EvitadorDeColisoes.cpp
         src/trabalho_final/Conversor.cpp
)

#add_executable(fuzzy_controller_node src/fuzzy_controller_node.cpp ${CPP_FILES})
#add_dependencies(fuzzy_controller_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(fuzzy_controller_node ${catkin_LIBRARIES} fuzzylite)

add_executable(evitador_de_colisoes src/evitador_de_colisoes.cpp ${CPP_FILES})
add_dependencies(evitador_de_colisoes ${catkin_EXPORTED_TARGETS})
target_link_libraries(evitador_de_colisoes ${catkin_LIBRARIES} fuzzylite)

add_executable(conversor src/conversor.cpp ${CPP_FILES})
add_dependencies(conversor ${catkin_EXPORTED_TARGETS})
target_link_libraries(conversor ${catkin_LIBRARIES} fuzzylite)

add_executable(controlador_de_deslocamento src/controlador_de_deslocamento.cpp ${CPP_FILES})
add_dependencies(controlador_de_deslocamento ${catkin_EXPORTED_TARGETS})
target_link_libraries(controlador_de_deslocamento ${catkin_LIBRARIES} fuzzylite)
