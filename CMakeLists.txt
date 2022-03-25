cmake_minimum_required(VERSION 3.5)

project(engine)

############################################################
# Set compiler flags
############################################################
# If supported by your compiler, you can add the -Wall, -Wextra, –fstack-protector-all and -g3 flags here.
set(OWN_GXX_FLAGS "-std=c++14 -Wall -Werror -Wextra -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "-Wall -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OWN_GXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OWN_GXX_FLAGS}")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OWN_GXX_FLAGS}")

############################################################
# List all sources
############################################################
set(engine_sources
	easy_image.cc
    easy_image.h
	engine.cc
	ini_configuration.cc

    lib/l_parser/l_parser.cc
    lib/l_parser/l_parser.cc

    lib/vector3d/vector3d.cc
    lib/vector3d/vector3d.h

    util/Color.cc
    util/Color.h
    util/Line2D.cc
    util/Line2D.h
    util/Point2D.cc
    util/Point2D.h

    util/Figure.h
    util/Figure.cc
    util/Face.h
    util/Face.cc

    util/generators/Transformations.h
    util/generators/Transformations.cc
    util/generators/PlatonicSolids.h
    util/generators/PlatonicSolids.cc

    handlers/2DLsystem.cc
    handlers/2DLsystem.h
    handlers/3DLsystem.cc
    handlers/3DLsystem.h
    handlers/Intro.cc
    handlers/Intro.h
    handlers/Universal.cc
    handlers/Universal.h
    handlers/WireFrame.cc
    handlers/WireFrame.h
)

############################################################
# Create an executable
############################################################
set(exe_name "engine")
add_executable( ${exe_name} ${engine_sources} )
install( TARGETS ${exe_name} DESTINATION ${PROJECT_SOURCE_DIR}/ )
