set(_q "")
if(QHULL_FIND_QUIETLY)
    set(_q QUIET)
endif()
find_package(QHULL ${QHULL_FIND_VERSION} CONFIG ${_q})

if(NOT QHULL_FIND_QUIETLY)
    if (NOT QHULL_FOUND)
        message(STATUS "Falling back to MODULE search for QHULL...")
    else()
        message(STATUS "QHULL found in ${QHULL_DIR}")
    endif()
endif()

if (NOT QHULL_FOUND)
    set(_modpath ${CMAKE_MODULE_PATH})
    set(CMAKE_MODULE_PATH "")
    include(CheckIncludeFileCXX)
    
    add_library(qhull INTERFACE)
	add_library(orgQhull::Qhull::qhull_r ALIAS qhull)
	add_library(orgQhull::Qhull::qhullcpp ALIAS qhull)
    if(SLIC3R_STATIC)
        slic3r_remap_configs("Qhull::qhullcpp;Qhull::qhullstatic_r" RelWithDebInfo Release)
    	target_link_libraries(qhull INTERFACE qhullcpp qhullstatic_r)
    else()
        slic3r_remap_configs("Qhull::qhullcpp;Qhull::qhull_r" RelWithDebInfo Release)
    	target_link_libraries(qhull INTERFACE qhullcpp qhull_r)
    endif()
	
	target_include_directories(qhull INTERFACE include)
	
    set(CMAKE_MODULE_PATH ${_modpath})

    CHECK_INCLUDE_FILE_CXX("libqhullcpp/Qhull.h" QHULL_FOUND)
endif()
