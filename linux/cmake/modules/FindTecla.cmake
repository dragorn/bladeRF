unset( TECLA_INCLUDE_DIRS CACHE )

FIND_PATH( TECLA_INCLUDE_DIR libtecla.h
  PATHS
    $ENV{TECLA_DIR}/include
    $ENV{TECLA_INC}/
    /usr/include
    /usr/local/include
)

set( TECLA_INCLUDE_DIRS ${TECLA_INCLUDE_DIR} )
message( STATUS "Tecla include path(s): ${TECLA_INCLUDE_DIRS}" )

find_library( TECLA_LIB tecla
  PATHS
    $ENV{TECLA_DIR}/lib
    $ENV{TECLA_LIB}/
    /usr/lib
    /usr/local/lib
)

SET( TECLA_LIBRARIES ${TECLA_LIB} )

message( STATUS "Tecla library path(s): ${TECLA_LIBRARIES}" )
if( NOT TECLA_INCLUDE_DIR OR NOT TECLA_LIB )
    message( STATUS " \n Library tecla not found.  Interactivity disabled." )
endif( NOT TECLA_INCLUDE_DIR OR NOT TECLA_LIB )

MARK_AS_ADVANCED( TECLA_INCLUDE_DIR TECLA_INCLUDE_DIRS TECLA_LIBRARIES )

