# v2x-kpi lives in the nr module's examples; compile it in rather than copying it.
set(NR_V2X_EX ${PROJECT_SOURCE_DIR}/contrib/nr/examples/nr-v2x-examples)
include_directories(${NR_V2X_EX})

build_lib_example(
  NAME sidewalk-robots
  SOURCE_FILES
    sidewalk-robots.cc
    ${NR_V2X_EX}/v2x-kpi.cc
  LIBRARIES_TO_LINK
    ${libsidewalk}
    ${libnr}
    ${libbuildings}
    ${libns3-cosim}
    ${libflow-monitor}
    ${libstats}
)
