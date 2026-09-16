# sidewalk: ns-3 contrib module on top of 5G-LENA NR V2X (sidelink Mode 2).
# Mount/symlink this repo at <ns-3-dev>/contrib/sidewalk.
build_lib(
  LIBNAME sidewalk
  SOURCE_FILES
    model/sensing-trace-sink.cc
    model/nr-sl-ue-mac-scheduler-earliest.cc
    model/robot-gateway.cc
  HEADER_FILES
    model/sensing-trace-sink.h
    model/nr-sl-ue-mac-scheduler-earliest.h
    model/robot-gateway.h
  LIBRARIES_TO_LINK
    ${libnr}
    ${libns3-cosim}
    ${libcore}
)
