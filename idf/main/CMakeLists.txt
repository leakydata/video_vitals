idf_component_register(SRCS "main.cpp" "rppg.cpp" INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
