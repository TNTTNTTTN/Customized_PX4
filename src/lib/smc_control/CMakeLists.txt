px4_add_library(SMC_Control
	smc_control.cpp
	smc_control.hpp
)
target_compile_options(SMC_Control PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(SMC_Control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SMC_Control PRIVATE mathlib)

px4_add_unit_gtest(SRC smc_control_test.cpp LINKLIBS SMC_Control)
