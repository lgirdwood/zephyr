# SPDX-License-Identifier: BSD-3-Clause

#add_library(pdown STATIC "")
#target_link_libraries(pdown sof_options)
#target_compile_options(pdown PRIVATE -mtext-section-literals)

#add_local_sources(pdown power_down.S)
#target_link_libraries(sof_static_libraries INTERFACE pdown)

zephyr_library_named(intel_adsp_common)
zephyr_library_include_directories(${ZEPHYR_BASE}/soc/xtensa/intel_adsp_common/include)

#zephyr_library_sources(adsp.c)
#zephyr_library_sources(clk.c)
#zephyr_library_sources(dai.c)
#zephyr_library_sources(dma.c)
#zephyr_library_sources(lps_pic_restore_vector.S)
#zephyr_library_sources(lps_wait.c)
#zephyr_library_sources(main_entry.S)
zephyr_library_sources(mem_window.c)
#zephyr_library_sources(memory.c)
#zephyr_library_sources(pm_memory.c)
#zephyr_library_sources(pm_runtime.c)

if(CAVS_VERSION_1_8 OR CAVS_VERSION_2_0 OR CAVS_VERSION_2_5)
  set_source_files_properties(power_down.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
  zephyr_library_sources(power_down.S)
endif()
