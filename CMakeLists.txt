cmake_minimum_required(VERSION 3.8.2)
set_property(GLOBAL PROPERTY CSTD c11)

macro(set_conf_file)
  if(EXISTS ${APPLICATION_SOURCE_DIR}/prj_${BOARD}.conf)
    set(CONF_FILE "${APPLICATION_SOURCE_DIR}/prj_${BOARD}.conf")
  elseif(EXISTS ${APPLICATION_SOURCE_DIR}/boards/${BOARD}.conf)
    set(CONF_FILE
      "prj.conf ${APPLICATION_SOURCE_DIR}/boards/${BOARD}.conf")
  else()
    set(CONF_FILE "prj.conf")
  endif()
endmacro()

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(otg2)

enable_language(C ASM)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
 -DARM_MATH_CM4 \
 -DDISABLE_FLOAT_API \
 -DFIXED_POINT \
 -DUSE_ALLOCA \
 -DHAVE_LRINT \
 -DHAVE_LRINTF \
 -DOPUS_ARM_ASM \
 -DOPUS_ARM_INLINE_EDSP \
 -DOPUS_ARM_INLINE_MEDIA \
 -DOPUS_ARM_MAY_HAVE_EDSP \
 -DOPUS_ARM_PRESUME_EDSP \
 -DCUSTOM_SUPPORT \
 -DVAR_ARRAYS \
 -DOPUS_BUILD \
 -DHAVE_CONFIG_H \
 -g \
")
# -DOVERRIDE_OPUS_ALLOC -DOVERRIDE_OPUS_FREE -D'opus_alloc(x)=NULL' -D'opus_free(x)=NULL' \

# Use DOPUS_ARM_INLINE_EDSP instead of
# -DOPUS_ARM_INLINE_ASM \

# Not if use minimal lib
# -DHAVE_ALLOCA_H \



target_sources(                     app PRIVATE src/main.c)


FILE(GLOB_RECURSE audio_sources src/audio/*.c)
FILE(GLOB_RECURSE radio_sources src/radio/*.c)
FILE(GLOB_RECURSE oled_sources src/oled/*.c)


target_sources(app PRIVATE
	${audio_sources}
	src/audio/opus-1.2.1/arm/celt_pitch_xcorr_arm_gcc.s
	${radio_sources}
	${oled_sources}
	)


zephyr_include_directories(
	src/
	src/audio/
	src/audio/opus-1.2.1/
	src/audio/sgtl5000/
	src/radio/
	src/oled/
	)

include($ENV{ZEPHYR_BASE}/samples/net/common/common.cmake)
