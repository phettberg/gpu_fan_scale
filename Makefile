#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

#
# The base directory for TivaWare.
#
ROOT=${TIVAWARE}

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=${TIVAWARE}/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=${TIVAWARE}

#
# The default rule, which causes the gpu_fan_control example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/gpu_fan_control.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the gpu_fan_control example.
#
${COMPILER}/gpu_fan_control.axf: ${COMPILER}/gpu_fan_control.o
${COMPILER}/gpu_fan_control.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/gpu_fan_control.axf: ${COMPILER}/uartstdio.o
${COMPILER}/gpu_fan_control.axf: ${COMPILER}/ustdlib.o
${COMPILER}/gpu_fan_control.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/gpu_fan_control.axf: gpu_fan_control.ld
SCATTERgcc_gpu_fan_control=gpu_fan_control.ld
ENTRY_gpu_fan_control=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
