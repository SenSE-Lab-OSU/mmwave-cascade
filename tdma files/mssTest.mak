###################################################################################
# Cascade Unit Test on MSS Makefile
###################################################################################
.PHONY: mssTest mssTestClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c $(MMWAVE_SDK_DEVICE_TYPE)

MSS_CPU := R5F

###################################################################################
# Additional include paths required
###################################################################################
#STD_INCL = -i$(XDC_INSTALL_PATH)/packages


###################################################################################
# The mmWave Unit Test requires additional libraries
###################################################################################
CASCADE_MSS_UNIT_TEST_STD_LIBS = $($(MSS_CPU)_COMMON_STD_LIB) \
								-llibtestlogger_$(MMWAVE_SDK_DEVICE_TYPE).$($(MSS_CPU)_LIB_EXT) \
								-llibmmwavelink_cascade_$(MMWAVE_SDK_DEVICE_TYPE).$($(MSS_CPU)_LIB_EXT) \
								-llibmmwave_cascade_$(MMWAVE_SDK_DEVICE_TYPE).$($(MSS_CPU)_LIB_EXT)
CASCADE_MSS_UNIT_TEST_LOC_LIBS = $($(MSS_CPU)_COMMON_LOC_LIB) \
								-Wl,-i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/testlogger/lib \
								-Wl,-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib \
								-Wl,-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib


###################################################################################
# Unit Test Files
###################################################################################
CASCADE_MSS_UNIT_TEST_CMD		 = $(MMWAVE_SDK_INSTALL_PATH)/ti/platform/$(MMWAVE_SDK_DEVICE_TYPE)
CASCADE_MSS_UNIT_TEST_MAP		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_cascade_mss.map
CASCADE_MSS_UNIT_TEST_OUT		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_cascade_mss.$($(MSS_CPU)_EXE_EXT)
CASCADE_MSS_UNIT_TEST_BIN		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_cascade_mss.bin
CASCADE_MSS_UNIT_TEST_APP_CMD	 = $(MMWAVE_SDK_DEVICE_TYPE)/mss_linker.cmd
CASCADE_MSS_UNIT_TEST_SOURCES	 = main_mss.c \
								   common.c \
								   cascade_lvds_stream.c \
								   cascade_csirx.c

CASCADE_MSS_UNIT_TEST_SOURCES_GEN = ti_board_config.c	\
									ti_board_open_close.c	\
									ti_dpl_config.c	\
									ti_drivers_config.c	\
									ti_pinmux_config.c	\
									ti_power_clock_config.c	\
									ti_drivers_open_close.c

CASCADE_MSS_UNIT_TEST_DEPENDS	 = $(addprefix $(PLATFORM_OBJDIR)/, $(CASCADE_MSS_UNIT_TEST_SOURCES:.c=.$($(MSS_CPU)_DEP_EXT)))
CASCADE_MSS_UNIT_TEST_OBJECTS	 = $(addprefix $(PLATFORM_OBJDIR)/, $(CASCADE_MSS_UNIT_TEST_SOURCES:.c=.$($(MSS_CPU)_OBJ_EXT)))

CASCADE_MSS_UNIT_TEST_OBJECTS_GEN	 = $(addprefix $(PLATFORM_OBJDIR)/mssgenerated/, $(CASCADE_MSS_UNIT_TEST_SOURCES_GEN:.c=.$($(MSS_CPU)_OBJ_EXT)))

###################################################################################
# Build Unit Test:
###################################################################################
mssTest: R5F_CFLAGS  += -DCASCADE_EVM
mssTest: buildDirectories mssbuildDirectories $(CASCADE_MSS_UNIT_TEST_OBJECTS) $(CASCADE_MSS_UNIT_TEST_OBJECTS_GEN)
	$($(MSS_CPU)_LD) $($(MSS_CPU)_LDFLAGS) $(CASCADE_MSS_UNIT_TEST_LOC_LIBS) -Wl,-m=$(CASCADE_MSS_UNIT_TEST_MAP) \
	-o $(CASCADE_MSS_UNIT_TEST_OUT) $(CASCADE_MSS_UNIT_TEST_OBJECTS) $(CASCADE_MSS_UNIT_TEST_OBJECTS_GEN) $(CASCADE_MSS_UNIT_TEST_STD_LIBS) \
	$(PLATFORM_$(MSS_CPU)_LINK_CMD) $(CASCADE_MSS_UNIT_TEST_APP_CMD)
	@echo '******************************************************************************'
	@echo 'Built the mmWave Cascade MSS Unit Test '
	@echo '******************************************************************************'

###################################################################################
# Cleanup Unit Test:
###################################################################################
mssTestClean:
	@echo 'Cleaning the mmWave Cascade MSS Unit Test objects'
	@$(DEL) $(CASCADE_MSS_UNIT_TEST_OBJECTS) $(CASCADE_MSS_UNIT_TEST_OUT) $(CASCADE_MSS_UNIT_TEST_BIN)
	@$(DEL) $(CASCADE_MSS_UNIT_TEST_MAP) $(CASCADE_MSS_UNIT_TEST_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(CASCADE_MSS_UNIT_TEST_DEPENDS)

