RUN_MODE = ROM

ifndef OPT
OPT = -Og
endif

MCU = SAM4S16B
BOARD = example
MMCULIB_DIR = ../../../mmculib

LIBS = ../../libs
INCLUDES += -I../../boards/$(BOARD)
VPATH += ../../boards/$(BOARD)

include $(MMCULIB_DIR)/mmculib.mk

