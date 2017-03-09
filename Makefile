PROJECT=ifcdaqdrv2

include $(EPICS_ENV_PATH)/module.Makefile

EXCLUDE_ARCHS = eldk56

SOURCES = $(wildcard src/*.c)
HEADERS = $(wildcard include/*.h)
