#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := otis-imu

EXTRA_COMPONENT_DIRS = main/hal

include $(IDF_PATH)/make/project.mk

