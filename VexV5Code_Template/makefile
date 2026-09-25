# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# use C++20
CXX_FLAGS := $(filter-out -std=%,$(CXX_FLAGS)) -std=gnu++2a

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)
SRC_C += $(wildcard src/*/*/*.cpp) 
SRC_C += $(wildcard src/*/*/*.c)
SRC_C += $(wildcard src/*/*/*/*.cpp) 
SRC_C += $(wildcard src/*/*/*/*.c)

rwildcard = $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )
EXT_OBJ := $(call rwildcard,obj/,*.o)
OBJ += $(EXT_OBJ)


# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
