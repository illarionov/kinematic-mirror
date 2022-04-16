
# Pick the current OS name
MYOS:=$(shell uname -s | tr A-Z a-z)
MYARCH:=$(shell arch)

# By default, pick the current architecture and OS
OS:=$(MYOS)
ARCH:=$(MYARCH)

# If we are cross compiling, parse the desired OS and ARCH, look for build env
ifneq ($(TARGET),)
    OS:=openwrt
    ARCH:=mips
    CROSS:=~/$(OS)/trunk/staging_dir/toolchain-$(ARCH)_gcc-4.3.3+cs_uClibc-0.9.30.1
    CROSSBIN:=$(CROSS)/usr/bin/$(ARCH)-$(OS)-$(MYOS)-
endif


AR:=$(CROSSBIN)ar
LD:=$(CROSSBIN)ld
GCC:=$(CROSSBIN)gcc
CC:=$(GCC)
CXX:=$(CROSSBIN)g++

# Where to put generated files
OBJDIR := $(PROJECT_ROOT)/../Obj/$(ARCH)-$(OS)/
BINDIR := $(PROJECT_ROOT)/../Bin/$(ARCH)-$(OS)/
INCDIR := $(PROJECT_ROOT)/../Include/

#CPPOPT:= -Os 
#LDOPT:= -s 
CPPOPT:= -g
LDOPT := -g

CPPFLAGS:= -I $(CROSS)/usr/include -I $(CROSS)/include $(CPPOPT)
CFLAGS:=$(CPPFLAGS) -DSQLITE_OMIT_LOAD_EXTENSION  -DSQLITE_THREADSAFE=0
LDFLAGS:= -L $(CROSS)/usr/lib -L $(CROSS)/lib  #-lpthread

.SUFFIXES : .cpp .c .o .lib .exe .h .dll .a

# We are linking C++, not C
#  (take the normal link rule and change from $(CC) to $(CXX) )
LINK.o = $(CXX) $(LDFLAGS) $(TARGET_ARCH)


# When linking executibles, use the Kinematic library. Won't work for building tools needed to create library.
LDLIBS += $(BINDIR)Kinematic.a


# objs := CompileTree <srcdir> <objdir> <includedirs>
CompileTree = $(foreach s,$(call FindDown,$1),$(call Compile,$s,$2,$3))

# obj := Compile <src> <objdir> <includedirs>
Compile = $(call CreateDir,$(call CreateRule,$(call ObjectName,$2,$1),$1,$$(COMPILE)))

define COMPILE
        $(COMPILE$(suffix $<)) $(OUTPUT_OPTION) $<
endef

# targets := CreateRule <targets> <dependencies> <body>
CreateRule = $(eval $1:$2;$3)$1

# Get the object file name from the source name
#   name = ObjectName <object dir> <source name>
ObjectName = $1$(basename $2).o

# Headers
HeaderTree = $(foreach h,$(call FindDown,$(1)%.h),$(call CreateHeader,$h,$2))

CreateHeader = $(call CreateDir,$(call CreateRule,$2$(notdir $1),$1,cp $1 $2$(notdir $1)))


# Directories
# <file name> = CreateDir <file name>
#  WORKS, BUT GENERATES EXCESSIVE COMMANDS. NEED TO FIX
CreateDir = $(eval $1 : | $(dir $1))$(call eval,$(dir $1) :: ; @ mkdir -p $(dir $1))$1


# 
CPPFLAGS += -I$(INCDIR) 
