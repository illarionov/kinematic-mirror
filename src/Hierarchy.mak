# FindUp finds matching files in the first higher level directory.
#   $(call FindUp,dir/pattern)
#   - stops when it finds a directory with a match
#   - can return multiple matches
#   - can match to subdirectory names
#   - if dir/ not specified, starts in current directory
FindUp = $(or $(filter $(notdir $1),$(notdir $(wildcard $(dir $1)*))),$(and $(realpath ../$(dir $1)),../$(call FindUp,../$1)))

# FindDown finds matching files in the subtree
#    $(call FindDown,dir/pattern)
#    - scans the entire subtree
#    - returns files, not directories 
#    - if dir/ not specified,starts in current directory
#
# FindDown(dir/pattern)
#   if (dir is not valid directory)
#       return dir
#   for each matching file in dir
#       return dir/ + FindDown(file/pattern)
#
FindDown = $(if $(realpath $(dir $1)),$(foreach f,$(wildcard $(dir $1)*),$(call FindDown,$f/$(notdir $1))),$(filter $(notdir $1),$(patsubst %/,%,$(dir $1))))

# The project root is the upper directory which contains Project.mak
PROJECT_ROOT := $(dir $(call FindUp,Project.mak))
MAKE_ROOT := $(dir $(call FindUp,Makefile.mak))

# Include the root makefiles if they exist
-include $(PROJECT_ROOT)Project.mak
-include $(MAKE_ROOT)Makefile.mak

