# patches to auto-configured config.mak

# EXTRA_INC is always including /usr/include 
EXTRA_INC = -I/home/lfu/nfsroot/usr/local/include

# Unresolved linker references
EXTRA_FLAGS = -D__strdup=strdup = -D__strtol=strtol

OPTFLAGS = -Wdeclaration-after-statement -O4   -pipe -ffast-math -fomit-frame-pointer -D_REENTRANT -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 $(EXTRA_INC) $(EXTRA_FLAGS)

# Additional video output drivers
VO_SRCS += vo_brio.c vo_brioyuv.c
