obj-m += mipp_pen0.o

# kleaf invokes `make -C <this dir>` with M=/O=/KERNEL_SRC=/TOOL_ARGS= on the
# command line; command-line variables propagate to this recursive make via
# MAKEFLAGS, so do NOT override them here -- kbuild must place the artifacts
# under $(O)/$(M) where kleaf expects them.
all modules:
	$(MAKE) -C $(KERNEL_SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) modules_install

.PHONY: all modules modules_install
