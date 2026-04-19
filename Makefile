TOOLCHAIN_BIN ?= /Users/litianyi/Desktop/MounRiver Studio 2.app/Contents/Resources/app/resources/darwin/components/WCH/Toolchain/RISC-V Embedded GCC12/bin
export PATH := $(TOOLCHAIN_BIN):$(PATH)

.PHONY: all v5f clean

all: v5f

v5f:
	$(MAKE) -C obj all

clean:
	$(MAKE) -C obj clean
