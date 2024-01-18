ifneq ($(KERNELRELEASE),)
obj-m := serial.o ref.o
else
KDIR := $(HOME)/projects/linux-kernel-labs/src/linux
all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
