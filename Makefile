KERNELDIR := /home/liu/桌面/project/ebf_linux_kernel
CURRENT_PATH := $(shell pwd)
obj-m := ICM20602.o
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
copy:
	sudo cp *.ko /srv/nfs/workdir