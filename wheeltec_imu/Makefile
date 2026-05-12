obj-m += wheeltec_imu_mod.o
wheeltec_imu_mod-objs := wheeltec_imu.o crc_tables.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	sudo mkdir -p /lib/modules/$(shell uname -r)/extra/
	sudo install -m 0644 wheeltec_imu_mod.ko /lib/modules/$(shell uname -r)/extra/
	sudo depmod -a

.PHONY: all clean install
