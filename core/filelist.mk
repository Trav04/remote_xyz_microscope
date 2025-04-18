# Including memory heap model
LIBSRCS += $(FREERTOS_PATH)/portable/MemMang/heap_4.c

# Set mylib folder path.
MYLIB_PATH=$(SOURCELIB_ROOT)/../repo/mylib

# Set folder path with header files to include.
CFLAGS += -I$(MYLIB_PATH)

# List all c file locations that must be included (use space as separator
LIBSRCS += $(MYLIB_PATH)/lta1000g.c $(MYLIB_PATH)/mfs_pb.c $(MYLIB_PATH)/rgb.c $(MYLIB_PATH)/mfs_trimpot.c $(MYLIB_PATH)/mfs_led.c $(MYLIB_PATH)/switchbank.c $(MYLIB_PATH)/hamming.c $(MYLIB_PATH)/rcmsys.c $(MYLIB_PATH)/txradio.c $(MYLIB_PATH)/cmdin.c $(MYLIB_PATH)/ssd.c $(MYLIB_PATH)/rcmext.c
