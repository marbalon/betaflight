F1_TARGETS  += $(TARGET)
256K_TARGETS += $(TARGET)
FEATURES    = ONBOARDFLASH HIGHEND
FLASH_SIZE = 256
DENISTY = hd
DEVICE_FLAGS    = -DSTM32F10X_HD
HSE_VALUE       = 16000000

TARGET_SRC = \
            io/osd.c \
            drivers/accgyro_mpu.c \
            drivers/accgyro_spi_mpu6000.c \
            drivers/flash_m25p16.c \
            drivers/max7456.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f10x.c
