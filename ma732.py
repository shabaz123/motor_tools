####################################
# ma732.py
# rev 1 - October 2022 - shabaz
####################################


from machine import Pin, SPI
import time

# defines

# SPI bus
SPI_BUS = 0
SCK_PIN = Pin(18)
CS = Pin(17, Pin.OUT)
MOSI_PIN = Pin(19)
MISO_PIN = Pin(16)

# MA732 Registers
MA732_REG_Z0 = 0        # Zero setting bits 7:0
MA732_REG_Z1 = 1        # Zero setting bits 15:8
MA732_REG_BCT = 2       # Bias current trimming
MA732_REG_ETYX = 3      # Enable bias current trimming in X or Y direction
MA732_REG_PPT0_ILIP = 4  # Pulses per turn bits 1:0 for encoder mode, and index pulse setting
MA732_REG_PPT1 = 5      # Pulses per turn bits 9:2 for encoder mode
MA732_REG_MAG_THRESHOLDS = 6  # Mag field strength high and low threshold settings
MA732_REG_RD = 9        # Rotation direction setting
MA732_REG_FW = 14       # Measurement filter window size
MA732_REG_HYS = 16      # Rotary encoder output hysteresis
MA732_REG_THRESH_ALM = 27  # Mag field strength alarm flags

READ_REG_CMD = 0x40
WRITE_REG_CMD = 0x80

CW = 1
CCW = 0

CS.value(1)
spi = SPI(SPI_BUS, baudrate=1000000, sck=SCK_PIN, mosi=MOSI_PIN, miso=MISO_PIN)

def read_angle():
    CS.value(0)
    buf = spi.read(2)
    CS.value(1)
    rawval = (buf[0]<<8) | buf[1]
    angdeg = (rawval / 65536) * 360.0
    return angdeg
    
def read_reg(regnum):
    buf = bytes([regnum | READ_REG_CMD, 0])
    CS.value(0)
    spi.write(buf)
    CS.value(1)
    CS.value(0)
    buf = spi.read(2)
    CS.value(1)
    return buf[0]

def write_reg(regnum, value):
    buf = bytes([regnum | WRITE_REG_CMD, value])
    CS.value(0)
    spi.write(buf)
    CS.value(1)
    time.sleep(0.1) # delay for NV write
    CS.value(0)
    buf = spi.read(2) # dummy read
    CS.value(1)

def set_zero(angdeg):
    v = 65536 - ((angdeg / 360) * 65536)
    v = int(v)
    write_reg(MA732_REG_Z0, v & 0x00ff)
    write_reg(MA732_REG_Z1, (v >> 8) & 0x00ff)

    
