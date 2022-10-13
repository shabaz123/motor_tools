####################################
# mps_motor.py
# rev 1 - October 2022 - shabaz
####################################


from machine import Pin, UART
import utime


# defines
MODE_POS = 0
MODE_SPEED = 1
MODE_TORQUE = 2
CW = 1
CCW = 0
ZERO_TURNS = 0
# common registers for gen1 motors
MSM_G1_REG_MODE = 0x34         # Control Mode[0:1]
MSM_G1_REG_ENABLE = 0x70       # motors enable
MSM_G1_REG_SPD_H = 0x4D        # speed reference - High
MSM_G1_REG_SPD_L = 0x4E        # speed reference - Low
MSM_G1_REG_UPDATE = 0x76       # command update
MSM_G1_REG_TURNS = 0x4A        # Target Position (High byte) - number of turns or revolution
MSM_G1_REG_DEGREES = 0x4B      # Target Position (Low byte) - angle in degrees
MSM_G1_REG_POS_SLOPE = 0x4C    # position reference - slope (acceleration)
MSM_G1_REG_PLOOP_LIM = 0x1C    # position loop limit (default 0x0147) (Speedlimit under Pos_loop)
MSM_G1_REG_IQ_CMD = 0x2F       # 12bit register - Target torque
MSM_G1_REG_AD_GAIN = 0x59      # AD_Gain
MSM_G1_REG_IQ_NOMINAL = 0x0A   # Nominal Motor rated IQ current
MSM_G1_REG_RATED_POWER = 0x03  # Motor Power Rating
MSM_G1_REG_RATED_SPEED = 0x05  # Motor Rated Speed
# constants for motors
MSM_SPD_REG2RPM = 0.00013970
MSM_SPD_RPM2REG = 7158.3
MSM_SLP_REG2RPM = 9.155
MSM_SLP_RPM2REG = 0.1092
MSM_POS_DEG2REG = 182.04  # 2^16/360

# built-in LED
boardled = Pin(25, Pin.OUT)
dir485 = Pin(5, Pin.OUT)
motor_slave_addr = 0x00

motor_uart = UART(1, baudrate=115200, bits=8, parity=1, stop=1, tx=Pin(8), rx=Pin(9))

def uart_reset():
    motor_uart.deinit()
    utime.sleep(0.1)
    motor_uart.init()
    utime.sleep(0.5)

def set_slave_addr(addr):
    global motor_slave_addr
    motor_slave_addr = addr

def uart_tx_flush():
    utime.sleep(0.5)  # MicroPython has no flush capability

def write_reg(addr, val):
    dir485.value(1)  # transmit
    msg = bytearray(4)
    msg[0] = (motor_slave_addr << 1) | 0x01
    msg[1] = (addr << 1) | 0x01
    msg[2] = val & 0x00ff
    msg[3] = (val & 0xff00) >> 8
    res_ignore = motor_uart.write(msg)
    uart_tx_flush()
    dir485.value(0)

def read_reg(addr):
    numread = 0
    dir485.value(1)  # transmit
    msg = bytearray(2)
    discbyte = 0
    motor_uart.read()  # flush the input
    msg[0] = (motor_slave_addr << 1) | 0x01
    msg[1] = (addr << 1) & 0xfe
    res_ignore = motor_uart.write(msg)
    uart_tx_flush()
    dir485.value(0)  # receive
    rbuf = bytearray(10)
    numread = motor_uart.readinto(rbuf, 2)
    if numread == 2:
        val = rbuf[0] + (rbuf[1]<<8)
        success = 1
    else:
        print(f"error, numread is {numread} instead of 2!")
        val = -1
    return (val)

def op_mode(mode):
    if mode == MODE_POS:
        val = 0x11
    elif mode == MODE_SPEED:
        val = 0x00
    elif mode == MODE_TORQUE:
        val = 0x02
    else:
        val = 0x11
    write_reg(MSM_G1_REG_MODE, val)
    utime.sleep(0.5)

def enable_op():
    write_reg(MSM_G1_REG_ENABLE, 1)

def disable_op():
    write_reg(MSM_G1_REG_ENABLE, 0)

def set_slope(slope_rpm):
    v = int(MSM_SLP_RPM2REG * slope_rpm)
    if v<=0:
        v = 1
    write_reg(MSM_G1_REG_POS_SLOPE, v)

def set_position(direction, turns, angle):
    reg32 = (turns + (angle/360))*pow(2,16)
    if direction==0:
        reg32 = pow(2,32)-reg32
    reg32 = int(reg32)
    write_reg(MSM_G1_REG_TURNS, (reg32 & 0xffff0000)>>16)
    write_reg(MSM_G1_REG_DEGREES, reg32 & 0x0000ffff)
    write_reg(MSM_G1_REG_UPDATE, 0)
    
def get_max_rpm():
    speed = read_reg(MSM_G1_REG_RATED_SPEED)
    return(speed)

def set_velocity(rpm):
    v = int(MSM_SPD_RPM2REG * rpm)
    # write_reg(MSM_G1_REG_SPD_H, (v & 0xffff0000)>>16)
    write_reg(MSM_G1_REG_SPD_H, (v >> 16) & 0x0000ffff)
    write_reg(MSM_G1_REG_SPD_L, v & 0x0000ffff)
    write_reg(MSM_G1_REG_UPDATE, 0)

    
