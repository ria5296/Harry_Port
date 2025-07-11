from machine import I2C, Pin
import time
from i2cslave import I2CSlave  # 외부 라이브러리 필요

# MPU9250 레지스터 주소
PWR_MGM_1      = 0x6B

ACCEL_XOUT_H   = 0x3B
ACCEL_XOUT_L   = 0x3C
ACCEL_YOUT_H   = 0x3D
ACCEL_YOUT_L   = 0x3E
ACCEL_ZOUT_H   = 0x3F
ACCEL_ZOUT_L   = 0x40

GYRO_XOUT_H    = 0x43
GYRO_XOUT_L    = 0x44
GYRO_YOUT_H    = 0x45
GYRO_YOUT_L    = 0x46
GYRO_ZOUT_H    = 0x47
GYRO_ZOUT_L    = 0x48

class MPU9250:
    def __init__(self, i2c, address=0x68):
        self.i2c = i2c
        self.address = address
        self._write_byte(PWR_MGM_1, 0x00)  # 센서 활성화

    def read_gyro(self):
        gx = self._read_word(GYRO_XOUT_H)
        gy = self._read_word(GYRO_YOUT_H)
        gz = self._read_word(GYRO_ZOUT_H)
        return gx, gy, gz

    def read_accel(self):
        ax = self._read_word(ACCEL_XOUT_H)
        ay = self._read_word(ACCEL_YOUT_H)
        az = self._read_word(ACCEL_ZOUT_H)
        return ax, ay, az

    def _write_byte(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def _read_byte(self, reg):
        return int.from_bytes(self.i2c.readfrom_mem(self.address, reg, 1), 'big')

    def _read_word(self, reg):
        high = self._read_byte(reg)
        low = self._read_byte(reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

# I2C 초기화 (4번: SDA, 5번: SCL)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
mpu = MPU9250(i2c)

# 변환 상수
ACCEL_SENSITIVITY = 16384  # LSB/g (±2g)
GYRO_SENSITIVITY = 131     # LSB/(°/s) (±250°/s)
GRAVITY = 9.80665          # m/s²

# 데이터 읽기 및 변환 출력 예시
while True:
    ax_raw, ay_raw, az_raw = mpu.read_accel()
    gx_raw, gy_raw, gz_raw = mpu.read_gyro()

    # 가속도 변환 (m/s²)
    ax = (ax_raw / ACCEL_SENSITIVITY) * GRAVITY
    ay = (ay_raw / ACCEL_SENSITIVITY) * GRAVITY
    az = (az_raw / ACCEL_SENSITIVITY) * GRAVITY

    # 자이로 변환 (°/s)
    gx = gx_raw / GYRO_SENSITIVITY
    gy = gy_raw / GYRO_SENSITIVITY
    gz = gz_raw / GYRO_SENSITIVITY

    print('Accel: {:.3f} {:.3f} {:.3f} m/s²  Gyro: {:.3f} {:.3f} {:.3f} °/s'.format(ax, ay, az, gx, gy, gz))
    time.sleep(0.5)
