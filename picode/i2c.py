from .i2c import I2C

dev = I2C(address=0x14)
print(dev.is_ready())
