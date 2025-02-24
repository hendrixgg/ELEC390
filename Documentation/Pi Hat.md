# I2C Interface
Everything on the Pi Hat is controlled over I2C
- Uses I2C Bus Address 1 (i2c-1)
- I2C Device Address `0x14`

## ADC Interface
- ADC Channel should be between 0-7
- I2C Register Addr: `(7-channel) | 0x10`
- To Read:
	1. Write `[reg, 0, 0]`
	2. Read `[msb, lsb]`
- Voltage = `value*3.3/4095`

## PWM
```
REG_CHN = 0x20
"""Channel register prefix"""
REG_PSC = 0x40
"""Prescaler register prefix"""
REG_ARR = 0x44
"""Period registor prefix"""
REG_PSC2 = 0x50
"""Prescaler register prefix"""
REG_ARR2 = 0x54
"""Period registor prefix"""

CLOCK = 72000000.0
"""Clock frequency"""
```

- reg = prescaler + timer

# Pin Mappings
```
servo_pins:list=['P0', 'P1', 'P2'], 
motor_pins:list=['D4', 'D5', 'P13', 'P12'],
grayscale_pins:list=['A0', 'A1', 'A2'],
ultrasonic_pins:list=['D2','D3'],
```