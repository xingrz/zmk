# DOIO KB16-01

![DOIO KB16-01](https://img.zfrontier.com/cover/20220310/FlSjU2uwwwuRZ4dz0V8jun0Fj1u-)

## Build

```
west build -p -b kb16
```

## Flash

You need to disassemble the keyboard and soild GND/SWCLK/SWDIO to a DAPLink probe.

[Device Support pack for STM32F1](https://www.keil.com/dd2/pack/) is also required.

```
west flash -r pyocd --flash-opt="--pack=/path/to/Keil.STM32F1xx_DFP.2.4.0.pack"
```

## Logging

```
pyocd rtt --pack /path/to/Keil.STM32F1xx_DFP.2.4.0.pack -t stm32f103cb -M attach
```
