# Raspberry Pi 5B rev 1.0 Pinout
## Generic
```
Description        : Raspberry Pi 5B rev 1.0
Revision           : c04170
SoC                : BCM2712
RAM                : 4GB
Storage            : MicroSD
USB ports          : 4 (of which 2 USB3)
Ethernet ports     : 1 (1000Mbps max. speed)
Wi-fi              : True
Bluetooth          : True
Camera ports (CSI) : 2
Display ports (DSI): 2

,--------------------------------.
| oooooooooooooooooooo J8   : +====
| 1ooooooooooooooooooo      : |USB2
|  Wi  Pi Model 5B  V1.0  fan +====
|  Fi     +---+      +---+       |
|         |RAM|      |RP1|    +====
||p       +---+      +---+    |USB3
||c      -------              +====
||i        SoC      |c|c J14     |
(        -------  J7|s|s 12 +======
|  J2 bat   uart   1|i|i oo |   Net
| pwr\..|hd|...|hd|o|1|0    +======
`-| |-1o|m0|---|m1|--------------'

J8:
       3V3  (1) (2)  5V    
     GPIO2  (3) (4)  5V    
     GPIO3  (5) (6)  GND   
   # GPIO4  (7) (8)  GPIO14 #
       GND  (9) (10) GPIO15
    GPIO17 (11) (12) GPIO18 #
    GPIO27 (13) (14) GND   
    GPIO22 (15) (16) GPIO23 #
       3V3 (17) (18) GPIO24 #
    GPIO10 (19) (20) GND   
   # GPIO9 (21) (22) GPIO25
    GPIO11 (23) (24) GPIO8 # 
       GND (25) (26) GPIO7 
   # GPIO0 (27) (28) GPIO1 #
   # GPIO5 (29) (30) GND   
     GPIO6 (31) (32) GPIO12 #
  # GPIO13 (33) (34) GND   
    GPIO19 (35) (36) GPIO16
    GPIO26 (37) (38) GPIO20
       GND (39) (40) GPIO21

J2:
RUN (1)
GND (2)

J7:
COMPOSITE (1)
      GND (2)

J14:
TR01 TAP (1) (2) TR00 TAP
TR03 TAP (3) (4) TR02 TAP

For further information, please refer to https://pinout.xyz/
```

## Alter-Function
* TTL-Serial (Tx/Rx)
   * 主控板通信：UART2 (GPIO4/GPIO5)
   * 机械臂控制: UART4 (GPIO12/GPIO13)
   * 备用串口: 
      *  UART1 (GPIO0/GPIO1)
      *  UART3 (GPIO8/GPIO9)

* RGB-LED
   * R: GPIO18
   * G: GPIO23
   * B: GPIO24

* Launched-Button
   * 启动按钮: GPIO14