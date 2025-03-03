# import RPi.GPIO as GPIO
from gpiozero import LED
import time

def ZERO_test():
    # 创建一个 LED 对象，连接到 GPIO 17
    led = LED(18)
    num=0

    # 反转电平
    while True:
        print(num)
        num+=1
        led.toggle()
        time.sleep(1)

def main():
    # RPIGPIO_test()
    ZERO_test()

if(__name__=="__main__"):
    main()
