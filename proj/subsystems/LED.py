import RPi.GPIO as GPIO
import time

# 设置GPIO模式为BCM（Broadcom SOC channel）
GPIO.setmode(GPIO.BCM)

# 设置GPIO引脚17为输出模式
GPIO.setup(17, GPIO.OUT)

try:
    while True:
        # 点亮LED
        GPIO.output(17, GPIO.HIGH)
        print("LED is ON")
        time.sleep(1)  # 等待1秒
        
        # 熄灭LED
        GPIO.output(17, GPIO.LOW)
        print("LED is OFF")
        time.sleep(1)  # 等待1秒
except KeyboardInterrupt:
    # 当按下Ctrl+C时，清理GPIO设置
    GPIO.cleanup()
    print("GPIO cleanup completed")
