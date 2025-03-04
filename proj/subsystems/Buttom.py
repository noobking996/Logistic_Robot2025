from gpiozero import Button
import time

class myButtom:
    def __init__(self,pin,poll_interval:float):
        """
        * 创建按键对象(低电平触发)
        @param pin: 按键引脚号
        @param poll_interval: 轮询间隔
        """
        self.buttom=Button(pin)
        self.status=False
        self.t0=None
        self.interval=poll_interval

    def Poll(self)->bool:
        """
        * 轮询按键状态
        ### returns
        按键按下标志位
        """
        pressed_flag=False
        if(self.status==False):
            self.status=True
            self.t0=time.time()
        else:
            if(time.time()-self.t0>=self.interval):
                self.status=False
                pressed_flag=self.buttom.is_active
        return pressed_flag

def main():
    buttom=myButtom(14,0.1)
    while True:
        if(buttom.Poll()==True):
            print("按键按下")

if(__name__=="__main__"):
    main()