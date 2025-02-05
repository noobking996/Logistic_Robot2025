from enum import Enum

class Keyboard_Enum(Enum):
    """
    @功能: 枚举部分键盘上非字母按键的值
    @注意事项: 1. 测试用键盘为狼蛛F75;
              2. CTRL键值无法读取;
              3. 引用该枚举时,注意调用方式: Keyboard_Enum.按键名.value
    """
    LEFT_ARROW = 81
    RIGHT_ARROW = 83
    UP_ARROW = 82
    DOWN_ARROW = 84

    SPACE = 32
    ESC = 27
    TAB=9
    Enter=13
    SHIFT=225
    
    ALT=233
    BACKSPACE=8