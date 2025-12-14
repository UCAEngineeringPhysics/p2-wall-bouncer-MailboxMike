from machine import Pin, Timer, Pin, PWM
from math import pi
from time import sleep_ms,sleep, sleep_us,ticks_us

from DualMotorDriver import DualMotorDriver_
from Diff_Driver import DiffDriver
from motor_driver import MotorDriver
from Sense_Distance import SenseDistance_
from Human_Interface import humanInterface_

button = Pin(22, Pin.IN, Pin.PULL_DOWN)
speed = 0
distance = None
tic = None
trig = Pin(9,Pin.OUT)
echo = Pin(8, Pin.IN, Pin.PULL_DOWN)
#HRI reqruitement
mod = 0
#LED
ledR = PWM(Pin(28))
ledG = PWM(Pin(27))
ledB = PWM(Pin(26))
ledY = PWM(Pin(18))
ledG.freq(1000)
ledB.freq(1000)
ledR.freq(1000)
ledY.freq(1000)

ledR.duty_u16(0)
ledG.duty_u16(0)
ledB.duty_u16(0)
ledY.duty_u16(0)


class wallBouncer_(SenseDistance_):
    global mod
    def __init__(self, right_ids: tuple, left_ids:tuple, stby_id: int):
        super().__init__(right_ids, left_ids, stby_id)
        """
        self.right_motor = DualMotorDriver_(*right_ids)  # unzip right_pins then feed to MotorDriver
        self.left_motor = MotorDriver_(*left_ids)  # unzip left_pins then feed to MotorDriver
        self.stby_pin = Pin(stby_id, Pin.OUT)
        """
   
    def setMode(self):
        if button.value() ==1:
            mod = humanInterface_.cycleMode(22)
            #mod+=1
            #print(mod)
            sleep(.5)
        else:
            mod = humanInterface_.cycleMode(22)
            #print(mod)
            return mod
            """
            if mod ==3:
                print(mod)
                return 2
            elif mod ==2:
                print(mod)
                return 3
            elif mod ==1 or mod ==0:
                print("mode = 0 or 1")
            """
        
    def setDir(self,speed):
        if speed >= 0:
            self.backward(0)
            self.forward(speed)
        elif speed < 0:
            self.forward(0)
            self.backward(speed*(-1))
        else:
            self.forward(0)
            self.backward(0)
            
    def computeDistance(pin):#didnt want to work as a called function
        global distance, tic
        #print("here")
        print(distance)
        if pin.value() == 1:
            tic =+ ticks_us()
        else:
            dur=ticks_us() - tic
            if dur < 100:
                distance = 0
            elif dur > 38000:
                distance = None
            elif 380000>dur >= 0:
                distance = dur/58/100 #unit: meters
            else:
                distance = None
        print(distance)
        return distance
echo.irq(handler=wallBouncer_.computeDistance)#, event=Pin.IRQ_RISING | Pin.IRQ_FALLING


while __name__ == "__main__":
    
    
    button.irq(trigger=Pin.IRQ_RISING, handler=wallBouncer_.setMode)
    wb = wallBouncer_(left_ids=(15,13,14), right_ids=(16,18,17), stby_id=12)
    dd = DiffDriver(left_ids=(15,13,14), right_ids=(16,18,17), stby_id=12)
    mode = wb.setMode()
    i = 1
    spinny = 0
    on=0
    wb.setDir(.0)
    if mode >1 and mode != None:
        #print("test1")
        if mode==2:
            #print("on")
            on+=1
            wb.setDir(.5)

    while mode == 2:
        mode = wb.setMode()
        #print("L2")
        trig.on()            
        sleep_ms(10)
        trig.off()
        #print("here")
        
        if distance != None and mode ==2:
            #print("L3")
            if distance <.5 and spinny ==0: #<.5
                ledY.duty_u16(10000)
                '''
                wb.setDir(-.25)
                sleep_ms(800)
                '''
                wb.setDir(0)
                dd.spinL(.5)
                spinny+=1
                sleep_ms(500)
            elif distance >=.8 and spinny ==1: #<.7
                ledY.duty_u16(0)
                dd.spinL(0)
                wb.setDir(.5)
                spinny-=1
            elif distance == None or distance > 1.2:
                ledY.duty_u16(10000)
                '''
                wb.setDir(-.25)
                sleep_ms(800)
                '''
                wb.setDir(0)
                dd.spinL(.5)
                sleep(.25)
                dd.spinL(0)
                wb.setDir(.5)
    if mode >1 and mode >2:
        #print("L4")
        wb.setDir(0)
        dd.spinL(0)
        dd.spinR(0)
            
        '''
        while distance <=.75:
        trig.on()     
        sleep_us(10)
        trig.off()
        dd.spinR(0)
        wb.setDir(0)
        '''
                    
        sleep_ms(60)#########################



