import RPi.GPIO as GPIO

class gp():
    def __init__(self):
      GPIO.setmode(GPIO.BCM)
      self.rly = 26
      GPIO.setup(self.rly, GPIO.OUT)
      GPIO.output(self.rly, GPIO.LOW)
     
      self.in0 = 13
      self.in1 = 6
      self.hallsen = 19
      self.faultState = 22
      self.charge = 27
      self.powerFail = 17
      self.forceReset = 24
      
      GPIO.setup(self.in0, GPIO.IN)
      GPIO.setup(self.in1, GPIO.IN)
      GPIO.setup(self.hallsen, GPIO.IN)
      GPIO.setup(self.faultState, GPIO.IN)
      GPIO.setup(self.charge, GPIO.IN)
      GPIO.setup(self.powerFail, GPIO.IN)
      GPIO.setup(self.forceReset, GPIO.IN, pull_up_down = GPIO.PUD_UP)
      

      self.blwakeup = 18
      GPIO.setup(self.blwakeup, GPIO.OUT)
      GPIO.output(self.blwakeup, GPIO.HIGH)
          
      self.gled = 25
      self.rled = 23
      #self.pwm = 100
      GPIO.setup(self.gled, GPIO.OUT)
      GPIO.output(self.gled, GPIO.HIGH)
      #self.pgled = GPIO.PWM(self.gled, 1000)
      #self.pgled.start(100)
      #self.pgled.ChangeDutyCycle(0)
      
      GPIO.setup(self.rled, GPIO.OUT)
      GPIO.output(self.rled, GPIO.HIGH)
      #self.prled = GPIO.PWM(self.rled, 1000)
      #self.prled.start(100)
      #self.prled.ChangeDutyCycle(self.pwm)            

    def relay(self, state):
      if state == 0:
          GPIO.output(self.rly, GPIO.LOW)
      elif state == 1:
          GPIO.output(self.rly, GPIO.HIGH)

    def blewakeup(self, state):
      if state == 0:
          GPIO.output(self.blwakeup, GPIO.LOW)
      elif state == 1:
          GPIO.output(self.blwakeup, GPIO.HIGH)
            
    def fled(self, state):
      if state == 0:
        GPIO.output(self.gled, GPIO.HIGH)
        GPIO.output(self.rled, GPIO.HIGH)
        #self.prled.ChangeDutyCycle(100)
        #self.pgled.ChangeDutyCycle(100)
      elif state == 1:
        GPIO.output(self.gled, GPIO.LOW)
        GPIO.output(self.rled, GPIO.HIGH)
        #self.prled.ChangeDutyCycle(100)
        #self.pgled.ChangeDutyCycle(100-self.pwm)
      elif state == 2:
        GPIO.output(self.gled, GPIO.HIGH)
        GPIO.output(self.rled, GPIO.LOW)
        #self.prled.ChangeDutyCycle(100-self.pwm)
        #self.pgled.ChangeDutyCycle(100)
    
    def pwmgled(self, value):
      self.pwm = value
      
    def close(self):
      GPIO.cleanup()
    
    