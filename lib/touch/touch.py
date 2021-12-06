import fcntl
import os
import time
import RPi.GPIO as GPIO

I2C_ADDRESS                    = 0x6a
I2C_CHANNEL                    = 1

I2C_SLAVE                      = 1795

class I2CDevice:
    """Implements I2C device on ioctl"""
    def __init__(self, channel, addr):
        self.addr = addr
        self.i2c = os.open('/dev/i2c-%d' % channel, os.O_RDWR)
        if self.i2c < 0:
            raise RuntimeError('i2c device does not exist')
        if fcntl.ioctl(self.i2c, I2C_SLAVE, addr) < 0:
            raise RuntimeError('i2c slave does not exist')

    def write(self, buf):
        """Wrapper method of os.write"""
        return os.write(self.i2c, buf)

    def read(self, count):
        """Wrapper method of os.read"""
        return os.read(self.i2c, count)


class TS20:
    def __init__(self, irq=None, reset=None, req=None, debug=False):
      try:
          self.debug = debug
          self._irq = irq
          self._req = req
          
  
          GPIO.setmode(GPIO.BCM)
          # With I2C, we recommend connecting RSTPD_N (reset) to a digital pin for manual
          # harware reset
          #GPIO.setup(reset, GPIO.OUT)
          # On Raspberry Pi, you must also connect a pin to P32 "H_Request" for hardware
          # wakeup! this means we don't need to do the I2C clock-stretch thing
          self._i2c = I2CDevice(I2C_CHANNEL, I2C_ADDRESS)
  
          data = b'\x0b\x52'
          self._write_data(data)
          
          data = b'\x0C\x22'
          self._write_data(data)
          
          data = b'\x0E\x00' # 1 ~ 4
          self._write_data(data)
          
          data = b'\x0F\x00' # 5 ~ 7
          self._write_data(data)
          
          data = b'\x10\x00' # 8 ~ 11
          self._write_data(data)
          
          data = b'\x11\x00' # 12 ~ 15
          self._write_data(data)

          
          data = b'\x00\x66'
          self._write_data(data)
          
          data = b'\x01\x33'
          self._write_data(data)
          
          data = b'\x02\x33'
          self._write_data(data)
          
          data = b'\x03\x33'
          self._write_data(data)
          
          data = b'\x04\x33'
          self._write_data(data)
          
          data = b'\x05\x66'
          self._write_data(data)
          
          data = b'\x06\x66'
          self._write_data(data)
          
          data = b'\x07\x66'
          self._write_data(data)

           
          
          data = b'\x14\x00'
          self._write_data(data)
          
          data = b'\x15\x00'
          self._write_data(data)
          
          data = b'\x16\x00'
          self._write_data(data)
          
          
          data = b'\x17\x1F'
          self._write_data(data)
          
          
          data = b'\x0D\xFA'
          self._write_data(data)
          

      except:
          print("TS20 Error")        

    def _gpio_init(self, reset, irq=None, req=None):
        self._irq = irq
        self._req = req
        GPIO.setmode(GPIO.BCM)
        if reset:
            GPIO.setup(reset, GPIO.OUT)
            GPIO.output(reset, True)
        if irq:
            GPIO.setup(irq, GPIO.IN)
        if req:
            GPIO.setup(req, GPIO.OUT)
            GPIO.output(req, True)

    def _read_data(self, count):
        try:
            frame = bytes(self._i2c.read(count))
        except OSError as err:
            if self.debug:
                print(err)
            return

        if self.debug:
            print("Reading: ", [hex(i) for i in frame[1:]])
        
        return frame   # don't return the status byte

    def _write_data(self, framebytes):
        try:
            self._i2c.write(framebytes)
        except OSError as err:
            if self.debug:
                print(err)
            return
        
    def getTouch(self):
        try:
            data = b'\x20'
            self._write_data(data)
            touch = self._i2c.read(3)      
            if touch[2] & 0x20 == 0:
              return touch[0] + touch[1] * 256
            else:
              return 0 
        except:
            return 0 
            
if __name__ == '__main__':
  ts20 = TS20(debug=False)
  
  while True:
    try:
        touch = ts20.getTouch()
        if touch != 0:
          print(hex(touch))
          time.sleep(0.1)
    except  KeyboardInterrupt:
        break 
  