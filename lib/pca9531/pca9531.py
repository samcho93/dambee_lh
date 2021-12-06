import fcntl
import os
import time
import RPi.GPIO as GPIO

I2C_ADDRESS                    = 0x60
I2C_CHANNEL                    = 1

I2C_SLAVE                      = 1795

class I2CDevice:
    """Implements I2C device on ioctl"""
    def __init__(self, channel, addr):
        self.addr = addr
        self.i2c2 = os.open('/dev/i2c-%d' % channel, os.O_RDWR)
        if self.i2c2 < 0:
            raise RuntimeError('i2c2 device does not exist')
        #if fcntl.ioctl(self.i2c2, I2C_SLAVE, addr) < 0:
        #    raise RuntimeError('i2c slave does not exist')

    def write(self, buf):
        """Wrapper method of os.write"""
        return os.write(self.i2c2, buf)

    def read(self, count):
        """Wrapper method of os.read"""
        return os.read(self.i2c2, count)


class PCA9531:
    def __init__(self, irq=None, reset=None, req=None, debug=False):
      try:
          self.debug = debug
          self._irq = irq
          self._req = req
  
          GPIO.setmode(GPIO.BCM)
          self._i2c2 = I2CDevice(I2C_CHANNEL, I2C_ADDRESS)
  
          data = b'\x60\x01\x48'
          self._write_data(data)
          
          data = b'\x60\x04\xFF'
          self._write_data(data)
          
          data = b'\x60\x06\xFF'
          self._write_data(data)
          
      except:
          print("PCA9531 Error")        


    def _write_data(self, framebytes):
        try:
            self._i2c2.write(framebytes)
        except OSError as err:
            if self.debug:
                print(err)
            return

            
if __name__ == '__main__':
  pca9531 = PCA9531(debug=True)
  
  
  
  