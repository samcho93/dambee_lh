
from datetime import datetime

filename = ""
flog = False

def Init(flag = False):
  global filename, flog
  
  flog = flag
  
  if flog == True:
    now = datetime.now()
    filename = now.strftime('/home/pi/dambee_lh/log/log%Y%m%d%H%M%S.txt')
    print(filename)
  
def Log(string):
  global filename, flog
  
  if flog == True:
    f = open(filename, mode="at", encoding='utf-8')
    now = datetime.now()
    time = now.strftime('[%Y-%m-%d %H:%M:%S]')
    f.writelines(time + string +'\n')
    f.close()