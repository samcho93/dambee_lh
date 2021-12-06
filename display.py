from PIL import ImageFont 
from time import sleep
import nfc
import command as pcmd

TFT = ""
draw = ""

fontfile = "/home/pi/dambee_lh/fonts/tway_air.ttf"

def DispInit(t, d):
    global TFT, draw
    TFT = t
    draw = d

def DispWait(init=False):
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("wait/wait.png")  
    TFT.display()
    if init == False:
      nfc.Initspi()

def DispOpen():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("wait/success.png")
    TFT.display()
    nfc.Initspi()

def DispFail():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("wait/fail.png")
    TFT.display()
    nfc.Initspi()
    
def DispCall():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/callbell.png")
    TFT.display()
    nfc.Initspi()

def DispCallFail():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/callfail.png")
    TFT.display()
    nfc.Initspi()
    
def DispCallWait():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/callwait.png")
    TFT.display()
    nfc.Initspi()

def DispCallEnd():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/callend.png")
    TFT.display()
    nfc.Initspi()

def DispCalling():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/calling.png")
    TFT.display()
    nfc.Initspi()
    
def DispRetry():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("call/retry.png")
    TFT.display()
    nfc.Initspi()
    
def DispNetworkDiscon():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("basic/basic3.png")
    TFT.display()
    nfc.Initspi()
    
def DispSocketDiscon():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("basic/basic4.png")
    TFT.display()
    nfc.Initspi()
    
def DispCoverOpen():
    global TFT
    TFT.initspi(20, 16, 12)
    TFT.load_wallpaper("basic/basic7.png")
    TFT.display()
    nfc.Initspi()
    
def DispGetinfo(sel):
    global TFT, draw
    img = ["setsysinfo/info.png", "setsysinfo/netinfo.png", "setsysinfo/setinfo.png"]
    TFT.load_wallpaper(img[sel])
    
    font = ImageFont.truetype(fontfile, 15)
    if sel == 0:
        draw.textrotated((45, 130), "dambee1", 90, font, fill='white')
        draw.textrotated((75, 155), pcmd.system["frmver"], 90, font, fill='white')
        draw.textrotated((105, 45), pcmd.system["plfmaddr"], 90, font, fill='white')
    elif sel == 1:
        draw.textrotated((45, 70), pcmd.system["mac"], 90, font, fill='white')
        draw.textrotated((75, 150), "DHCP", 90, font, fill='white')
        draw.textrotated((105, 80), pcmd.network["ipaddr"], 90, font, fill='white')
        draw.textrotated((135, 100), pcmd.network["netmask"], 90, font, fill='white')
        draw.textrotated((165, 95), pcmd.network["gateway"], 90, font, fill='white')
        draw.textrotated((195, 100), pcmd.network["dns"], 90, font, fill='white')
    elif sel == 2:
        draw.textrotated((45, 140), str(pcmd.option["opetime"]), 90, font, fill='white')
        draw.textrotated((75, 140), str(pcmd.option["lcd"]), 90, font, fill='white')
        draw.textrotated((105,140), str(pcmd.option["lcdtxt"]), 90, font, fill='white')
        draw.textrotated((135, 140), str(pcmd.option["spk"]), 90, font, fill='white')
        draw.textrotated((165, 140), str(pcmd.option["bluepwr"]), 90, font, fill='white')
        draw.textrotated((195, 140), str(pcmd.option["brkyn"]), 90, font, fill='white')

    TFT.display()
    
def DispTemp():
    global TFT
    TFT.load_wallpaper('setmanf/br.png')
    
    font = ImageFont.truetype(fontfile, 20)
    
    text = "BLE MAC : %s" %(str(pcmd.ble['mac']))
    draw.textrotated((45, 30), text, 90, font, fill='white')
    text = "IP Address : %s" %(str(pcmd.network['ipaddr']))
    draw.textrotated((90, 40), text, 90, font, fill='white')
    text = "S/W Ver. : v.%s" %(str(pcmd.system['frmver']))
    draw.textrotated((135, 100), text, 90, font, fill='white')
    
    TFT.display()
    
def DispTestColor():
    global TFT
    col = ['bk','br','cy','gr','pk','re','wh','ye']
    for c in col:
        file = "setmanf/%s.png" % c
        TFT.load_wallpaper(file)
        TFT.display()
        sleep(1)
    DispWait()

def DispBlepower():
    global TFT, draw
    font = ImageFont.truetype(fontfile, 25)
    TFT.load_wallpaper("setnorm/blepower.png")  
    draw.textrotated((165, 150), str(pcmd.option["bluepwr"]), 90, font, fill='white')
    TFT.display()
def DispBraket():
    global TFT, draw  
    stat = ["OFF", "ON"]
    TFT.load_wallpaper("setnorm/braket.png")  
    font = ImageFont.truetype(fontfile, 25)
    draw.textrotated((165, 140), stat[pcmd.option["brkyn"]], 90, font, fill='white')
    TFT.display()
def DispLcdbright():
    global TFT, draw
    TFT.load_wallpaper("setnorm/lcdbright.png")  
    font = ImageFont.truetype(fontfile, 25)
    draw.textrotated((165, 150), str(pcmd.option["lcd"]), 90, font, fill='white')
    TFT.display()
def DispLcdtext():
    global TFT, draw
    TFT.load_wallpaper("setnorm/lcdtext.png")  
    font = ImageFont.truetype(fontfile, 25)
    draw.textrotated((165, 150), str(pcmd.option["lcdtxt"]), 90, font, fill='white')
    TFT.display()
def DispOpentime():
    global TFT, draw
    TFT.load_wallpaper("setnorm/opentime.png")  
    font = ImageFont.truetype(fontfile, 25)
    draw.textrotated((165, 150), str(pcmd.option["opetime"]), 90, font, fill='white')
    TFT.display()
def DispVolume():
    global TFT, draw
    TFT.load_wallpaper("setnorm/volume.png")  
    font = ImageFont.truetype(fontfile, 25)
    draw.textrotated((160, 140), str(pcmd.option["spk"]), 90, font, fill='white')
    draw.textrotated((190, 140), str(pcmd.option["mic"]), 90, font, fill='white')
    TFT.display()
def DispHelp():
    print("\r\n+----------------------+")
    print("| HELP                 |")
    print("+----------------------+")
    print("| info                 |")
    print("| netinfo              |")
    print("| setinfo              |")
    print("| testcolor            |")
    print("| blepower             |")
    print("| braket               |")
    print("| lcdbright            |")
    print("| lcdtext              |")
    print("| opentime             |")
    print("| volume               |")
    print("| opentime             |")
    print("+----------------------+")
    print("| open/Invalid         |")
    print("| option/nework/system |")
    print("| setopentime          |")
    print("| setvol               |")
    print("| bright               |")
    print("| s0/s1/s2/s3          |")
    print("+----------------------+")

def DispTest(key):
    if key == '0':
        DispWait()
    elif key == '1':
        DispOpen()
    elif key == '2':
        DispFail()
    elif key == 'info':
        DispGetinfo(0)
    elif key == 'netinfo':
        DispGetinfo(1)
    elif key == 'setinfo':
        DispGetinfo(2)
    elif key == 'testcolor':
        DispTestColor()
    elif key == 'blepower':
        DispBlepower()
    elif key == 'braket':
        DispBraket()
    elif key == 'lcdbright':
        DispLcdbright()
    elif key == 'lcdtext':
        DispLcdtext()
    elif key == 'opentime':
        DispOpentime()
    elif key == 'volume':
        DispVolume()
    elif key == 'sys':
        DispTemp()
    elif key == 'help':
        DispHelp()
