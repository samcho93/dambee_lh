from pygame import mixer

soundok = False

def DEBUGPrint(msg, param1="", param2=""):
    print("[SOUND]", msg, param1, param2)
     
def Init(vol):
    global soundok
    try:
      if soundok == False:
          soundok = True
          mixer.init()
          #SetVol(1)
                 
          print("Sound Mixer Init.")
    except Exception as e:        
        soundok = False
        print(e)
        

def Play(file, repeat):
    global soundok
    if soundok == True:
      try:
        filename = "/home/pi/dambee_lh/music/"+file
        DEBUGPrint("Play Music : ", filename)
        mixer.music.load(filename)
        mixer.music.play(repeat)
      except:
        soundok = False
        DEBUGPrint("Error MP3 Play")          

def Stop():
    global soundok
    if soundok == True:
      try:
        DEBUGPrint("Stop Music")
        mixer.music.stop()
      except:
        DEBUGPrint("Error MP3 Stop")

def SetVol(vol):
    global soundok
    if soundok == True:
        mixer.music.set_volume(vol)
        
def Quit():
    global soundok
    if soundok == True:
      try:
        DEBUGPrint("Quit Sound")
        soundok = False
        mixer.quit()
      except:
        DEBUGPrint("Error Quit Sound")
        