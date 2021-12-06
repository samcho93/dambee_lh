from subprocess import call


def SetVolume(vol):
  vol = "%d" % vol
  call(["/usr/bin/amixer", "-q", "set", "PCM", vol])