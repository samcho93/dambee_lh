#-*-coding:utf-8-*-
#from Cryptodome import Random   
from Crypto.Cipher import AES 

from socket import *
import json
from ast import literal_eval

BLOCK_SIZE=8
iv = "9873CE4DAB0BCF6F"
key = "20C0BD9B84CFF79873CE4DAB0BCF6F21"

class AESCryptoCBC():
    def __init__(self, key, iv):
        lkey = []
        liv = []
        for i in key:
            lkey.append(ord(i))

        for i in iv:
            liv.append(ord(i))

        self.crypto = AES.new(bytes(lkey), AES.MODE_CBC, bytes(liv))

    def encrypt(self, data):
        ldata = []
        for i in data:
            ldata.append(ord(i))
        
        a = len(ldata) % 16
        if a != 0:
            for i in range(16-a):
                ldata.append(16-a)

        enc = self.crypto.encrypt(bytes(ldata))

        return enc

    def decrypt(self, enc):
        ldata = []
        for i in enc:
            ldata.append(i)

        a = len(ldata) % 16
        if a != 0:
            for i in range(16-a):
                ldata.append(16-a)

        dec = self.crypto.decrypt(bytes(ldata))
        temp = []
        for i in dec:
            if i > 16:
                temp.append(chr(i))

        string = "".join(temp)
        return string

if __name__ == "__main__":
    data = '{"method":"getDeviceFcltInfo","macAddr":"CF:78:4D:10:83:10"}'
    #data = '{"method":"getAuthKeyInfo", "orgnztSn":1,"fcltSn":1}'
    #data = '{"method":"setMapXY","orgnztSn":1,"fcltSn":1,"mapX":"32.52024821","mapY":"126.359038"}'
    #data = '{"method":"setError","orgnztSn":1,"fcltSn":1,"errorCode":0}'

    # Encoding
    aes = AESCryptoCBC(key, iv)
    enc = aes.encrypt(data)

    #enc = data
    #print("The encrypted value is " + str(list(enc)))
    
    port = 17005
    server_ip = '58.124.36.107'
    #port = 9102
    #server_ip = '192.168.0.4'
    clientSock = socket(AF_INET, SOCK_STREAM)
    clientSock.connect((server_ip, port))

    clientSock.send(enc)
    recvData = clientSock.recv(1024) 
    #print('from Server > ', recvData.decode('utf-8'))

    #키 생성
    # Decoding
    aes2 = AESCryptoCBC(key, iv)
    dec = aes2.decrypt(recvData)
    print(dec)

    #data = json.loads(dec)
    #print(type(data), str(data))
