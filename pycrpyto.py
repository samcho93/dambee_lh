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
    data1 = '{"method":"getDeviceFcltInfo","macAddr":"D3:68:BF:B2:F2:5C"}'
    data2 = '{"method":"getAuthKeyInfo", "orgnztSn":1,"fcltSn":1}'
    data3 = '{"method":"setMapXY","orgnztSn":1,"fcltSn":1,"mapX":"32.52024821","mapY":"126.359038"}'
    data4 = '{"method":"setError","orgnztSn":1,"fcltSn":1,"errorCode":0}'

    # Encoding
    aes = AESCryptoCBC(key, iv)
    aes2 = AESCryptoCBC(key, iv)
    
    
    enc = aes.encrypt(data1)
    dec = aes2.decrypt(enc)
    print(dec)

    enc = aes.encrypt(data2)
    dec = aes2.decrypt(enc)
    print(dec)  
    
    enc = aes.encrypt(data3)
    dec = aes2.decrypt(enc)
    print(dec)
    
    enc = aes.encrypt(data4)
    dec = aes2.decrypt(enc)
    print(dec)    
    port = 17005
    server_ip = "yeolligo-soc.com"

    #clientSock = socket(AF_INET, SOCK_STREAM)
    #clientSock.connect((server_ip, port))

    #clientSock.send(enc)
    #recvData = clientSock.recv(1024) 

    #키 생성
    # Decoding
    #aes2 = AESCryptoCBC(key, iv)
    #dec = aes.decrypt(recvData)
    #print(dec)

    #data = json.loads(dec)
    #print(type(data), str(data))
