import os
import sound

url = "https://127.0.0.1:8080/janus?gateway_url=https://yeolligo-rtc.com/&gateway_root=/janus&room=1234&admin_key=&room_pin=&username=&token=&proxy_host=&proxy_port=80&proxy_password=&proxy_bypass=&create_room=1&publish=1&vformat=60&hw_vcodec=0&subscribe=1&reconnect=1&action=Start"
headers= {
'Accept-Encoding: gzip, deflate, br',
'Accept-Language: ko-KR,ko;q=0.8,en-US;q=0.6,en;q=0.4',
'Upgrade-Insecure-Requests: 1',
'User-Agent: Mozilla/5.0 (X11; Linux armv7l) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/60.0.3112.89 Safari/537.36',
'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8',
'Referer: ',
'Connection: keep-alive'
}

def start(roomnumber):
    #sound.Stop()
    #os.system("rm -r startresult.txt")
    url = "curl 'https://127.0.0.1:8080/janus?gateway_url=https://yeolligo-rtc.com/&gateway_root=/janus&room=5555&admin_key=&room_pin=&username=&token=&proxy_host=&proxy_port=80&proxy_password=&proxy_bypass=&create_room=1&publish=1&vformat=60&hw_vcodec=0&subscribe=1&reconnect=1&action=Start' -H 'Accept-Encoding: gzip, deflate, br' -H 'Accept-Language: ko-KR,ko;q=0.8,en-US;q=0.6,en;q=0.4' -H 'Upgrade-Insecure-Requests: 1' -H 'User-Agent: Mozilla/5.0 (X11; Linux armv7l) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/60.0.3112.89 Safari/537.36' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8' -H 'Referer: ' -H 'Connection: keep-alive' --compressed --insecure > startresult.txt"
    
    n = "%d" % roomnumber
    url = url.replace("5555", n)
    
    os.system(url)

def stop():
    os.system("rm stopresult.txt")
    os.system("curl 'https://127.0.0.1:8080/janus?action=Stop' -H 'Accept-Encoding: gzip, deflate, br' -H 'Accept-Language: ko-KR,ko;q=0.8,en-US;q=0.6,en;q=0.4' -H 'Upgrade-Insecure-Requests: 1' -H 'User-Agent: Mozilla/5.0 (X11; Linux armv7l) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/60.0.3112.89 Safari/537.36' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8' -H 'Connection: keep-alive' --compressed --insecure > stopresult.txt &")

