<div id="toc" class="toc"><div id="toctitle"><h2>Contents</h2></div>
<ul>
<li class="toclevel-1 tocsection-1"><a href="#Instruction"><span class="tocnumber">1</span> <span class="toctext">Instruction</span></a>
<ul>
<li class="toclevel-2 tocsection-2"><a href="#Features"><span class="tocnumber">1.1</span> <span class="toctext">Features</span></a></li>
</ul>
</li>
<li class="toclevel-1 tocsection-3"><a href="#Note"><span class="tocnumber">2</span> <span class="toctext">Note</span></a></li>
<li class="toclevel-1 tocsection-4"><a href="#Quick_testing"><span class="tocnumber">3</span> <span class="toctext">Quick testing</span></a></li>
<li class="toclevel-1 tocsection-5"><a href="#Using_demo_codes"><span class="tocnumber">4</span> <span class="toctext">Using demo codes</span></a>
<ul>
<li class="toclevel-2 tocsection-6"><a href="#Raspberry_Pi_examples"><span class="tocnumber">4.1</span> <span class="toctext">Raspberry Pi examples</span></a></li>
<li class="toclevel-2 tocsection-7"><a href="#Arduino_examples"><span class="tocnumber">4.2</span> <span class="toctext">Arduino examples</span></a></li>
<li class="toclevel-2 tocsection-8"><a href="#STM32_example"><span class="tocnumber">4.3</span> <span class="toctext">STM32 example</span></a></li>
</ul>
</li>
<li class="toclevel-1 tocsection-9"><a href="#Description_of_examples"><span class="tocnumber">5</span> <span class="toctext">Description of examples</span></a>
<ul>
<li class="toclevel-2 tocsection-10"><a href="#Read_Mifare_Classic_card"><span class="tocnumber">5.1</span> <span class="toctext">Read Mifare Classic card</span></a></li>
<li class="toclevel-2 tocsection-11"><a href="#Read.2FWrite_Mifare_Classic"><span class="tocnumber">5.2</span> <span class="toctext">Read/Write Mifare Classic</span></a></li>
<li class="toclevel-2 tocsection-12"><a href="#Read_NTAG2XX_card"><span class="tocnumber">5.3</span> <span class="toctext">Read NTAG2XX card</span></a></li>
<li class="toclevel-2 tocsection-13"><a href="#Read.2FWrite_NTAG2XX_Card"><span class="tocnumber">5.4</span> <span class="toctext">Read/Write NTAG2XX Card</span></a></li>
<li class="toclevel-2 tocsection-14"><a href="#Write_GPIO_of_PN532"><span class="tocnumber">5.5</span> <span class="toctext">Write GPIO of PN532</span></a></li>
<li class="toclevel-2 tocsection-15"><a href="#Read_GPIO_of_PN532"><span class="tocnumber">5.6</span> <span class="toctext">Read GPIO of PN532</span></a></li>
</ul>
</li>
<li class="toclevel-1 tocsection-16"><a href="#Resources"><span class="tocnumber">6</span> <span class="toctext">Resources</span></a>
<ul>
<li class="toclevel-2 tocsection-17"><a href="#Documents"><span class="tocnumber">6.1</span> <span class="toctext">Documents</span></a></li>
<li class="toclevel-2 tocsection-18"><a href="#Demo_codes"><span class="tocnumber">6.2</span> <span class="toctext">Demo codes</span></a></li>
</ul>
</li>
<li class="toclevel-1 tocsection-19"><a href="#FAQ"><span class="tocnumber">7</span> <span class="toctext">FAQ</span></a></li>
<li class="toclevel-1 tocsection-20"><a href="#Supports"><span class="tocnumber">8</span> <span class="toctext">Supports</span></a>
<ul>
<li class="toclevel-2"><a href="#Please_contact_us_by_Email.2FSkype.2FWeChat_for_technology_support.Our_response_may_be_delay.2C_you_can_just_leave_your_questions.2C_we_will_reply_to_you_as_soon_as_possible_in_working_time."><span class="tocnumber">8.1</span> <span class="toctext">Please contact us by Email/Skype/WeChat for technology support.Our response may be delay, you can just leave your questions, we will reply to you as soon as possible in working time.</span></a></li>
</ul>
</li>
</ul>
</div>

<h2><span class="mw-headline" id="Instruction">Instruction</span></h2>
<p>This is a Raspberry Pi NFC HAT based on PN532 operating in the 13.56MHz frequency range. It supports three communication interfaces: I2C, SPI, and UART.
</p><p>NFC (Near Field Communication) is a wireless technology allows contactless point-to-point data communication between devices within a short distance of 10 cm. It is widely used in applications such as access control system, smart tickets, meal card, etc.
</p><p>Based on the popular NFC controller PN532 with multi interface options, this HAT will easily enable NFC function for your Raspberry Pi.
</p>
<h3><span class="mw-headline" id="Features">Features</span></h3>
<ul>
<li>Standard Raspberry Pi 40PIN GPIO extension header, supports Raspberry Pi series boards
</li>
<li>Onboard PN532 chip, supports various NFC/RFID cards like MIFARE/NTAG2xx, etc.
</li>
<li>Three interface options: I2C, SPI, and UART, configured via jumpers and switches
</li>
<li>Breakout control pins, for easily connecting with host boards like STM32/Arduino
</li>
<li>Comes with development resources and manual (examples for Raspberry Python/C, STM32, Arduino)
</li>
</ul>
<h2><span class="mw-headline" id="Note">Note</span></h2>
<ul>
<li>This module can only be used to write/read NFC card whose password is known, it cannot be used for decrypting encrypted NFC card. For example, the default password of all blocks of Mifare Classic card is 0xFFFFFFFFFFFF. the Mifare card can be written/read only when the default password isn't changed
</li>
<li>This module cannot be used to copy card, unless that the card use default password.
</li>
<li>This module cannot be used to simulate NFC card. Because ID of NFC card are 4 bytes, because of security policy of PN532, it will set the first byte of simulate card to 0x08. For more details, please refer to <a rel="nofollow" target="_blank" class="external free" href="http://www.nfc-tools.org/index.php?title=PN53x%E3%80%82">http://www.nfc-tools.org/index.php?title=PN53x%E3%80%82</a> 
</li>
</ul>
<h2><span class="mw-headline" id="Quick_testing">Quick testing</span></h2>
<p>You can quick test the module by connecting it to PC with USB to TTL module instead of Raspberry Pi
</p>
<dl>
<dt>1. Hardware connection
</dt>
</dl>
<table class="wikitable">
<tr>
<th>PN532 NFC HAT</th>
<th>USB to TTL Module
</th></tr>
<tr>
<td>3V3</td>
<td>3.3V
</td></tr>
<tr>
<td>GND</td>
<td>GND
</td></tr>
<tr>
<td>TX</td>
<td>RX
</td></tr>
<tr>
<td>RX</td>
<td>TX
</td></tr></table>
<dl>
<dt>2. Set L0 to L and L1 to L by jumpers
</dt>
</dl>
<dl>
<dt>3. Connect USB to TTL Module to PC by USB cable
</dt>
</dl>
<dl>
<dt>4. Open Serial assistant software, set it 
</dt>
</dl>
<ul>
<li>波特率（Baud rate）：115200
</li>
<li>数据位（Data bits）：8
</li>
<li>停止位（Stop bits）：1
</li>
<li>校验位（Parity）：None
</li>
<li>流控制（Flow control）：None
</li>
</ul>
<dl>
<dt>5. Check "HEX发送” and “HEX显示”
</dt>
<dd><a href="/wiki/File:PN532_NFC_HAT.jpg" class="image"><img alt="PN532 NFC HAT.jpg" src="/w/upload/a/a9/PN532_NFC_HAT.jpg" width="700" height="555" /></a>
</dd>
</dl>
<dl>
<dt>6. Select correct serial port and open
</dt>
</dl>
<dl>
<dt>7. Send data below to wake up FN532 module：
</dt>
</dl>
<pre>55 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 03 FD D4 14 01 17 00
</pre>
<p>(Please refer to PN532 User Manual HSU wake up condition Chapter)
</p><p>The response from PN532 module should be：
</p>
<pre>00 00 FF 00 FF 00 00 00 FF 02 FE D5 15 16 00
</pre>
<dl>
<dt>8. Send data below to scan Mifare Classic card（The blue card provided, hereafter called as "card"）
</dt>
</dl>
<pre>00 00 FF 04 FC D4 4A 01 00 E1 00
</pre>
<p>Closing card to coil part of module, module scan it and response:
</p>
<pre>00 00 FF 0C F4 D5 4B 01 01 00 04 08 04 XXXXXXXXXX 00
</pre>
<p>XXXXXXXXXX in response data is ID (3 bytes) and checksum (1 byte) of card.(Please refer to PN532 User Manual InListPassiveTarget Chapter）
</p>
<h2><span class="mw-headline" id="Using_demo_codes">Using demo codes</span></h2>
<p>PN532 NFC HAT supports UART, I2C and SPI interface. You can use them according to your situation.After connecting PN532 NFC HAT (hereafter called as PN532) to Raspberry PI, then set the L1, L0 jumpers and DIP switch for different interfaces.
</p><p>Choose the interface：
</p>
<ul>
<li>UART：By default, UART interface of Raspberry Pi is used for Sheell debugging. If you use serial port for degbugging, you need to add an USB to TTL module for communicating just like what we do on <a href="#Quick_testing">#Quick testing</a>. In this case, the serial port is mapped as ttyUSB0 instead of ttyS0
</li>
<li>I2C： Raspberry Pi doesn't supports I2C Clock Stretching. However, PN532 may use Clock Stretching (slaver pull-down SCL pin of I2C interface). Clock Stretching cause that Raspberry Pi cannot control all I2C devices, therefore, if you need to connect other i2C device, we do not recommend you to use I2C interface.
</li>
<li>SPI：We use D4 (BCM) pin as Chip select pin of SPI interface. Take care about GPIO conflict. 
</li>
</ul>
<h3><span class="mw-headline" id="Raspberry_Pi_examples">Raspberry Pi examples</span></h3>
<p>Download demo code from <a href="#Resources">#Resources</a>, unzip and copy raspberrypi folder to /home/pi of Raspberry Pi. You can firstly copy it to /boot of SD card, then copy it to /home/pi. 
</p>
<dl>
<dt>SPI Interface
</dt>
</dl>
<p>1. Set L0 toL and L1 to H by jumpers
</p><p>2. Connect RSTPDN-&gt;D20 by jumper
</p><p>3. Set DIP switch to 
</p>
<table class="wikitable" width="500px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-2.jpg" class="image"><img alt="PN532 NFC HAT-2.jpg" src="/w/thumb.php?f=PN532_NFC_HAT-2.jpg&amp;width=700" width="700" height="525" srcset="/w/thumb.php?f=PN532_NFC_HAT-2.jpg&amp;width=1050 1.5x, /w/upload/e/ea/PN532_NFC_HAT-2.jpg 2x" /></a>
</dd>
</dl>
<p>4. Connect PN532 NFC HAT to Raspberry Pi
</p>
<table class="wikitable" width="600px">
<caption>Connect PN532 NFC HAT to Raspberry Pi via SPI interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>Raspberry Pi (BCM)
</th></tr>
<tr align="center">
<td>SCK</td>
<td>SCK
</td></tr>
<tr align="center">
<td>MISO</td>
<td>MISO
</td></tr>
<tr align="center">
<td>MOSI</td>
<td>MOSI
</td></tr>
<tr align="center">
<td>NSS</td>
<td>P4 (D4)
</td></tr></table>
<p>5. Enable SPI interface
</p><p>Open Terminal of Raspberry Pi，use command: sudo raspi-config 
</p><p>Choose Interfacing Options -&gt; SPI -&gt; Yes
</p><p>6. Run demo codes（Use example_get_uid.py and rpi_get_uid.c as example）
</p><p>Open Terminal, navigate to directory of demo codes
</p>
<pre>cd ~/raspberrypi/
</pre>
<p>1) python code：
</p><p>Enter directory of python codes： cd ~/raspberrypi/python/
</p><p>Modify example_get_uid.py file，set the initialize code as ：
</p>
<pre>pn532 = PN532_SPI(debug=False, reset=20, cs=4)
#pn532 = PN532_I2C(debug=False, reset=20, req=16)
#pn532 = PN532_UART(debug=False, reset=20)
</pre>
<p>Save after modifying, then run the codes with command:
</p>
<pre>python3 example_get_uid.py
</pre>
<p>2) C codes：
</p><p>Enter directory of c code: cd ~/raspberrypi/c/example/
</p><p>Modify rpi_get_uid.c file, set initialize code as：
</p>
<pre>PN532_SPI_Init(&amp;pn532);
//PN532_I2C_Init(&amp;pn532);
//PN532_UART_Init(&amp;pn532);
</pre>
<p>Save and compile：sudo make
</p><p>Run the code：
</p>
<pre>./rpi_get_uid.exe
</pre>
<p>7. Expected result：Close card to coil part of PN532, the UID of card is read
</p>
<dl>
<dt>UART interface
</dt>
</dl>
<p>1. Set L0 to L and L1 to L by jumpers
</p><p>2. Connecting RSTPDN -&gt;D20 by jumper
</p><p>3. Set DIP switch to
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>ON</td>
<td>ON
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-3.jpg" class="image"><img alt="PN532 NFC HAT-3.jpg" src="/w/thumb.php?f=PN532_NFC_HAT-3.jpg&amp;width=700" width="700" height="525" srcset="/w/thumb.php?f=PN532_NFC_HAT-3.jpg&amp;width=1050 1.5x, /w/upload/6/6b/PN532_NFC_HAT-3.jpg 2x" /></a>
</dd>
</dl>
<p>4. Connect PN532 to Raspberry Pi
</p>
<table class="wikitable" width="600px">
<caption>Connect PN532 NFC HAT to Raspberry Pi via UART interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>Raspberry Pi
</th></tr>
<tr align="center">
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>TX</td>
<td>RX
</td></tr></table>
<p>5. Enable Serial port. By default, the serial port is used for Shell debugging.
</p><p>Open Terminal of Raspberry PI and run command: sudo raspi-config 
</p><p>Choose Interfacing Options-&gt; Serial -&gt; No -&gt; Yes
</p><p>【Note】You need to restart Raspberry Pi after enabling serial port
</p><p>6. Run demo codes（Use example_get_uid.py and rpi_get_uid.c as examples）
</p><p>Open Terminal and navigate to directory of demo codes：
</p>
<pre>cd ~/raspberrypi/
</pre>
<p>1) python code：
</p><p>Enter directory of python code： cd ~/raspberrypi/python/
</p><p>Modify example_get_uid.py file, set initialize code to：
</p>
<pre>#pn532 = PN532_SPI(debug=False, reset=20, cs=4)
#pn532 = PN532_I2C(debug=False, reset=20, req=16)
pn532 = PN532_UART(debug=False, reset=20)
</pre>
<p>Save.Then run code by command:
</p>
<pre>python3 example_get_uid.py
</pre>
<p>2) C code：
</p><p>Enter directory of c code：cd ~/raspberrypi/c/example/
</p><p>Modify rpi_get_uid.c file, set initialize code to：
</p>
<pre>//PN532_SPI_Init(&amp;pn532);
//PN532_I2C_Init(&amp;pn532);
PN532_UART_Init(&amp;pn532);
</pre>
<p>Save then compile code: sudo make
</p><p>Run code：
</p>
<pre>./rpi_get_uid.exe
</pre>
<p>7. Expected result：Close card to coil part of PN532, the UID of card is read
</p>
<dl>
<dt>If demo code of UART run failed, you can test serial port by this example</dt>
<dd>
</dd>
</dl>
<pre>cd ~/raspberrypi/python/
python3 example_uart_hex.py
</pre>
<p>Enter data and sent, data sent and received should be printed on terminal as expected. e.g. sent data below to wake up PN532：
</p>
<pre>55 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 03 FD D4 14 01 17 00
</pre>
<p>The response data should be：
</p>
<pre>00 00 FF 00 FF 00 00 00 FF 02 FE D5 15 16 00
</pre>
<dl>
<dt>I2C interface
</dt>
</dl>
<p>1. Set L0 as H, L1 as L by jumpers
</p><p>2. Connect RSTPDN -&gt;D20 and INT0 -&gt; D16(avoid from Clock Stretching) by jumpers
</p><p>3. Set DIP switch to
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-9.png" class="image"><img alt="PN532 NFC HAT-9.png" src="/w/thumb.php?f=PN532_NFC_HAT-9.png&amp;width=700" width="700" height="471" srcset="/w/upload/6/62/PN532_NFC_HAT-9.png 1.5x, /w/upload/6/62/PN532_NFC_HAT-9.png 2x" /></a>
</dd>
</dl>
<p>4. Connect PN532 to Raspberry Pi
</p>
<table class="wikitable" width="600px">
<caption>Connect PN532 NFC HAT to Raspberry Pi via I2C interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>Raspberry Pi
</th></tr>
<tr align="center">
<td>SCL</td>
<td>SCL
</td></tr>
<tr align="center">
<td>SDA</td>
<td>SDA
</td></tr></table>
<p>5. Enable I2C interface
</p><p>Open Terminal of Raspberry Pi and run command: sudo raspi-config
</p><p>Choose Interfacing Options-&gt; I2C -&gt; Yes
</p><p><br />
6. Run code（Use example_get_uid.py and rpi_get_uid.c as examples）
</p><p>Open ternimal and navigate to directory of demo codes:
</p>
<pre>cd ~/raspberrypi/
</pre>
<p>1) python code：
</p><p>Enter directory of python code： cd ~/raspberrypi/python/
</p><p>Modify example_get_uid.py file, set initialize code to:
</p>
<pre>#pn532 = PN532_SPI(debug=False, reset=20, cs=4)
pn532 = PN532_I2C(debug=False, reset=20, req=16)
#pn532 = PN532_UART(debug=False, reset=20)
</pre>
<p>Save, then run code
</p>
<pre>python3 example_get_uid.py
</pre>
<p>2) C code：
</p><p>Enter directory of c code：cd ~/raspberrypi/c/example/
</p><p>Modify rpi_get_uid.c file, set initialize code to：
</p>
<pre>//PN532_SPI_Init(&amp;pn532);
PN532_I2C_Init(&amp;pn532);
//PN532_UART_Init(&amp;pn532);
</pre>
<p>Save then compile codes：sudo make
</p><p>Run code：
</p>
<pre>./rpi_get_uid.exe
</pre>
<p>7. Expected result：Close card to coil part of PN532, the UID of card is read
</p>
<h3><span class="mw-headline" id="Arduino_examples">Arduino examples</span></h3>
<p>1. Make sure that you have installed Arduino IDE in your PC
</p><p>2. Create a new folder in ...\Arduino\libraries (Installation directory of Arduino IDE) and named it as pn532
</p><p>3. Copy files pn532.c, pn532.h, pn532_uno.cpp and pn532_uno.h to ...\Arduino\libraries\pn532 from Arduino demo codes
</p><p>4. Demo codes is under examples\arduino (demo codes you download) directory
</p><p>5. Herein we take Arduino UNO board as example
</p>
<dl>
<dt>SPI interface
</dt>
</dl>
<p>1. Set L0 to L and L1 to H by hy jumpers
</p><p>2. Set DIP switch to：
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<p>3. Connect PN532 NFC HAT to Arduino UNO:
</p>
<table class="wikitable" width="700px">
<caption>Connect PN532 NFC HAT to Arduino UNO via SPI interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>Arduino UNO
</th></tr>
<tr align="center">
<td>SCK</td>
<td>D13
</td></tr>
<tr align="center">
<td>MISO</td>
<td>D12
</td></tr>
<tr align="center">
<td>MOSI</td>
<td>D11
</td></tr>
<tr align="center">
<td>NSS</td>
<td>D4
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-5.png" class="image"><img alt="PN532 NFC HAT-5.png" src="/w/thumb.php?f=PN532_NFC_HAT-5.png&amp;width=800" width="800" height="500" srcset="/w/thumb.php?f=PN532_NFC_HAT-5.png&amp;width=1200 1.5x, /w/upload/1/15/PN532_NFC_HAT-5.png 2x" /></a>
</dd>
</dl>
<p>4. Run codes（Use examples\arduino\uno_get_uid\ uno_get_uid.ino as examples）：
</p><p>Open uno_get_uid.ino file, set initialize code to：
</p>
<pre>PN532_SPI_Init(&amp;pn532);
//PN532_I2C_Init(&amp;pn532);
</pre>
<p>Compile and upload codes to Arduino UNO
</p><p>Open Serial monitor, press Reset button of Arduino Uno to reset
</p><p>5. Expected result: close card to coil part of PN532, the UID of card is read and printed.
</p>
<dl>
<dt>I2C interface
</dt>
</dl>
<p>1. Set L0 to L and L1 to H by jumpers
</p><p>2. Set DIP switch to
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<p>3. Connect PN532 NFC HAT to Arduino UNO 
</p>
<table class="wikitable" width="700px">
<caption> Connect PN532 NFC HAT to Arduino UNO via I2C interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>Arduino UNO
</th></tr>
<tr align="center">
<td>SCL</td>
<td>A5
</td></tr>
<tr align="center">
<td>SDA</td>
<td>A4
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-4.png" class="image"><img alt="PN532 NFC HAT-4.png" src="/w/thumb.php?f=PN532_NFC_HAT-4.png&amp;width=800" width="800" height="500" srcset="/w/thumb.php?f=PN532_NFC_HAT-4.png&amp;width=1200 1.5x, /w/upload/7/76/PN532_NFC_HAT-4.png 2x" /></a>
</dd>
</dl>
<p>4. Run code（Use examples\arduino\uno_get_uid\ uno_get_uid.ino as example）：
</p><p>Open uno_get_uid.ino file, set initialize code to：
</p>
<pre>//PN532_SPI_Init(&amp;pn532);
PN532_I2C_Init(&amp;pn532);
</pre>
<p>Compile and upload codes to Arduino UNO board
</p><p>Open Serial monitor, press Reset button of Arduino UNO board to reset.
</p><p>5. Expected result: close card to coil part of PN532, the UID of card is read and printed.
</p>
<h3><span class="mw-headline" id="STM32_example">STM32 example</span></h3>
<p>The development board used here is Open103C which is based on STM32F103CBT6.
</p>
<dl>
<dt>Download project
</dt>
</dl>
<p>1. Open project by keil software（...\MDK-ARM\pn532_stm32.uvprojx），Click Rebuild to compile project.
</p><p>2. Choose programmer：Options for Target -&gt; Debug-&gt; Use，default: ST-Link Debugger。
</p><p>3. Choose download method：Options for Target -&gt; Debug选项卡 -&gt; Settings -&gt; Debug -&gt; Port，Default: JTAG.
</p><p>4. Connect Open board to PC by programmer. Note that you should power Open board separately.
</p><p>5. Click  Download  to download project.
</p>
<dl>
<dt>Hardware connection
</dt>
</dl>
<p><b>SPI connecting</b>
</p><p>1. Set L0 to L and L1 to H by jumpers
</p><p>2. Set DIP switch to 
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<p>3. Connect PN632 to Open103C board 
</p>
<table class="wikitable" width="700px">
<caption>Connect PN532 NFC HAT to STM32F via SPI interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>STM32F103CBT6
</th></tr>
<tr align="center">
<td>SCK</td>
<td>PA5
</td></tr>
<tr align="center">
<td>MISO</td>
<td>PA6
</td></tr>
<tr align="center">
<td>MOSI</td>
<td>PA7
</td></tr>
<tr align="center">
<td>NSS</td>
<td>PA4
</td></tr></table>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-5.png" class="image"><img alt="PN532 NFC HAT-5.png" src="/w/thumb.php?f=PN532_NFC_HAT-5.png&amp;width=800" width="800" height="500" srcset="/w/thumb.php?f=PN532_NFC_HAT-5.png&amp;width=1200 1.5x, /w/upload/1/15/PN532_NFC_HAT-5.png 2x" /></a>
</dd>
</dl>
<p><b>I2C connecting</b>
</p><p>1. Set L0 to H and L1 to L by jumpers
</p><p>2. Set DIP switch to 
</p>
<table class="wikitable" width="600px">

<tr align="center">
<td>SCK</td>
<td>MISO</td>
<td>MOSI</td>
<td>NSS</td>
<td>SCL</td>
<td>SDA</td>
<td>RX</td>
<td>TX
</td></tr>
<tr align="center">
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>OFF</td>
<td>ON</td>
<td>ON</td>
<td>OFF</td>
<td>OFF
</td></tr></table>
<p>3. Connect PN632 to Open103C board 
</p>
<table class="wikitable" width="700px">
<caption>Connect PN532 NFC HAT to STM32F via I2C interface
</caption>
<tr>
<th>PN532 NFC HAT</th>
<th>STM32F103CBT6
</th></tr>
<tr align="center">
<td>SCL</td>
<td>PB6
</td></tr>
<tr align="center">
<td>SDA</td>
<td>PB7
</td></tr></table>
<dl>
<dt>Software setting
</dt>
</dl>
<p>Open project and choose the interface according to interface used. recompile and download
</p>
<div dir="ltr" class="mw-geshi mw-code mw-content-ltr"><div class="c source-c"><pre class="de1"> <span class="co1">// PN532_SPI_Init(&amp;pn532);</span>
PN532_I2C_Init<span class="br0">&#40;</span><span class="sy0">&amp;</span>pn532<span class="br0">&#41;</span><span class="sy0">;</span></pre></div></div>
<p>4. Connect USB to TTL module to UART1 interface (PA9-&gt;RX, PA10-&gt;TX） of STM32 and PC.
</p><p>5. Open serial assistant software, and reset Open103C
</p><p>6. Expected result: close card to coil part of PN532, the UID of card is read and printed.
</p>
<h2><span class="mw-headline" id="Description_of_examples">Description of examples</span></h2>
<p>Examples above are used to read UID of Mifare Classic card (example_get_uid.py / rpi_get_uid.exe / uno_get_uid.ino / stm32_get_uid), here we describe other examples.
</p><p>【Note】
</p><p>Before run the examples, you should set the L0/L1 pins and DIp switch according to interfaces used. You cannot set all the pins to ON,it will cause that data received are wrong. Here we take 0 is OFF and 1 is ON
</p>
<ul>
<li>UART interface:Set jumpers [L1..L0] as LL，set DIP switch to 00000011
</li>
<li>I2C interface: Set jumpers [L1..L0] as LH, set DIP switch to 00001100.
</li>
<li>SPI interface: Set jumpers [L1..L0] as HL, set DIP switch to 11110000.
</li>
</ul>
<h3><span class="mw-headline" id="Read_Mifare_Classic_card">Read Mifare Classic card</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware platform</b>
</td></tr>
<tr align="center">
<td>example_dump_mifare.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_dump_mifare.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_dump_mifare.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_dump_mifare/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p><br />
Expected result:Close the Mifare Classic card to PN532 NFC HAT, the data in card are printed:
</p>
<div dir="ltr" class="mw-geshi mw-code mw-content-ltr"><div class="c source-c"><pre class="de1"><span class="nu0">0</span> <span class="sy0">:</span> <span class="nu0">37</span> F9 <span class="nu0">20</span> <span class="nu0">69</span> <span class="nu0">87</span> <span class="nu19">08</span> <span class="nu8">04</span> <span class="nu8">00</span> <span class="nu0">62</span> <span class="nu0">63</span> <span class="nu0">64</span> <span class="nu0">65</span> <span class="nu0">66</span> <span class="nu0">67</span> <span class="nu0">68</span> <span class="nu0">69</span>
<span class="nu0">1</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">2</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">3</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> FF <span class="nu8">07</span> <span class="nu0">80</span> <span class="nu0">69</span> FF FF FF FF FF FF
<span class="nu0">4</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">5</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">6</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">7</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> FF <span class="nu8">07</span> <span class="nu0">80</span> <span class="nu0">69</span> FF FF FF FF FF FF
<span class="nu0">8</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">9</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">10</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">11</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> FF <span class="nu8">07</span> <span class="nu0">80</span> <span class="nu0">69</span> FF FF FF FF FF FF
… …
<span class="nu0">63</span> <span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> FF <span class="nu8">07</span> <span class="nu0">80</span> <span class="nu0">69</span> FF FF FF FF FF FF</pre></div></div>
<p>Note：
</p>
<ul>
<li>Data of every row are related to every block of card
</li>
<li>First 4 bytes of block 0 are UID of Mifare Classic card, fifth byte is parity bit. It is checksum of the first 4 bytes.
</li>
<li>Block 0 (UID) of classic card cannot be modified.
</li>
<li>Every sector have four blocks, these four blocks use same password. For example, blocks 4N~4N+3 belong to same sector.
</li>
<li>Block 4N+3 is cipher block, that is if you want to read the data of blocks 4N~4N+2, you must use cipher of block 4N+3.
</li>
<li>The car has 64 blocks in total, which is 1K. 
</li>
</ul>
<p>e.g. To read block 6, we need to use cipher of the related sector, that is the data of block 7. 
</p>
<pre>7&#160;: 00 00 00 00 00 00 FF 07 80 69 FF FF FF FF FF FF
</pre>
<p>The first six bytes are KEY A, it is default FF FF FF FF FF FF, if you read it, the data response would be 00 00 00 00 00 00. The last six bytes are KEY B saved in plaintext. KEY A and KEY B of Mifare Classic card are default FF FF FF FF FF FF.
</p><p>Four bytes in the middle are Access Bits, default FF 07 80 69. Access Bits are used for controlling access permission, <b>it will be locked if user write wrong data to them</b>.
</p>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-6.png" class="image"><img alt="PN532 NFC HAT-6.png" src="/w/upload/7/7c/PN532_NFC_HAT-6.png" width="700" height="351" /></a>
</dd>
</dl>
<p>For example, if you write 0xFF to byte 6, high four bits of byte 7 should be 0b0000,low four bits of byte 8 should be 0b0000. For more details, please refer to Access conditions for data blocks section of MF1S50YYX_V1.pdf
</p><p><b>The card provided can be unlock by "back door" leaved, however, if you use common Mifare card, once you write wrong data to Access Bits, the card will be locked and unable to restore. Be careful when you writing card</b>
</p>
<h3><span class="mw-headline" id="Read.2FWrite_Mifare_Classic">Read/Write Mifare Classic</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware Platform</b>
</td></tr>
<tr align="center">
<td>example_rw_mifare.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_rw_mifare.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_rw_mifare.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_rw_mifare/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p><br />
Expected result: Close Mifare Classic card to PN532 NFC HAT, block 6 will be written with data 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F.
</p><p>Note:
</p>
<ul>
<li>Before you write the block 6, you should make sure that the cipher of block 6 is 0xFF FF FF FF FF FF, which are the first six bytes of block 7.
</li>
<li>If you want to modify examples and try to write block N+3, you much be careful and remember the cipher. Because that N+3 is cipher block, once you forget the cipher, you should not access block N~N+2 any more.
</li>
</ul>
<h3><span class="mw-headline" id="Read_NTAG2XX_card">Read NTAG2XX card</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware plaftform</b>
</td></tr>
<tr align="center">
<td>example_dump_ntag2.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_dump_ntag2.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_dump_ntag2.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_dump_ntag2/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p>Expected result: Close Ntag215 card to PN532 NFC HAT, data of the card will be printed
</p>
<div dir="ltr" class="mw-geshi mw-code mw-content-ltr"><div class="c source-c"><pre class="de1"><span class="nu0">0</span><span class="sy0">:</span> <span class="nu8">04</span> <span class="nu0">85</span> <span class="nu0">32</span> 3b
<span class="nu0">1</span><span class="sy0">:</span> <span class="nu0">92</span> a8 <span class="nu0">64</span> <span class="nu0">80</span>
<span class="nu0">2</span><span class="sy0">:</span> de <span class="nu0">48</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">3</span><span class="sy0">:</span> e1 <span class="nu0">10</span> 3e <span class="nu8">00</span>
<span class="nu0">4</span><span class="sy0">:</span> <span class="nu8">03</span> <span class="nu8">00</span> fe <span class="nu8">00</span>
<span class="nu0">5</span><span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">6</span><span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
<span class="nu0">7</span><span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span>
… …
<span class="nu0">134</span><span class="sy0">:</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span> <span class="nu8">00</span></pre></div></div>
<p>Note：
</p>
<ul>
<li>Ntag215 card should be purchase separately.
</li>
<li>Page of Ntag215 card just like blocks of Mifare card.
</li>
<li>The first three bytes of page 0 is UID0~UID2 of Ntag215 card, the fourth byte is parity bits, it is checksum of the first three bytes and 0x88 (Cascade Tag).
</li>
<li>Page 1 are UID3 ~ UID6 of card.
</li>
<li>The first byte of Page 2 is parity bit, it is checksum (Xor) of UID3 ~ UOD6.
</li>
</ul>
<h3><span class="mw-headline" id="Read.2FWrite_NTAG2XX_Card">Read/Write NTAG2XX Card</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware platform</b>
</td></tr>
<tr align="center">
<td>example_rw_ntag2.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_rw_ntag2.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_rw_ntag2.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_rw_ntag2/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p>Expected result: Close Ntag215 card to PN532 NFC HAT, data 00 01 02 03 are written to Page 6.
</p><p>Note：
</p>
<ul>
<li>The last two bytes of Page 2 are used for flag and lock page from 03h to 0Fh, set them to read-only mode. <b>This operation is not reversible.</b>  Please take care about it when you try to write the page. For more details, please refer to Static lock bytes (NTAG21x) Section of NTAG213/215/216 file.
</li>
</ul>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-7.png" class="image"><img alt="PN532 NFC HAT-7.png" src="/w/upload/c/c8/PN532_NFC_HAT-7.png" width="800" height="308" /></a>
</dd>
</dl>
<ul>
<li>If you want to lock data of pages after Page 10h, you should write corresponding addresses to first three bytes of Page 28h （Ntag213）, Page 82h （Ntag215） or Page E2h（Ntag216）.For details, please refer to Dynamic Lock Bytes Section of NTAG213/215/216 file.
</li>
</ul>
<dl>
<dd><a href="/wiki/File:PN532_NFC_HAT-8.png" class="image"><img alt="PN532 NFC HAT-8.png" src="/w/upload/f/fb/PN532_NFC_HAT-8.png" width="800" height="628" /></a>
</dd>
</dl>
<ul>
<li>About more functions of Ntag2xx car please refer to  NTAG213/215/216 manual.
</li>
</ul>
<h3><span class="mw-headline" id="Write_GPIO_of_PN532">Write GPIO of PN532</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware platform</b>
</td></tr>
<tr align="center">
<td>example_write_gpio.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_write_gpio.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_write_gpio.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_write_gpio/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p>Expected result: Print status of PN532's GPIO
</p>
<div dir="ltr" class="mw-geshi mw-code mw-content-ltr"><div class="c source-c"><pre class="de1">Pin P30<span class="sy0">:</span> <span class="nu0">1</span>
Pin P31<span class="sy0">:</span> <span class="nu0">0</span>
Pin P32<span class="sy0">:</span> <span class="nu0">1</span>
Pin P33<span class="sy0">:</span> <span class="nu0">0</span>
Pin P34<span class="sy0">:</span> <span class="nu0">1</span>
Pin P35<span class="sy0">:</span> <span class="nu0">0</span>
Pin P71<span class="sy0">:</span> <span class="nu0">0</span>
Pin P72<span class="sy0">:</span> <span class="nu0">1</span>
Pin I0<span class="sy0">:</span> <span class="nu0">1</span>
Pin I1<span class="sy0">:</span> <span class="nu0">0</span></pre></div></div>
<p>Note：
</p><p>Examples will set GPIO：P30 -&gt; HIHG，P31 -&gt; LOW，P33 -&gt; LOW，P35 -&gt; LOW，P71 -&gt; LOW，P72 -&gt; HIGH
</p><p>Note：
</p>
<ul>
<li>P32 is int0, once it is set to LOW, PN532 is reset.
</li>
<li>P34 is SIC_CLK, it is always High when you read it
</li>
<li>P71 is MISO, it is High when you set PN532 to SPI mode.
</li>
<li>P72 is SCK，it is High when you set PN532 to SPI Mode.
</li>
<li>When powering, PN532 will set communication mode according to L0/L1. After setting, you can remove jumpers and use them as GPIO.
</li>
</ul>
<p>The states of PIN P30 -- P35 will be set to Hihg after hardware reset (RSTPDN is set to Low for 2s and then set to High)
</p>
<h3><span class="mw-headline" id="Read_GPIO_of_PN532">Read GPIO of PN532</span></h3>
<table border="1px" width="800px">

<tr align="center" style="background:grey; color:white">
<td><b>Examples</b></td>
<td><b>Hardware platform</b>
</td></tr>
<tr align="center">
<td>example_read_gpio.py</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>rpi_read_gpio.c</td>
<td>Raspberry Pi
</td></tr>
<tr align="center">
<td>uno_read_gpio.ino</td>
<td>Arduino UNO
</td></tr>
<tr align="center">
<td>stm32_read_gpio/MDK-ARM/pn532_stm32.uvprojx</td>
<td>STM32F103CBT6
</td></tr></table>
<p>Expected result: Print states of PN532's GPIO
</p>
<div dir="ltr" class="mw-geshi mw-code mw-content-ltr"><div class="c source-c"><pre class="de1">Port P3<span class="sy0">:</span> <span class="nu12">0x3f</span>
Port P7<span class="sy0">:</span> <span class="nu12">0x07</span>
Port I<span class="sy0">:</span> <span class="nu12">0x07</span>
Pin P30<span class="sy0">:</span> <span class="nu0">1</span>
Pin P31<span class="sy0">:</span> <span class="nu0">1</span>
Pin P32<span class="sy0">:</span> <span class="nu0">1</span>
Pin P33<span class="sy0">:</span> <span class="nu0">1</span>
Pin P34<span class="sy0">:</span> <span class="nu0">1</span>
Pin P35<span class="sy0">:</span> <span class="nu0">1</span>
Pin I0<span class="sy0">:</span> <span class="nu0">1</span>
Pin I1<span class="sy0">:</span> <span class="nu0">0</span></pre></div></div>
<p>Note:
</p><p>The states of PIN P30 -- P35 will be set to Hihg after hardware reset (RSTPDN is set to Low for 2s and then set to High)
</p>
<h2><span class="mw-headline" id="Resources">Resources</span></h2>
<h3><span class="mw-headline" id="Documents">Documents</span></h3>
<ul>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/f/f1/Pn532ds.pdf">pn532ds.pdf</a>
</li>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/b/bb/Pn532um.pdf">pn532um.pdf</a>
</li>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/0/0f/NTAG213_215_216.pdf">NTAG213_215_216.pdf</a>
</li>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/c/c7/MF1S50YYX_V1.pdf">MF1S50YYX_V1.pdf</a>
</li>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/8/83/Nfc-tools_reference_manual.pdf">nfc-tools Reference Manual</a>
</li>
</ul>
<h3><span class="mw-headline" id="Demo_codes">Demo codes</span></h3>
<ul>
<li><a rel="nofollow" target="_blank" class="external text" href="https://www.waveshare.com/w/upload/6/67/Pn532-nfc-hat-code.7z">Demo codes</a>
</li>
</ul>
<h2><span class="mw-headline" id="FAQ">FAQ</span></h2>
<h2><span class="mw-headline" id="Supports">Supports</span></h2>
<h3 style="text-align:center; width: 600px"><span class="mw-headline" id="Please_contact_us_by_Email.2FSkype.2FWeChat_for_technology_support.Our_response_may_be_delay.2C_you_can_just_leave_your_questions.2C_we_will_reply_to_you_as_soon_as_possible_in_working_time.">Please contact us by Email/Skype/WeChat for technology support.Our response may be delay, you can just leave your questions, we will reply to you as soon as possible in working time.</span></h3>
<table class="bd-yellow at-c" style="table-layout:fixed;border-radius:50px;width:600px;border:2px solid #&#123;&#123;&#123;bordercolor}}}">

<tr>
<td class="bg-yellow roundy-65" style="table-layout:fixed;border-radius:100px;padding:10px;width:100px;background:#&#123;&#123;&#123;background}}};"><a href="/wiki/File:Service_email.png" class="image"><img alt="Service email.png" src="/w/upload/5/53/Service_email.png" width="100" height="69" /></a>
</td>
<td class="bg-green" style="border-radius:100px;background:#&#123;&#123;&#123;background}}};font-weight:bold; font-size:20px;"> <a href="/cdn-cgi/l/email-protection" class="__cf_email__" data-cfemail="6b180e191d02080e2b1c0a1d0e18030a190e45080406">[email&#160;protected]</a>
</td></tr>
<tr>
<td class="bg-yellow roundy-65" style="table-layout:fixed;border-radius:100px;padding:10px;background:#&#123;&#123;&#123;background}}};"><a href="/wiki/File:Service_skype.png" class="image"><img alt="Service skype.png" src="/w/upload/c/c5/Service_skype.png" width="100" height="69" /></a>
</td>
<td class="bg-green" style="border-radius:100px;background:#&#123;&#123;&#123;background}}};font-weight:bold; font-size:20px;"> <a href="/cdn-cgi/l/email-protection" class="__cf_email__" data-cfemail="dfacbaada9b6bcba9fa8bea9baacb7beadba">[email&#160;protected]</a>
</td></tr>
<tr>
<td class="bg-yellow roundy-65" style="table-layout:fixed;border-radius:100px;padding:10px;width:100px;background:#&#123;&#123;&#123;background}}};"><a href="/wiki/File:Service_wechat.png" class="image"><img alt="Service wechat.png" src="/w/upload/3/30/Service_wechat.png" width="100" height="69" /></a>
</td>
<td class="bg-green" style="border-radius:100px;background:#&#123;&#123;&#123;background}}};;font-weight:bold; font-size:20px;"> <a href="/wiki/File:Webchat_Linzeru.png" class="image"><img alt="Webchat Linzeru.png" src="/w/thumb.php?f=Webchat_Linzeru.png&amp;width=100" width="100" height="100" srcset="/w/thumb.php?f=Webchat_Linzeru.png&amp;width=150 1.5x, /w/thumb.php?f=Webchat_Linzeru.png&amp;width=200 2x" /></a>
</td></tr>
<tr>
<td class="bg-yellow roundy-65" style="table-layout:fixed;border-radius:100px;padding:10px;width:100px;background:#&#123;&#123;&#123;background}}};"><a href="/wiki/File:Service_time.png" class="image"><img alt="Service time.png" src="/w/upload/b/b4/Service_time.png" width="100" height="69" /></a>
</td>
<td class="bg-green" style="border-radius:100px;background:#&#123;&#123;&#123;background}}};; font-weight:bold; font-size:20px;"> 09:00 - 18:00 (UTC+8 Monday to Staturday)
</td></tr></table>
<p></font>
</p>
<!-- 
NewPP limit report
CPU time usage: 0.251 seconds
Real time usage: 0.275 seconds
Preprocessor visited node count: 573/1000000
Preprocessor generated node count: 4023/1000000
Post‐expand include size: 9130/2097152 bytes
Template argument size: 692/2097152 bytes
Highest expansion depth: 4/40
Expensive parser function count: 0/100
-->

<!-- Saved in parser cache with key waveshareDB:pcache:idhash:5733-0!*!0!!en!2!* and timestamp 20210718135422 and revision id 20726
 -->
</div>								<div class="printfooter">
				Retrieved from "<a href="https://www.waveshare.com/w/index.php?title=PN532_NFC_HAT&amp;oldid=20726">https://www.waveshare.com/w/index.php?title=PN532_NFC_HAT&amp;oldid=20726</a>"				</div>
												<div id='catlinks' class='catlinks catlinks-allhidden'><div id="mw-hidden-catlinks" class="mw-hidden-catlinks mw-hidden-cats-hidden">Hidden categories: <ul><li><a href="/wiki/Category:I2C_interface" title="Category:I2C interface">I2C interface</a></li><li><a href="/wiki/Category:SPI_interface" title="Category:SPI interface">SPI interface</a></li><li><a href="/wiki/Category:UART_interface" title="Category:UART interface">UART interface</a></li></ul></div></div>												<div class="visualClear"></div>
							</div>
		</div>
		<div id="mw-navigation">
			<h2>Navigation menu</h2>
			<div id="mw-head">
				<div id="p-personal" role="navigation" class="" aria-labelledby="p-personal-label">
	<h3 id="p-personal-label">Personal tools</h3>
	<ul>
<li id="pt-login"><a href="/w/index.php?title=Special:UserLogin&amp;returnto=PN532+NFC+HAT" title="You are encouraged to log in; however, it is not mandatory [o]" accesskey="o">Log in</a></li>	</ul>
</div>
				<div id="left-navigation">
					<div id="p-namespaces" role="navigation" class="vectorTabs" aria-labelledby="p-namespaces-label">
	<h3 id="p-namespaces-label">Namespaces</h3>
	<ul>
					<li  id="ca-nstab-main" class="selected"><span><a href="/wiki/PN532_NFC_HAT"  title="View the content page [c]" accesskey="c">Page</a></span></li>
					<li  id="ca-talk" class="new"><span><a href="/w/index.php?title=Talk:PN532_NFC_HAT&amp;action=edit&amp;redlink=1"  title="Discussion about the content page [t]" accesskey="t">Discussion</a></span></li>
			</ul>
</div>
<div id="p-variants" role="navigation" class="vectorMenu emptyPortlet" aria-labelledby="p-variants-label">
	<h3 id="mw-vector-current-variant">
		</h3>
	<h3 id="p-variants-label"><span>Variants</span><a href="#"></a></h3>
	<div class="menu">
		<ul>
					</ul>
	</div>
</div>
				</div>
				<div id="right-navigation">
					<div id="p-views" role="navigation" class="vectorTabs" aria-labelledby="p-views-label">
	<h3 id="p-views-label">Views</h3>
	<ul>
					<li id="ca-view" class="selected"><span><a href="/wiki/PN532_NFC_HAT" >Read</a></span></li>
					<li id="ca-viewsource"><span><a href="/w/index.php?title=PN532_NFC_HAT&amp;action=edit"  title="This page is protected.&#10;You can view its source [e]" accesskey="e">View source</a></span></li>
					<li id="ca-history" class="collapsible"><span><a href="/w/index.php?title=PN532_NFC_HAT&amp;action=history"  title="Past revisions of this page [h]" accesskey="h">View history</a></span></li>
			</ul>
</div>
<div id="p-cactions" role="navigation" class="vectorMenu emptyPortlet" aria-labelledby="p-cactions-label">
	<h3 id="p-cactions-label"><span>Actions</span><a href="#"></a></h3>
	<div class="menu">
		<ul>
					</ul>
	</div>
</div>
<div id="p-search" role="search">
	<h3><label for="searchInput">Search</label></h3>
	<form action="/w/index.php" id="searchform">
					<div id="simpleSearch">
					<input type="search" name="search" placeholder="Search" title="Search Waveshare Wiki [f]" accesskey="f" id="searchInput" /><input type="hidden" value="Special:Search" name="title" /><input type="submit" name="fulltext" value="Search" title="Search the pages for this text" id="mw-searchButton" class="searchButton mw-fallbackSearchButton" /><input type="submit" name="go" value="Go" title="Go to a page with this exact name if exists" id="searchButton" class="searchButton" />		</div>
	</form>
</div>
				</div>
			</div>
			<div id="mw-panel">
					<div id="p-logo" role="banner"><a style="background-image: url(/w/upload/logo/waveshare-logo.png);" href="/wiki/Main_Page"  title="Visit the main page"></a></div>
				<div class="portal" role="navigation" id='p-navigation' aria-labelledby='p-navigation-label'>
	<h3 id='p-navigation-label'>Navigation</h3>
	<div class="body">
		<ul>
			<li id="n-mainpage-description"><a href="/wiki/Main_Page" title="Visit the main page [z]" accesskey="z">Main page</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-mini_PC' aria-labelledby='p-mini_PC-label'>
	<h3 id='p-mini_PC-label'>mini PC</h3>
	<div class="body">
		<ul>
			<li id="n-Raspberry-Pi"><a href="/wiki/Main_Page#Raspberry_Pi">Raspberry Pi</a></li>
			<li id="n-BeagleBone"><a href="/wiki/Main_Page#BeagleBone">BeagleBone</a></li>
			<li id="n-Cubie-Series"><a href="/wiki/Main_Page#Cubie_Series">Cubie Series</a></li>
			<li id="n-Misc"><a href="/wiki/Main_Page#Misc">Misc</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-MCU_Tools' aria-labelledby='p-MCU_Tools-label'>
	<h3 id='p-MCU_Tools-label'>MCU Tools</h3>
	<div class="body">
		<ul>
			<li id="n-STM32"><a href="/wiki/Main_Page#STM32">STM32</a></li>
			<li id="n-STM8"><a href="/wiki/Main_Page#STM8">STM8</a></li>
			<li id="n-LPC"><a href="/wiki/Main_Page#LPC">LPC</a></li>
			<li id="n-AVR"><a href="/wiki/Main_Page#AVR">AVR</a></li>
			<li id="n-PIC-.2F-C8051F"><a href="/wiki/Main_Page#PIC_.2F_C8051F">PIC / C8051F</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-FPGA_Tools' aria-labelledby='p-FPGA_Tools-label'>
	<h3 id='p-FPGA_Tools-label'>FPGA Tools</h3>
	<div class="body">
		<ul>
			<li id="n-ALTERA"><a href="/wiki/Main_Page#ALTERA">ALTERA</a></li>
			<li id="n-XILINX"><a href="/wiki/Main_Page#XILINX">XILINX</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-Arduino_Compatible' aria-labelledby='p-Arduino_Compatible-label'>
	<h3 id='p-Arduino_Compatible-label'>Arduino Compatible</h3>
	<div class="body">
		<ul>
			<li id="n-Main-Board"><a href="/wiki/Main_Page#Main_Board">Main Board</a></li>
			<li id="n-Shields-.2F-Adapters"><a href="/wiki/Main_Page#Shields_.2F_Adapters">Shields / Adapters</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-Modules' aria-labelledby='p-Modules-label'>
	<h3 id='p-Modules-label'>Modules</h3>
	<div class="body">
		<ul>
			<li id="n-OLEDs-.2F-LCDs"><a href="/wiki/Main_Page#OLEDs_.2F_LCDs">OLEDs / LCDs</a></li>
			<li id="n-Wireless-Communication"><a href="/wiki/Main_Page#Wireless_Communication">Wireless Communication</a></li>
			<li id="n-Wired-Communication"><a href="/wiki/Main_Page#Wired_Communication">Wired Communication</a></li>
			<li id="n-Cameras"><a href="/wiki/Main_Page#Cameras">Cameras</a></li>
			<li id="n-Sensors"><a href="/wiki/Main_Page#Sensors">Sensors</a></li>
			<li id="n-Storage"><a href="/wiki/Main_Page#Storage">Storage</a></li>
			<li id="n-Others"><a href="/wiki/Main_Page#Others">others</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-Support' aria-labelledby='p-Support-label'>
	<h3 id='p-Support-label'>Support</h3>
	<div class="body">
		<ul>
			<li id="n-Waveshare"><a href="http://www.waveshare.com" rel="nofollow" target="_blank">Waveshare</a></li>
		</ul>
	</div>
</div>
<div class="portal" role="navigation" id='p-tb' aria-labelledby='p-tb-label'>
	<h3 id='p-tb-label'>Tools</h3>
	<div class="body">
		<ul>
			<li id="t-whatlinkshere"><a href="/wiki/Special:WhatLinksHere/PN532_NFC_HAT" title="A list of all wiki pages that link here [j]" accesskey="j">What links here</a></li>
			<li id="t-recentchangeslinked"><a href="/wiki/Special:RecentChangesLinked/PN532_NFC_HAT" title="Recent changes in pages linked from this page [k]" accesskey="k">Related changes</a></li>
			<li id="t-specialpages"><a href="/wiki/Special:SpecialPages" title="A list of all special pages [q]" accesskey="q">Special pages</a></li>
			<li id="t-print"><a href="/w/index.php?title=PN532_NFC_HAT&amp;printable=yes" rel="alternate" title="Printable version of this page [p]" accesskey="p">Printable version</a></li>
			<li id="t-permalink"><a href="/w/index.php?title=PN532_NFC_HAT&amp;oldid=20726" title="Permanent link to this revision of the page">Permanent link</a></li>
			<li id="t-info"><a href="/w/index.php?title=PN532_NFC_HAT&amp;action=info">Page information</a></li>
		</ul>
	</div>
</div>
			</div>
		</div>
		<div id="footer" role="contentinfo">
							<ul id="footer-info">
											<li id="footer-info-lastmod"> This page was last modified on 15 December 2020, at 07:09.</li>
											<li id="footer-info-viewcount">This page has been accessed 28,279 times.</li>
									</ul>
							<ul id="footer-places">
											<li id="footer-places-privacy"><a href="/wiki/Waveshare_Wiki:Privacy_policy" title="Waveshare Wiki:Privacy policy">Privacy policy</a></li>
											<li id="footer-places-about"><a href="/wiki/Waveshare_Wiki:About" title="Waveshare Wiki:About">About Waveshare Wiki</a></li>
											<li id="footer-places-disclaimer"><a href="/wiki/Waveshare_Wiki:General_disclaimer" title="Waveshare Wiki:General disclaimer">Disclaimers</a></li>
									</ul>
										<ul id="footer-icons" class="noprint">
					<li id="footer-poweredbyico">
						<a href="//www.mediawiki.org/"><img src="/w/skins/common/images/poweredby_mediawiki_88x31.png" alt="Powered by MediaWiki" width="88" height="31" /></a>
					</li>
				</ul>
						<div style="clear:both"></div>
		</div>
		<script data-cfasync="false" src="/cdn-cgi/scripts/5c5dd728/cloudflare-static/email-decode.min.js"></script><script>/*<![CDATA[*/window.jQuery && jQuery.ready();/*]]>*/</script><script>if(window.mw){
mw.loader.state({"site":"loading","user":"ready","user.groups":"ready"});
}</script>
<script>if(window.mw){
mw.loader.load(["mediawiki.action.view.postEdit","mediawiki.user","mediawiki.hidpi","mediawiki.page.ready","ext.FancyBoxThumbs","ext.gadget.edit0","ext.gadget.timer","ext.gadget.collapsible","ext.gadget.msp","ext.gadget.tabber","skins.vector.collapsibleNav"],null,true);
}</script>
<script>
var fbtFancyBoxOptions = {"openEffect":"elastic","closeEffect":"elastic","helpers":{"title":{"type":"inside"}}};
</script>
<script src="https://www.waveshare.com/w/load.php?debug=false&amp;lang=en&amp;modules=site&amp;only=scripts&amp;skin=vector&amp;*"></script>
<script>if(window.mw){
mw.config.set({"wgBackendResponseTime":253});
}</script>
	</body>
</html>
