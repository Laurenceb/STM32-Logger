import serial

ser=serial.Serial("/dev/rfcomm3",timeout=0)

while 1:
	for n in range(256):
		ser.write(chr(n))
		ret=ser.read()
		if len(ret):
			print ord(ret)

