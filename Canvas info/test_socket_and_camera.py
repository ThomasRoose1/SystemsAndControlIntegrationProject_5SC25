import socket
import time
import cv2

serverAddressPort=("192.168.141.7", 49001)
bufferSize=32

#create UDP socket
UDPClientSocket=socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

#set up camera capture
cv2.namedWindow('test')
video=cv2.VideoCapture(0)

#send something
bytesToSend=bytearray([130,75,0,0,200,3])

#print(bytesToSend)

UDPClientSocket.sendto(bytesToSend, serverAddressPort)

for i in range(1,100):
    #catch frame
    ret, frame=video.read()
    cv2.imshow('test', frame)
    cv2.waitKey(2)

    #find ball position
    bytesToSend=bytearray([130,i,0,0,200,3])

    #send ball position to MicrolabBox
    #print(bytesToSend)
    UDPClientSocket.sendto(bytesToSend, serverAddressPort)
    


video.release()
cv2.destroyAllWindows()



