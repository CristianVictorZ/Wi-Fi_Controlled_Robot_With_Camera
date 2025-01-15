import socket
import time
import numpy as np
import threading
from scipy import ndimage
from PIL import Image
import PySimpleGUI as sg

#---PySimpleGUI init------------------------------

sg.set_options(font=("Arial Bold", 14))

layout = [
    [sg.Image('OV7670_image.png', expand_x=True, expand_y=True, key='-IMAGE-')],
    [sg.Checkbox("Camera", key='-CAMERA-')],
    [sg.Text('Saturation: '), sg.Radio("320x240", "resolution", key = 'R320-', default = True), 
                             sg.Radio("160x120", "resolution", key = '-R160-'), 
                             sg.Radio("80x60", "resolution", key = '-R80-')],
    [sg.Text('Saturation: '), sg.Radio("-2", "saturation", key = '-S-2-'), 
                             sg.Radio("-1", "saturation", key = '-S-1-'), 
                             sg.Radio("0", "saturation", key = '-S0-', default = True), 
                             sg.Radio("1", "saturation", key = '-S1-'), 
                             sg.Radio("2", "saturation", key = '-S2-')],
    [sg.Text('Brightness: '), sg.Radio("-2", "brightness", key = '-B-2-'), 
                             sg.Radio("-1", "brightness", key = '-B-1-'), 
                             sg.Radio("0", "brightness", key = '-B0-', default = True), 
                             sg.Radio("1", "brightness", key = '-B1-'), 
                             sg.Radio("2", "brightness", key = '-B2-')],
    [sg.Text('Contrast: '), sg.Radio("-2", "contrast", key = '-C-2-'), 
                             sg.Radio("-1", "contrast", key = '-C-1-'), 
                             sg.Radio("0", "contrast", key = '-C0-', default = True), 
                             sg.Radio("1", "contrast", key = '-C1-'), 
                             sg.Radio("2", "contrast", key = '-C2-')],
    [sg.Checkbox("Movement", key='-MOVEMENT-'),  sg.Checkbox("Idle", key = '-MOVE_IDLE-'),
                                                 sg.Checkbox("", key = '-MOVE_SELECT-'), sg.Text("Advance", key = '-MOVE_SELECT_TEXT-')],
    [sg.Checkbox("Forward", key = '-MOVE_ADVANCE-', default = True), 
    sg.Checkbox("Right", key = '-MOVE_ROTATE-')],
    [sg.Text("Status: ", key = '-STAUS_TEXT-')],
    [sg.Button("Exit")]
]

window = sg.Window('Interfata', layout, size=(500, 700), element_justification='c')

#--------------------------------------------------

CAMERA_CONTROL1 = b'\x11'
CAMERA_CONTROL2 = b'\x01'
MOVEMENT_CONTROL = b'\x01'
ACKNOWLEDGE = 6

numberOfBytesPerPixel = 2
xLen = 320
yLen = 240
sizeOfFrame = xLen * yLen * numberOfBytesPerPixel
sizeOfByteSet = 1024
readingImage = 0
imageRead = 0
byteSetSendTimeout = 5

wifiThread = threading.Thread()
imageThread = threading.Thread()
receivedDataStack = []
dataToSendStack = []

HOST = "192.168.4.1"
PORT = 10000

connected = 0
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
time.sleep(0.1)
RECV_BUFFER_SIZE = 2048
s.setblocking(0)
connected = 1

def decodeByteList(byte_list):
    dec_list = []
    for i in range(len(byte_list)):
        dec_list.append(byte_list[i])
    return dec_list

def wifiCommunication():
    global s, connected, dataToSendStack, receivedDataStack
    
    confirmed = 1
    
    while connected == 1:
        try:
            receivedData = s.recv(RECV_BUFFER_SIZE)
            print(receivedData[0])
            if receivedData[0] == 1:
                confirmed = 1
                receivedData = receivedData[1:]
                
            elif receivedData[0] == 16:
                receivedDataStack.append(receivedData[1:])
                sendSet = bytearray()
                sendSet.append(ACKNOWLEDGE)
                s.sendall(sendSet)
                receivedData = receivedData[sizeOfByteSet+1:]
                
            else:
                receivedData = receivedData[1:]
                
            while(len(receivedData) > 0):
                print(receivedData[0])
                if receivedData[0] == 1:
                    confirmed = 1
                    receivedData = receivedData[1:]
                    
                elif receivedData[0] == 16:
                    receivedDataStack.append(receivedData[1:])
                    sendSet = bytearray()
                    sendSet.append(ACKNOWLEDGE)
                    s.sendall(sendSet)
                    receivedData = receivedData[sizeOfByteSet+1:]
                    
                else:
                    receivedData = receivedData[1:]
            
        except:# BlockingIOError:
            None
        
        
        if len(dataToSendStack) > 0 and confirmed == 1:
            print("Data to Send: ", dataToSendStack[0])
            s.sendall(dataToSendStack[0])
            dataToSendStack.pop(0)
            confirmed = 0
    
    s.close()
    
def getAndShowImage():
    
    global readingImage, receivedDataStack, imageRead, xLen, sizeOfFrame
    
    localXlen = xLen
    localFrameSize = sizeOfFrame
    
    print("Getting Image!")
    
    data = []

    time.sleep(0.1)
    readBytes = 0
    
    lastByteSetMoment = time.time()
    
    while readBytes < localFrameSize:
        
        if readBytes + sizeOfByteSet > localFrameSize:
            bytesToRead = localFrameSize - readBytes
        else:
            bytesToRead = sizeOfByteSet

        while True:
            if len(receivedDataStack) > 0:
                if len(receivedDataStack[0]) >= bytesToRead:
                    print(len(receivedDataStack[0]))
                    break
            if time.time() - lastByteSetMoment > byteSetSendTimeout:
                print("ERROR GETTING IMAGE!")
                return None

        dataRaw = receivedDataStack[0]
        receivedDataStack.pop(0)
        lastByteSetMoment = time.time()
        
        dataInterm = np.array(decodeByteList(dataRaw[0:bytesToRead]))
        dataLength = len(dataInterm)
        
        dataInterm[dataInterm==1] = 0
        r = (dataInterm[::2]>>3)<<3
        g = ((dataInterm[::2]%8)*32 + (dataInterm[1::2]>>5)*4)
        b = (dataInterm[1::2]%32*8)
        dataInterm2 = np.zeros(bytesToRead*2, dtype = np.uint8)
        dataInterm2[::4] = r
        dataInterm2[1::4] = g
        dataInterm2[2::4] = b
        dataInterm2[3::4] = 255
        
        data.extend(dataInterm2.astype(np.uint8))
        readBytes = readBytes + dataLength
        
        print(dataLength, readBytes, bytesToRead)
            
        
    frame = np.array(data).reshape(-1, int(localXlen), 4)
    
    img = Image.fromarray(ndimage.rotate(frame, 90))
    
    img = img.resize((320, 240))
    
    img.save("OV7670_image.png")
    
    imageRead = 1
    readingImage = 0

wifiThread = threading.Thread(target = wifiCommunication)
wifiThread.start()

def inputParse():
    
    global CAMERA_CONTROL1, CAMERA_CONTROL2, MOVEMENT_CONTROL, xLen, yLen, sizeOfFrame, numberOfBytesPerPixel
    
    oldCameraControl1 = CAMERA_CONTROL1
    oldCameraControl2 = CAMERA_CONTROL2
    oldMovementControl1 = MOVEMENT_CONTROL
    
    CAMERA_CONTROL1 = bytes([values['-CAMERA-'] * 128 + 
                             values['-R80-'] * 64 +
                             values['-R160-'] * 32 +
                             (values['-S-2-'] | values['-S-1-'])* 16 +
                             (values['-S-2-'] | values['-S2-']) * 8 +
                             (values['-S-1-'] | values['-S1-']) * 4 + 1])
    
    CAMERA_CONTROL2 = bytes([(values['-B-2-'] | values['-B-1-'])* 128 +
                            (values['-B-2-'] | values['-B2-']) * 64 +
                            (values['-B-1-'] | values['-B1-']) * 32 +
                            (values['-C-2-'] | values['-C-1-'])* 16 +
                            (values['-C-2-'] | values['-C2-']) * 8 +
                            (values['-C-1-'] | values['-C1-']) * 4 + 1])
    
    MOVEMENT_CONTROL = bytes([values['-MOVEMENT-'] * 128 + 
                              values['-MOVE_IDLE-'] * 64 +
                              values['-MOVE_SELECT-'] * 32 +
                              (values['-MOVE_ADVANCE-'] & (not values['-MOVE_IDLE-'])) * 16 +
                              (values['-MOVE_ROTATE-']  & (not values['-MOVE_IDLE-'])) * 8 + 1])
    
    if values['-R80-']:
        xLen = 80
        yLen = 60
    elif values['-R160-']:
        xLen = 160
        yLen = 120
    else:
        xLen = 320
        yLen = 240
    
    #print(xLen, yLen, (2**(2 * values['-R80-'] + values['-R160-'])))
    
    sizeOfFrame = xLen * yLen * numberOfBytesPerPixel
    
    return ((oldCameraControl1 != CAMERA_CONTROL1) or
            (oldCameraControl2 != CAMERA_CONTROL2) or
            (oldMovementControl1 != MOVEMENT_CONTROL))

lastSentMessage = time.time()

while True:

    event, values = window.read(timeout=0)

    if event == sg.WIN_CLOSED or event == 'Exit':
        try:
            imageThread.join()
        except:
            None
        dataToSendStack.append(b'\xff')
        connected = 0
        break

    if values['-CAMERA-'] and readingImage == 0:
        
        readingImage = 1
        imageThread = threading.Thread(target = getAndShowImage)
        imageThread.start()
        
    if values['-MOVE_SELECT-']:
        window['-MOVE_SELECT_TEXT-'].update('Directional')
    else:
        window['-MOVE_SELECT_TEXT-'].update('Rotation')

    if imageRead == 1:
        window['-IMAGE-'].update(filename='OV7670_image.png', visible=True)
        window.refresh()
        #window.move_to_center()
        imageRead = 0
        
    if time.time() - lastSentMessage > 0.5:
        
        updateNecesary = inputParse()
        
        if updateNecesary:
            lastSentMessage = time.time()
            registerSet = bytearray()
            registerSet.append(12)
            registerSet.append(CAMERA_CONTROL1[0])
            registerSet.append(CAMERA_CONTROL2[0])
            registerSet.append(MOVEMENT_CONTROL[0])
            
            dataToSendStack.append(registerSet)

window.close()
wifiThread.join()


