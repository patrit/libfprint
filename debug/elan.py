import usb.core
import usb.util
import array
import numpy as np


# find our device
dev = usb.core.find(idVendor=0x04f3, idProduct=0x0c03)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]

#print(intf)

ep1_out = intf[1]
ep2_in = intf[2]
ep3_in = intf[3]

#print("Enpoints ", ep1_out, ep2_in, ep3_in)

def readRawData():
    ep1_out.write(array.array('B', [0x00, 0x09]))
    # sum = 0x4800 = 18432
    part1 = ep2_in.read(0x2400)
    part2 = ep2_in.read(0x2400)
    return part1 + part2

def readData():
    data = readRawData()
    tmp = [(data[idx*2+1] * 2**8 + data[idx*2])/128 for idx in range(int(len(data)/2))]
    tmp = [(x -40) * 4 for x in tmp]
    print("min=", min(tmp), ", max=", max(tmp), ", thres=", sum([x > 140 for x in tmp]) / (96**2) *100, "%")
    return np.array(tmp).reshape(96, -1)

def init():
    ep1_out.write(array.array('B', [0x00, 0x0c]))
    ep3_in.read(0x40)
    readRawData()
    ep1_out.write(array.array('B', [0x40, 0x7d]))
    ep3_in.read(0x40)
    ep1_out.write(array.array('B', [0x40, 0xbd, 0x12]))
    ep1_out.write(array.array('B', [0x40, 0xa8, 0x78]))
    ep1_out.write(array.array('B', [0x40, 0x68]))
    ep3_in.read(0x40)
    ep1_out.write(array.array('B', [0x40, 0x67]))
    ep3_in.read(0x40)
    ep1_out.write(array.array('B', [0x40, 0x47]))
    ep3_in.read(0x40)
    ep1_out.write(array.array('B', [0x40, 0x87, 0xc0]))
    ep1_out.write(array.array('B', [0x40, 0xa8, 0x97]))
    ep1_out.write(array.array('B', [0x40, 0x8b, 0x72]))


init()
   
import matplotlib.pyplot as plt
img = plt.imshow(readData().astype(float), cmap=plt.cm.gray, vmin = 0, vmax = 255)
plt.show(block=False)
while True:
    img.set_data(readData().astype(float))
    img.figure.canvas.draw()

#import pdb; pdb.set_trace()

