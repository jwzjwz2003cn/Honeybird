import sys
import struct

def hex2int32(x):
    
    hexstring = '%s' % (x)
    floatnum = struct.unpack('!i', hexstring.decode('hex'))[0]
    return floatnum

if __name__ == '__main__':
    x = hex2int32(sys.argv[1])
    print x