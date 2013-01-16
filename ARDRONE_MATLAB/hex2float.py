import sys
import struct

def hex2float(x):
    
    hexstring = '%s' % (x)
    floatnum = struct.unpack('!f', hexstring.decode('hex'))[0]
    return floatnum

if __name__ == '__main__':
    x = hex2float(sys.argv[1])
    print x