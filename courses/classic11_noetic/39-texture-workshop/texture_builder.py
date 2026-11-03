#!/usr/bin/env python3
"""Create a real PNG with stdlib only; changing this file changes actual appearance."""
from pathlib import Path
import struct,zlib

def chunk(name,payload):
 return struct.pack('>I',len(payload))+name+payload+struct.pack('>I',zlib.crc32(name+payload)&0xffffffff)
def build():
 width=height=128;cell_pixels=32
 yellow=(240,205,30);blue=(20,60,210)
 scanlines=b''.join(b'\0'+b''.join(bytes(yellow if (x//cell_pixels+y//cell_pixels)%2==0 else blue) for x in range(width)) for y in range(height))
 data=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(scanlines))+chunk(b'IEND',b'')
 target=Path('models/texture_cards/materials/textures/checker.png');target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
 print('Offline checker PNG',target,len(data),'bytes')
if __name__=='__main__':build()
