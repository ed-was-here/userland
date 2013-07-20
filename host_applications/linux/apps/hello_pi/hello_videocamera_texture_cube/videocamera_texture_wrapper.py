import ctypes
import os

testlib = ctypes.CDLL(os.path.abspath('./videocamera_texture.so'))
testlib.myprint()
testlib.main()
