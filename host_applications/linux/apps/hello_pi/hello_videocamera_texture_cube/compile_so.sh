echo "This should run from /opt/vc/src/hello_pi/whatever_name_you_want_to_call_it so it can find ../libs/ilclient and ../libs/vgfont which are in /opt/vc/src/hello_pi"
gcc -shared -Wl,-soname,videocamera_texture -o videocamera_texture.so -fPIC videocamera_texture.c -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi -I/opt/vc/include/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux -I./ -I../libs/ilclient -I../libs/vgfont -g -lilclient -L/opt/vc/lib/ -lGLESv2 -lEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -L../libs/ilclient -L../libs/vgfont -Wl,--no-whole-archive -rdynamic
