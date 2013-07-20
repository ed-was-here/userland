#include <stdio.h>

// a test function that is easy to call from python to check that this works as a .so
void myprint(void);
void myprint()
{
    printf("hello world\n");
}

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bcm_host.h"
#include "ilclient.h"

/* Macro borrowed from omxtx */
#define OERR(cmd) do { \
OMX_ERRORTYPE oerr = cmd; \
if (oerr != OMX_ErrorNone) { \
fprintf(stderr, #cmd " failed on line %d: %x\n", __LINE__, oerr); \
exit(1); \
} \
} while (0)


/* Macro borrowed from omxplayer */
#define OMX_INIT_STRUCTURE(a) \
memset(&(a), 0, sizeof(a)); \
(a).nSize = sizeof(a); \
(a).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
(a).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
(a).nVersion.s.nRevision = OMX_VERSION_REVISION; \
(a).nVersion.s.nStep = OMX_VERSION_STEP



OMX_BUFFERHEADERTYPE* eglBuffer = NULL;
COMPONENT_T* video_render = NULL;
COMPONENT_T* egl_render = NULL;
volatile int quitting  = 0;

void* eglImage = 0;

void my_fill_buffer_done(void* data, COMPONENT_T* comp)
{
  if (quitting)
	{
		printf("Quitting my_fill_buffer_done\n");
		return;
	}
	
	if (video_render != NULL)
	if (OMX_FillThisBuffer(ilclient_get_handle(video_render), eglBuffer) != OMX_ErrorNone)
	{
		printf("OMX_FillThisBuffer failed in callback\n");
		//exit(1);
	}
	
	if (egl_render != NULL)
	if (OMX_FillThisBuffer(ilclient_get_handle(egl_render), eglBuffer) != OMX_ErrorNone)
	{
		printf("OMX_FillThisBuffer failed in callback\n");
		//exit(1);
	}
	printf("[new video frame filled]\n");
}


	 /*
	 Copyright (c) 2012, Broadcom Europe Ltd
	 All rights reserved.

	 Redistribution and use in source and binary forms, with or without
	 modification, are permitted provided that the following conditions are met:
			* Redistributions of source code must retain the above copyright
			 notice, this list of conditions and the following disclaimer.
			* Redistributions in binary form must reproduce the above copyright
			 notice, this list of conditions and the following disclaimer in the
			 documentation and/or other materials provided with the distribution.
			* Neither the name of the copyright holder nor the
			 names of its contributors may be used to endorse or promote products
			 derived from this software without specific prior written permission.

	 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
	 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	 */

	 // Camera demo using OpenMAX IL though the ilcient helper library
	 // modified from hello_video.c by HJ Imbens
	
	 #include <stdio.h>
	 #include <stdlib.h>
	 #include <string.h>
	 #include <time.h>

	 #include "bcm_host.h"
	 #include "ilclient.h"

	 #define kDecoderInputPort 130					
	 #define kDecoderOutputPort 131					

	 #define kSchedulerInputPort 10					
	 #define kSchedulerOutputPort 11					
	 #define kSchedulerClockPort 12					

	 #define kRendererInputPort 90					

	 #define kEGLRendererInputPort 220					
	 #define kEGLRendererImagePort 221

	 #define kClockOutputPort0 80					
	 #define kClockOutputPort1 81					
	 #define kClockOutputPort2 82					
	 #define kClockOutputPort3 83					
	 #define kClockOutputPort4 84					
	 #define kClockOutputPort5 85

	 #define kAudioDecoderInputPort 120					
	 #define kAudioDecoderOutputPort 121					

	 #define kAudioRendererInputPort 100					
	 #define kAudioRendererClockPort 101					

	 #define kAudioMixerClockPort 230					
	 #define kAudioMixerOutputPort 231					
	 #define kAudioMixerInputPort0 232					
	 #define kAudioMixerInputPort1 233					
	 #define kAudioMixerInputPort2 234					
	 #define kAudioMixerInputPort3 235					

	 #define kCameraPreviewPort 70					
	 #define kCameraCapturePort 71					
	 #define kCameraStillImagePort 72					
	 #define kCameraClockPort 73					

	 #define kResizeInputPort 60
	 #define kResizeOutputPort 61
	
	int set_camera_port_resolution(COMPONENT_T *camera, int port, int w, int h, int stride)
	{
		 /* Set the resolution */
		OMX_PARAM_PORTDEFINITIONTYPE portdef;
		OMX_INIT_STRUCTURE(portdef);
		portdef.nPortIndex = port;//kCameraCapturePort;
		OERR(OMX_GetParameter(ILC_GET_HANDLE(camera), OMX_IndexParamPortDefinition, &portdef));
		//printf("w:%d ", portdef.format.image.nFrameWidth);
		//printf("h: %d ", portdef.format.image.nFrameHeight);

		portdef.format.image.nFrameWidth = w;
		portdef.format.image.nFrameHeight = h;
		portdef.format.image.nStride = stride;

		return (OMX_ErrorNone == OMX_SetParameter(ILC_GET_HANDLE(camera), OMX_IndexParamPortDefinition, &portdef));
	}
	
	COMPONENT_T *list[5];
	TUNNEL_T tunnel[4];
	ILCLIENT_T *client;
	
	 //static int camera_test()
	 // Modified function prototype to work with pthreads
	//void *video_camera_test(void* arg)
	int video_camera_test(void* arg, int w, int h)
	{
		//const char* filename = "/opt/vc/src/hello_pi/hello_video/test.h264";
		eglImage = arg;
		
		if (eglImage == 0)
		{
			printf("eglImage is null.\n");
			exit(1);
		}
		OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
		OMX_CONFIG_PORTBOOLEANTYPE cameraport;
		OMX_CONFIG_DISPLAYREGIONTYPE displayconfig;
		COMPONENT_T *camera = NULL, *video_render = NULL, *clock = NULL;
		//COMPONENT_T *egl_render = NULL;
		int status = 0;
		int height = 600;
		//int w = 4*height/3;
		//int h = height;
		int x = 0;//(1280 - 4*height/3)/2;
		int y = 0;//(720 - height)/2;
		int layer = 0;
		
		memset(list, 0, sizeof(list));
		memset(tunnel, 0, sizeof(tunnel));

		if((client = ilclient_init()) == NULL)
		{
			return -3;
		}

		if(OMX_Init() != OMX_ErrorNone)
		{
			ilclient_destroy(client);
			return -4;
		}
		
		// callback
		ilclient_set_fill_buffer_done_callback(client, my_fill_buffer_done, 0);

		printf("create objects\n");
		// create video_decode
		if(ilclient_create_component(client, &camera, "camera", ILCLIENT_DISABLE_ALL_PORTS) != 0)
			status = -14;
		list[0] = camera;

		// create video_render
		if(status == 0 && ilclient_create_component(client, &video_render, "video_render", ILCLIENT_DISABLE_ALL_PORTS) != 0)
			status = -14;
		list[1] = video_render;

		// ADDED: egl_render
		if(status == 0 && ilclient_create_component(client, &egl_render, "egl_render", ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_OUTPUT_BUFFERS) != 0)
		//if(status == 0 && ilclient_create_component(client, &egl_render, "egl_render", ILCLIENT_DISABLE_ALL_PORTS) != 0)
		 status = -14;
		list[2] = egl_render;

		
		// create clock
		if(status == 0 && ilclient_create_component(client, &clock, "clock", ILCLIENT_DISABLE_ALL_PORTS) != 0)
			status = -14;
		list[3] = clock;

		/////
		{
			int s = ceilf(w/32.0)*32;
			int ret = set_camera_port_resolution(camera, kCameraCapturePort, w, h, s);
			ret += set_camera_port_resolution(camera, kCameraPreviewPort, w, h, s);
			printf("ok:%d\n",ret);
		}
		/////
		
		printf("enable camera\n");
		// enable the capture port of the camera
		memset(&cameraport, 0, sizeof(cameraport));
		cameraport.nSize = sizeof(cameraport);
		cameraport.nVersion.nVersion = OMX_VERSION;
		cameraport.nPortIndex = kCameraCapturePort;
		cameraport.bEnabled = OMX_TRUE;
		if(camera != NULL && OMX_SetParameter(ILC_GET_HANDLE(camera), OMX_IndexConfigPortCapturing, &cameraport) != OMX_ErrorNone)
			status = -13;

		
		// configure the renderer to display the content in a 4:3 rectangle in the middle of a 1280x720 screen
		memset(&displayconfig, 0, sizeof(displayconfig));
		displayconfig.nSize = sizeof(displayconfig);
		displayconfig.nVersion.nVersion = OMX_VERSION;
		displayconfig.set = (OMX_DISPLAYSETTYPE)(OMX_DISPLAY_SET_FULLSCREEN | OMX_DISPLAY_SET_DEST_RECT | OMX_DISPLAY_SET_LAYER);
		displayconfig.nPortIndex = kRendererInputPort;
		displayconfig.fullscreen = (w > 0 && h > 0) ? OMX_FALSE : OMX_TRUE;
		displayconfig.dest_rect.x_offset = x;
		displayconfig.dest_rect.y_offset = y;
		displayconfig.dest_rect.width = w;
		displayconfig.dest_rect.height = h;
		displayconfig.layer = layer;
		printf ("dest_rect: %d,%d,%d,%d\n", x, y, w, h);
		printf ("layer: %d\n", (int)displayconfig.layer);
		if (video_render != NULL && OMX_SetParameter(ILC_GET_HANDLE(video_render), OMX_IndexConfigDisplayRegion, &displayconfig) != OMX_ErrorNone) {
			status = -13;
			printf ("OMX_IndexConfigDisplayRegion failed\n");
		}

///////////
		printf("set_tunnels\n");
		// create a tunnel from the camera to the video_render component
		set_tunnel(tunnel+0, camera, kCameraCapturePort, video_render, kRendererInputPort);
		// create a tunnel from the clock to the camera
		set_tunnel(tunnel+1, clock, kClockOutputPort0, camera, kCameraClockPort);
		
		
		printf("setup_tunnels\n");
		// setup both tunnels
		if(status == 0 && ilclient_setup_tunnel(tunnel+0, 0, 0) != 0) {
			status = -15;
		}
		if(status == 0 && ilclient_setup_tunnel(tunnel+1, 0, 0) != 0) {
			status = -15;
		}
///////////////
		/////////// set up egl_render
		//if (once)
		{
			//once = 0;
			printf("setup stuff\n");
			set_tunnel(tunnel+2, camera, kCameraPreviewPort, egl_render, kEGLRendererInputPort); // egl_render
			if(status == 0 && ilclient_setup_tunnel(tunnel+2, 0, 0) != 0) {
				status = -15;
			}
				// Set egl_render to idle
				ilclient_change_component_state(egl_render, OMX_StateIdle);

				// Enable the output port and tell egl_render to use the texture as a buffer
				//ilclient_enable_port(egl_render, kEGLRendererImagePort); THIS BLOCKS SO CANT BE USED
				if (OMX_SendCommand(ILC_GET_HANDLE(egl_render), OMX_CommandPortEnable, kEGLRendererImagePort, NULL) != OMX_ErrorNone)
				{
					printf("OMX_CommandPortEnable failed.\n");
					status = -16;
				}

				if (OMX_UseEGLImage(ILC_GET_HANDLE(egl_render), &eglBuffer, kEGLRendererImagePort, NULL, eglImage) != OMX_ErrorNone)
				{
					printf("OMX_UseEGLImage failed.\n");
					status = -16;
				}

				// Set egl_render to executing
				ilclient_change_component_state(egl_render, OMX_StateExecuting);
			
				// Request egl_render to write data to the texture buffer
				if(OMX_FillThisBuffer(ILC_GET_HANDLE(egl_render), eglBuffer) != OMX_ErrorNone)
				{
					printf("OMX_FillThisBuffer failed.\n");
					status = -16;
				}
			printf("port_settings_changedDone\n");
		}
/////////////////
		// change state of components to executing
		ilclient_change_component_state(camera, OMX_StateExecuting);
		ilclient_change_component_state(video_render, OMX_StateExecuting);
		
		ilclient_change_component_state(clock, OMX_StateExecuting);

		// start the camera by changing the clock state to running
		memset(&cstate, 0, sizeof(cstate));
		cstate.nSize = sizeof(displayconfig);
		cstate.nVersion.nVersion = OMX_VERSION;
		cstate.eState = OMX_TIME_ClockStateRunning;
		OMX_SetParameter (ILC_GET_HANDLE(clock), OMX_IndexConfigTimeClockState, &cstate);

		
		printf("main loop\n");
		return status;
	}
	
	int end_video_camera_test()
	{
		printf("Quitting preview...\n");
		quitting = 1; // set flag to stop the callback from copying data
		ilclient_disable_tunnel(tunnel);
		ilclient_disable_tunnel(tunnel+1);
		ilclient_disable_tunnel(tunnel+2);
		ilclient_teardown_tunnels(tunnel);
		printf("close\n");
		
		ilclient_state_transition(list, OMX_StateIdle);
		
		// TODO: This hangs.  The code is never executed in the example so it might be a bug
		// The docs say ilclient_state_transition waits for the components to complete the state transition,
		// so one or more may be stuck
		//ilclient_state_transition(list, OMX_StateLoaded);
		
		printf("cleanup\n");
		ilclient_cleanup_components(list);
		
		printf("omx deinit\n");
		OMX_Deinit();
		
		printf("ilclient destroy\n");
		ilclient_destroy(client);
		printf("Done quitting preview\n");
		
		return 0;
	}
/*
	int main (int argc, char **argv)
	{
		bcm_host_init();
		return camera_test();
	}
*/


//////////////////////////////////////////////////////////////////////////////////////////////////////

/*
Copyright (c) 2012, Broadcom Europe Ltd
Copyright (c) 2012, OtherCrashOverride
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	 * Redistributions of source code must retain the above copyright
		notice, this list of conditions and the following disclaimer.
	 * Redistributions in binary form must reproduce the above copyright
		notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
	 * Neither the name of the copyright holder nor the
		names of its contributors may be used to endorse or promote products
		derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// A rotating cube rendered with OpenGL|ES. Three images used as textures on the cube faces.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include "bcm_host.h"

#include "GLES/gl.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"

#include "cube_texture_and_coords.h"

//#include "triangle.h"
//#include <pthread.h>


#define PATH "./"
// The resolution to capture video at
#define IMAGE_SIZE_WIDTH (1920/2)
#define IMAGE_SIZE_HEIGHT (1080/2)

#ifndef M_PI
	#define M_PI 3.141592654
#endif
  

typedef struct
{
	uint32_t screen_width;
	uint32_t screen_height;
// OpenGL|ES objects
	EGLDisplay display;
	EGLSurface surface;
	EGLContext context;
	GLuint tex;
// model rotation vector and direction
	GLfloat rot_angle_x_inc;
	GLfloat rot_angle_y_inc;
	GLfloat rot_angle_z_inc;
// current model rotation angles
	GLfloat rot_angle_x;
	GLfloat rot_angle_y;
	GLfloat rot_angle_z;
// current distance from camera
	GLfloat distance;
	GLfloat distance_inc;
} CUBE_STATE_T;

static void init_ogl(CUBE_STATE_T *state);
static void init_model_proj(CUBE_STATE_T *state);
static void reset_model(CUBE_STATE_T *state);
static GLfloat inc_and_wrap_angle(GLfloat angle, GLfloat angle_inc);
static GLfloat inc_and_clip_distance(GLfloat distance, GLfloat distance_inc);
static void redraw_scene(CUBE_STATE_T *state);
static void update_model(CUBE_STATE_T *state);
static void init_textures(CUBE_STATE_T *state);
static void exit_func(void);
static volatile int terminate;
static CUBE_STATE_T _state, *state=&_state;

//static void* eglImage = 0;
static pthread_t thread1;


/***********************************************************
 * Name: init_ogl
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the display, OpenGL|ES context and screen stuff
 *
 * Returns: void
 *
 ***********************************************************/
static void init_ogl(CUBE_STATE_T *state)
{
	int32_t success = 0;
	EGLBoolean result;
	EGLint num_config;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	static const EGLint attribute_list[] =
	{
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_DEPTH_SIZE, 16,
		//EGL_SAMPLES, 4,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_NONE
	};
	
	EGLConfig config;

	// get an EGL display connection
	state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	assert(state->display!=EGL_NO_DISPLAY);

	// initialize the EGL display connection
	result = eglInitialize(state->display, NULL, NULL);
	assert(EGL_FALSE != result);

	// get an appropriate EGL frame buffer configuration
	// this uses a BRCM extension that gets the closest match, rather than standard which returns anything that matches
	result = eglSaneChooseConfigBRCM(state->display, attribute_list, &config, 1, &num_config);
	assert(EGL_FALSE != result);

	// create an EGL rendering context
	state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, NULL);
	assert(state->context!=EGL_NO_CONTEXT);

	// create an EGL window surface
	success = graphics_get_display_size(0 /* LCD */, &state->screen_width, &state->screen_height);
	assert( success >= 0 );

	dst_rect.x = 0;
	dst_rect.y = 0;
	dst_rect.width = state->screen_width;
	dst_rect.height = state->screen_height;
		
	src_rect.x = 0;
	src_rect.y = 0;
	src_rect.width = state->screen_width << 16;
	src_rect.height = state->screen_height << 16;		  

	dispman_display = vc_dispmanx_display_open( 0 /* LCD */);
	dispman_update = vc_dispmanx_update_start( 0 );
			
	dispman_element = vc_dispmanx_element_add ( dispman_update, dispman_display,
		0/*layer*/, &dst_rect, 0/*src*/,
		&src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);
		
	nativewindow.element = dispman_element;
	nativewindow.width = state->screen_width;
	nativewindow.height = state->screen_height;
	vc_dispmanx_update_submit_sync( dispman_update );
		
	state->surface = eglCreateWindowSurface( state->display, config, &nativewindow, NULL );
	assert(state->surface != EGL_NO_SURFACE);

	// connect the context to the surface
	result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
	assert(EGL_FALSE != result);

	// Set background color and clear buffers
	glClearColor(0.15f, 0.25f, 0.35f, 1.0f);

	// Enable back face culling.
	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_MODELVIEW);
}

/***********************************************************
 * Name: init_model_proj
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Sets the OpenGL|ES model to default values
 *
 * Returns: void
 *
 ***********************************************************/
static void init_model_proj(CUBE_STATE_T *state)
{
	float nearp = 1.0f;
	float farp = 500.0f;
	float hht;
	float hwd;

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	glViewport(0, 0, (GLsizei)state->screen_width, (GLsizei)state->screen_height);
		
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	hht = nearp * (float)tan(45.0 / 2.0 / 180.0 * M_PI);
	hwd = hht * (float)state->screen_width / (float)state->screen_height;

	glFrustumf(-hwd, hwd, -hht, hht, nearp, farp);
	
	glEnableClientState( GL_VERTEX_ARRAY );
	glVertexPointer( 3, GL_BYTE, 0, quadx );

	reset_model(state);
}

/***********************************************************
 * Name: reset_model
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Resets the Model projection and rotation direction
 *
 * Returns: void
 *
 ***********************************************************/
static void reset_model(CUBE_STATE_T *state)
{
	// reset model position
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.f, 0.f, -50.f);

	// reset model rotation
	state->rot_angle_x = 45.f; state->rot_angle_y = 30.f; state->rot_angle_z = 0.f;
	state->rot_angle_x_inc = 0.5f; state->rot_angle_y_inc = 0.5f; state->rot_angle_z_inc = 0.f;
	state->distance = 2000.f;
}

/***********************************************************
 * Name: update_model
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description: Updates model projection to current position/rotation
 *
 * Returns: void
 *
 ***********************************************************/
static void update_model(CUBE_STATE_T *state)
{
	// update position
	state->rot_angle_x = inc_and_wrap_angle(state->rot_angle_x, state->rot_angle_x_inc);
	state->rot_angle_y = inc_and_wrap_angle(state->rot_angle_y, state->rot_angle_y_inc);
	state->rot_angle_z = inc_and_wrap_angle(state->rot_angle_z, state->rot_angle_z_inc);
	state->distance	 = inc_and_clip_distance(state->distance, state->distance_inc);

	glLoadIdentity();
	// move camera back to see the cube
	glTranslatef(0.f, 0.f, -state->distance);

	// Rotate model to new position
	glRotatef(state->rot_angle_x, 1.f, 0.f, 0.f);
	glRotatef(state->rot_angle_y, 0.f, 1.f, 0.f);
	glRotatef(state->rot_angle_z, 0.f, 0.f, 1.f);
}

/***********************************************************
 * Name: inc_and_wrap_angle
 *
 * Arguments:
 *		 GLfloat angle	  current angle
 *		 GLfloat angle_inc angle increment
 *
 * Description:	Increments or decrements angle by angle_inc degrees
 *					 Wraps to 0 at 360 deg.
 *
 * Returns: new value of angle
 *
 ***********************************************************/
static GLfloat inc_and_wrap_angle(GLfloat angle, GLfloat angle_inc)
{
	angle += angle_inc;

	if (angle >= 360.0)
		angle -= 360.f;
	else if (angle <=0)
		angle += 360.f;

	return angle;
}

/***********************************************************
 * Name: inc_and_clip_distance
 *
 * Arguments:
 *		 GLfloat distance	  current distance
 *		 GLfloat distance_inc distance increment
 *
 * Description:	Increments or decrements distance by distance_inc units
 *					 Clips to range
 *
 * Returns: new value of angle
 *
 ***********************************************************/
static GLfloat inc_and_clip_distance(GLfloat distance, GLfloat distance_inc)
{
	distance += distance_inc;

	if (distance >= 120.0f)
		distance = 120.f;
	else if (distance <= 40.0f)
		distance = 40.0f;

	return distance;
}

#include <time.h>
#include <unistd.h>

/***********************************************************
 * Name: redraw_scene
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description:	Draws the model and calls eglSwapBuffers
 *					 to render to screen
 *
 * Returns: void
 *
 ***********************************************************/
static void redraw_scene(CUBE_STATE_T *state)
{
	
	// Start with a clear screen
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	//glDrawArrays( GL_TRIANGLE_STRIP, 0, 4*6); TODO: fix the texture coordinates of the model and use this
	//or use the existing code below...
	
	// Need to rotate textures - do this by rotating each cube face
	glRotatef(270.f, 0.f, 0.f, 1.f ); // front face normal along z axis
	
	// draw first 4 vertices
	glDrawArrays( GL_TRIANGLE_STRIP, 0, 4);

	// same pattern for other 5 faces - rotation chosen to make image orientation 'nice'
	glRotatef(90.f, 0.f, 0.f, 1.f ); // back face normal along z axis
	glDrawArrays( GL_TRIANGLE_STRIP, 4, 4);

	glRotatef(90.f, 1.f, 0.f, 0.f ); // left face normal along x axis
	glDrawArrays( GL_TRIANGLE_STRIP, 8, 4);

	glRotatef(90.f, 1.f, 0.f, 0.f ); // right face normal along x axis
	glDrawArrays( GL_TRIANGLE_STRIP, 12, 4);

	glRotatef(270.f, 0.f, 1.f, 0.f ); // top face normal along y axis
	glDrawArrays( GL_TRIANGLE_STRIP, 16, 4);

	glRotatef(90.f, 0.f, 1.f, 0.f ); // bottom face normal along y axis
	glDrawArrays( GL_TRIANGLE_STRIP, 20, 4);
	
	eglSwapBuffers(state->display, state->surface);
	printf("[egl frame]");
}

/***********************************************************
 * Name: init_textures
 *
 * Arguments:
 *		 CUBE_STATE_T *state - holds OGLES model info
 *
 * Description:	Initialise OGL|ES texture surfaces to use image
 *					 buffers
 *
 * Returns: void
 *
 ***********************************************************/
static void init_textures(CUBE_STATE_T *state)
{
	//// load three texture buffers but use them on six OGL|ES texture surfaces
	glGenTextures(1, &state->tex);

	glBindTexture(GL_TEXTURE_2D, state->tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, IMAGE_SIZE_WIDTH, IMAGE_SIZE_HEIGHT, 0,
					 GL_RGBA, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


	/* Create EGL Image */
	eglImage = eglCreateImageKHR(
					 state->display,
					 state->context,
					 EGL_GL_TEXTURE_2D_KHR,
					 (EGLClientBuffer)state->tex,
					 0);
	 
	if (eglImage == EGL_NO_IMAGE_KHR)
	{
		printf("eglCreateImageKHR failed.\n");
		exit(1);
	}

	// Start rendering
	//pthread_create(&thread1, NULL, video_decode_test, eglImage);
	// We don't need a thread for the video capture ->
	video_camera_test(eglImage, IMAGE_SIZE_WIDTH, IMAGE_SIZE_HEIGHT);
	
	// setup overall texture environment
	glTexCoordPointer(2, GL_FLOAT, 0, texCoords);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glEnable(GL_TEXTURE_2D);

	// Bind texture surface to current vertices
	glBindTexture(GL_TEXTURE_2D, state->tex);
}
//------------------------------------------------------------------------------

static void exit_func(void)
// Function to be passed to atexit().
{
	printf("closing\n");

	if (eglImage != 0)
	{
		if (!eglDestroyImageKHR(state->display, (EGLImageKHR) eglImage))
			printf("eglDestroyImageKHR failed.");
	}
	
	printf(".\n");

	// clear screen
	glClear( GL_COLOR_BUFFER_BIT );
	eglSwapBuffers(state->display, state->surface);

	printf(".\n");

	// Release OpenGL resources
	eglMakeCurrent( state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT );
	printf(".\n");
	eglDestroySurface( state->display, state->surface );
	printf(".\n");
	eglDestroyContext( state->display, state->context );
	printf(".\n");
	eglTerminate( state->display );

	printf("\ncube closed\n");
} // exit_func()

//==============================================================================

int main ()
{
	bcm_host_init();
	printf("Note: ensure you have sufficient gpu_mem configured\n");

	// Clear application state
	memset( state, 0, sizeof( *state ) );
		
	// Start OGLES
	init_ogl(state);

	// Setup the model world
	init_model_proj(state);

	// initialise the OGLES texture(s)
	init_textures(state);

	//while (!terminate)
	int n;
	for (n=0;n<200;n++)
	{
		update_model(state);
		redraw_scene(state);
	}
	printf("Finished - closing now...");
	end_video_camera_test();
	
	exit_func();
	return 0;
}
