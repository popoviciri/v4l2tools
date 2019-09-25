/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** v4l2multi_stream_mmal.cpp
**
** Read YUV420 from a V4L2 capture -> compress in H264 using OMX -> write to a V4L2 output device
**
** -------------------------------------------------------------------------*/

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <signal.h>
#include <semaphore.h>
#include <fcntl.h>

#include <fstream>

extern "C"
{
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiHelpers.h"
//#include "RaspiGPS.h"
}

#include "logger.h"

#include "V4l2Device.h"
#include "V4l2Capture.h"
#include "V4l2Output.h"

//#include "encode_omx.h"

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms

int stop=0;

typedef struct MMALCAM_STATE_S MMALCAM_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   MMALCAM_STATE *pstate;               /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   //char *cb_buff;                       /// Circular buffer
   //int   cb_len;                        /// Length of buffer
   //int   cb_wptr;                       /// Current write pointer
   //int   cb_wrap;                       /// Has buffer wrapped at least once?
   //int   cb_data;                       /// Valid bytes in buffer
//#define IFRAME_BUFSIZE (60*1000)
//   int   iframe_buff[IFRAME_BUFSIZE];          /// buffer of iframe pointers
//   int   iframe_buff_wpos;
//   int   iframe_buff_rpos;
//   char  header_bytes[29];
//   int  header_wptr;
   FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
   //FILE *raw_file_handle;               /// File handle to write raw data to.
   int  flush_buffers;
   //FILE *pts_file_handle;               /// File timestamps
   int64_t mjpeg_frame_duration_ms;
} PORT_USERDATA;

/** Possible raw output formats
 */
typedef enum
{
   RAW_OUTPUT_FMT_YUV = 0,
   RAW_OUTPUT_FMT_RGB,
   RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;


/** Structure containing all state information for the current run
 */
struct MMALCAM_STATE_S
{
   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
   //int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   MMAL_FOURCC_T video_encoding;       /// Requested codec video encoding (MJPEG or H264)
   MMAL_FOURCC_T mjpeg_encoding;       /// Requested codec video encoding (MJPEG or H264)
   MMAL_FOURCC_T still_encoding;       /// Encoding to use for the output file.
   int jpeg_quality;                   /// JPEG quality setting (1-100)
   int jpeg_restart_interval;          /// JPEG restart interval. 0 for none.
   MMAL_PARAM_THUMBNAIL_CONFIG_T thumbnailConfig; // JPEG thumbnail
   int mjpeg_width;                    /// Requested MJPEG width
   int mjpeg_height;                   /// Requested MJPEG height
   int mjpeg_bitrate;                  /// Requested MJPEG bitrate
   int mjpeg_framerate;                /// Requested MJPEG frame rate (fps)
   int bitrate;                        /// Requested bitrate
   int framerate;                      /// Requested frame rate (fps)
   int intraperiod;                    /// Intra-refresh period (key frame rate)
   int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
   int bInlineHeaders;                  /// Insert inline headers to stream (SPS, PPS)
   //int demoMode;                       /// Run app in demo mode
   //int demoInterval;                   /// Interval between camera settings changes
   int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
   /// the camera output or the encoder output (with compression artifacts)
   int profile;                        /// H264 profile to use for encoding
   int level;                          /// H264 level to use for encoding
   //int waitMethod;                     /// Method for switching between pause and capture

   //int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
   //int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

   //int segmentSize;                    /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
   //int segmentWrap;                    /// Point at which to wrap segment counter
   //int segmentNumber;                  /// Current segment counter
   //int splitNow;                       /// Split at next possible i-frame if set to 1.
   //int splitWait;                      /// Switch if user wants splited files

   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
   MMAL_COMPONENT_T *isp_component;       /// Pointer to the isp component
   MMAL_COMPONENT_T *video_encoder_component; /// Pointer to the encoder component
   MMAL_COMPONENT_T *mjpeg_encoder_component; /// Pointer to the encoder component
   MMAL_COMPONENT_T *still_encoder_component; /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
   MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
   MMAL_CONNECTION_T *isp_connection;     /// Pointer to the connection from camera to isp
   MMAL_CONNECTION_T *video_encoder_connection; /// Pointer to the connection from camera to encoder
   MMAL_CONNECTION_T *mjpeg_encoder_connection; /// Pointer to the connection from camera to encoder
   MMAL_CONNECTION_T *still_encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *isp_pool; /// Pointer to the pool of buffers used by splitter output port 0
   MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
   MMAL_POOL_T *video_encoder_pool; /// Pointer to the pool of buffers used by encoder output port
   MMAL_POOL_T *mjpeg_encoder_pool; /// Pointer to the pool of buffers used by encoder output port
   MMAL_POOL_T *still_encoder_pool; /// Pointer to the pool of buffers used by encoder output port

   PORT_USERDATA callback_data;        /// Used to move data to the encoder callback

   int bCapturing;                     /// State of capture/pause
   //int bCircularBuffer;                /// Whether we are writing to a circular buffer

   int inlineMotionVectors;             /// Encoder outputs inline Motion Vectors
   char *imv_filename;                  /// filename of inline Motion Vectors output
   //int raw_output;                      /// Output raw video from camera as well
   RAW_OUTPUT_FMT raw_output_fmt;       /// The raw video format
   //char *raw_filename;                  /// Filename for raw video output
   int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.
   int frame;
   //char *pts_filename;
   //int save_pts;
   //int64_t starttime;
   //int64_t lasttime;

   //bool netListen;
   MMAL_BOOL_T addSPSTiming;
   int slices;
   char *v4l2loopback_dev;              /// v4l2loopback device for video output
   V4l2Output* videoOutput;
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
   {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
   {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
   {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T  level_map[] =
{
   {"4",           MMAL_VIDEO_LEVEL_H264_4},
   {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
   {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T  initial_map[] =
{
   {"record",     0},
   {"pause",      1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

static XREF_T  intra_refresh_map[] =
{
   {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
   {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
   {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
   {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

//static XREF_T  raw_output_fmt_map[] =
//{
//   {"yuv",  RAW_OUTPUT_FMT_YUV},
//   {"rgb",  RAW_OUTPUT_FMT_RGB},
//   {"gray", RAW_OUTPUT_FMT_GRAY},
//};
//
//static int raw_output_fmt_map_size = sizeof(raw_output_fmt_map) / sizeof(raw_output_fmt_map[0]);

//static struct
//{
//   char *format;
//   MMAL_FOURCC_T encoding;
//} still_encoding_xref[] =
//{
//   {"jpg", MMAL_ENCODING_JPEG},
//   {"bmp", MMAL_ENCODING_BMP},
//   {"gif", MMAL_ENCODING_GIF},
//   {"png", MMAL_ENCODING_PNG},
//   {"ppm", MMAL_ENCODING_PPM},
//   {"tga", MMAL_ENCODING_TGA}
//};
//
//static int still_encoding_xref_size = sizeof(still_encoding_xref) / sizeof(still_encoding_xref[0]);


/// Command ID's and Structure defining our command line options
enum
{
   CommandV4L2LoopbackDev,
   CommandQuality,
   CommandBitrate,
   CommandMjpegBitrate,
   CommandMjpegFramerate,
   CommandMjpegWidth,
   CommandMjpegHeight,
   //CommandTimeout,
   //CommandDemoMode,
   CommandFramerate,
   //CommandPreviewEnc,
   CommandIntraPeriod,
   CommandProfile,
   //CommandTimed,
   //CommandSignal,
   //CommandKeypress,
   //CommandInitialState,
   CommandQP,
   CommandInlineHeaders,
   //CommandSegmentFile,
   //CommandSegmentWrap,
   //CommandSegmentStart,
   //CommandSplitWait,
   //CommandCircular,
   CommandIMV,
   CommandIntraRefreshType,
   CommandFlush,
   //CommandSavePTS,
   //CommandCodec,
   CommandLevel,
   CommandRaw,
   CommandRawFormat,
   //CommandNetListen,
   CommandSPSTimings,
   //CommandSlices,
   CommandRestartInterval,
};

static COMMAND_LIST cmdline_commands[] =
{
   { CommandV4L2LoopbackDev, "-videoout", "vo", "V4L2Loopback device for video output (default /dev/video90)", 1 },
   { CommandQuality,       "-quality",    "q",  "Set jpeg quality <0 to 100>", 1 },
   { CommandBitrate,       "-bitrate",    "b",  "Set bitrate. Use bits per second (e.g. 10MBits/s would be -b 10000000)", 1 },
   { CommandMjpegBitrate,  "-mjpegbitrate", "mjb", "Set MJPEG bitrate. Use bits per second (e.g. 10MBits/s would be -b 10000000)", 1 },
   { CommandMjpegFramerate,"-mjpegframerate", "mjfps","Specify the MJPEG frames per second to record", 1},
   { CommandMjpegWidth,    "-mjpegwidth", "mjw", "Set MJPEG output width", 1 },
   { CommandMjpegHeight,   "-mjpegheight", "mjh", "Set MJPEG output height", 1 },
   //{ CommandTimeout,       "-timeout",    "t",  "Time (in ms) to capture for. If not specified, set to 5s. Zero to disable", 1 },
   //{ CommandDemoMode,      "-demo",       "d",  "Run a demo mode (cycle through range of camera options, no capture)", 1},
   { CommandFramerate,     "-framerate",  "fps","Specify the frames per second to record", 1},
   //{ CommandPreviewEnc,    "-penc",       "e",  "Display preview image *after* encoding (shows compression artifacts)", 0},
   { CommandIntraPeriod,   "-intra",      "g",  "Specify the intra refresh period (key frame rate/GoP size). Zero to produce an initial I-frame and then just P-frames.", 1},
   { CommandProfile,       "-profile",    "pf", "Specify H264 profile to use for encoding", 1},
   //{ CommandTimed,         "-timed",      "td", "Cycle between capture and pause. -cycle on,off where on is record time and off is pause time in ms", 0},
   //{ CommandSignal,        "-signal",     "s",  "Cycle between capture and pause on Signal", 0},
   //{ CommandKeypress,      "-keypress",   "k",  "Cycle between capture and pause on ENTER", 0},
   //{ CommandInitialState,  "-initial",    "i",  "Initial state. Use 'record' or 'pause'. Default 'record'", 1},
   { CommandQP,            "-qp",         "qp", "Quantisation parameter. Use approximately 10-40. Default 0 (off)", 1},
   { CommandInlineHeaders, "-inline",     "ih", "Insert inline headers (SPS, PPS) to stream", 0},
   //{ CommandSegmentFile,   "-segment",    "sg", "Segment output file in to multiple files at specified interval <ms>", 1},
   //{ CommandSegmentWrap,   "-wrap",       "wr", "In segment mode, wrap any numbered filename back to 1 when reach number", 1},
   //{ CommandSegmentStart,  "-start",      "sn", "In segment mode, start with specified segment number", 1},
   //{ CommandSplitWait,     "-split",      "sp", "In wait mode, create new output file for each start event", 0},
   //{ CommandCircular,      "-circular",   "c",  "Run encoded data through circular buffer until triggered then save", 0},
   { CommandIMV,           "-vectors",    "x",  "Output filename <filename> for inline motion vectors", 1 },
   { CommandIntraRefreshType,"-irefresh", "if", "Set intra refresh type", 1},
   { CommandFlush,         "-flush",      "fl",  "Flush buffers in order to decrease latency", 0 },
   //{ CommandSavePTS,       "-save-pts",   "pts","Save Timestamps to file for mkvmerge", 1 },
   //{ CommandCodec,         "-codec",      "cd", "Specify the codec to use - H264 (default) or MJPEG", 1 },
   { CommandLevel,         "-level",      "lev","Specify H264 level to use for encoding", 1},
   //{ CommandRaw,           "-raw",        "r",  "Output filename <filename> for raw video", 1 },
   //{ CommandRawFormat,     "-raw-format", "rf", "Specify output format for raw video. Default is yuv", 1},
   //{ CommandNetListen,     "-listen",     "l", "Listen on a TCP socket", 0},
   { CommandSPSTimings,    "-spstimings",    "stm", "Add in h.264 sps timings", 0},
   //{ CommandSlices   ,     "-slices",     "sl", "Horizontal slices per frame. Default 1 (off)", 1},
   { CommandRestartInterval, "-restart","rs","JPEG Restart interval (default of 0 for none)", 1},
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);


/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(MMALCAM_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(MMALCAM_STATE));

   raspicommonsettings_set_defaults(&state->common_settings);

   // Now set anything non-zero
   state->v4l2loopback_dev = "/dev/video90";
   state->jpeg_quality = 85;
   state->still_encoding = MMAL_ENCODING_JPEG;
   state->jpeg_restart_interval = 0;
   state->thumbnailConfig.enable = 0;
   state->thumbnailConfig.width = 64;
   state->thumbnailConfig.height = 48;
   state->thumbnailConfig.quality = 35;
   //state->timeout = -1; // replaced with 5000ms later if unset
   state->mjpeg_width = 640;
   state->mjpeg_height = 480;
   state->common_settings.width = 640;
   state->common_settings.height = 480;
   state->mjpeg_encoding = MMAL_ENCODING_MJPEG;
   state->video_encoding = MMAL_ENCODING_H264;
   state->mjpeg_bitrate = 2000000;
   state->mjpeg_framerate = 5;
   state->bitrate = 2000000;
   state->framerate = VIDEO_FRAME_RATE_NUM;
   state->intraperiod = -1;    // Not set
   state->quantisationParameter = 0;
   //state->demoMode = 0;
   //state->demoInterval = 250; // ms
   state->immutableInput = 1;
   state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   state->level = MMAL_VIDEO_LEVEL_H264_4;
   //state->waitMethod = WAIT_METHOD_NONE;
   //state->onTime = 5000;
   //state->offTime = 5000;
   state->bCapturing = 0;
   state->bInlineHeaders = 1;
   //state->segmentSize = 0;  // 0 = not segmenting the file.
   //state->segmentNumber = 1;
   //state->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
   //state->splitNow = 0;
   //state->splitWait = 0;
   state->inlineMotionVectors = 0;
   state->intra_refresh_type = -1;
   state->frame = 0;
   //state->save_pts = 0;
   //state->netListen = false;
   state->addSPSTiming = MMAL_FALSE;
   state->slices = 1;


   // Setup preview window defaults
   //raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

static void check_camera_model(int cam_num)
{
   MMAL_COMPONENT_T *camera_info;
   MMAL_STATUS_T status;

   // Try to get the camera name
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
   if (status == MMAL_SUCCESS)
   {
      MMAL_PARAMETER_CAMERA_INFO_T param;
      param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
      param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);

      if (status != MMAL_SUCCESS)
      {
         // Running on newer firmware
         param.hdr.size = sizeof(param);
         status = mmal_port_parameter_get(camera_info->control, &param.hdr);
         if (status == MMAL_SUCCESS && param.num_cameras > cam_num)
         {
            if (!strncmp(param.cameras[cam_num].camera_name, "toshh2c", 7))
            {
               fprintf(stderr, "The driver for the TC358743 HDMI to CSI2 chip you are using is NOT supported.\n");
               fprintf(stderr, "They were written for a demo purposes only, and are in the firmware on an as-is\n");
               fprintf(stderr, "basis and therefore requests for support or changes will not be acted on.\n\n");
            }
         }
      }

      mmal_component_destroy(camera_info);
   }
}

/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(MMALCAM_STATE *state)
{
   //int i;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   raspicommonsettings_dump_parameters(&state->common_settings);

   //fprintf(stderr, "JPEG Quality %d\n", state->jpeg_quality);
   fprintf(stderr, "H264 bitrate %d\n", state->bitrate);
   fprintf(stderr, "H264 framerate %d\n", state->framerate);
   fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state->profile, profile_map, profile_map_size));
   fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state->level, level_map, level_map_size));
   fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state->quantisationParameter, state->bInlineHeaders ? "Yes" : "No");
   fprintf(stderr, "H264 Fill SPS Timings %s\n", state->addSPSTiming ? "Yes" : "No");
   fprintf(stderr, "H264 Intra refresh type %s, period %d\n", raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size), state->intraperiod);
   //fprintf(stderr, "H264 Slices %d\n", state->slices);
   fprintf(stderr, "\n");
   fprintf(stderr, "MJPEG bitrate %d\n", state->mjpeg_bitrate);
   fprintf(stderr, "MJPEG framerate %d\n", state->mjpeg_framerate);
   fprintf(stderr, "MJPEG width %d, height %d\n", state->mjpeg_width, state->mjpeg_height);
   fprintf(stderr, "\n");

   // Not going to display segment data unless asked for it.
   //if (state->segmentSize)
   //   fprintf(stderr, "Segment size %d, segment wrap value %d, initial segment number %d\n", state->segmentSize, state->segmentWrap, state->segmentNumber);

   //if (state->raw_output)
   //   fprintf(stderr, "Raw output enabled, format %s\n", raspicli_unmap_xref(state->raw_output_fmt, raw_output_fmt_map, raw_output_fmt_map_size));

   //fprintf(stderr, "Wait method : ");
   //for (i=0; i<wait_method_description_size; i++)
   //{
   //   if (state->waitMethod == wait_method_description[i].nextWaitMethod)
   //      fprintf(stderr, "%s", wait_method_description[i].description);
   //}
   fprintf(stderr, "\nInitial state '%s'\n", raspicli_unmap_xref(state->bCapturing, initial_map, initial_map_size));
   fprintf(stderr, "\n\n");

   //raspipreview_dump_parameters(&state->preview_parameters);
   raspicamcontrol_dump_parameters(&state->camera_parameters);
}

/**
 * Display usage information for the application to stdout
 *
 * @param app_name String to display as the application name
 */
static void application_help_message(char *app_name)
{
   int i;

   fprintf(stdout, "\nusage: %s [options]\n\n", app_name);

   fprintf(stdout, "Image parameter commands\n\n");

   raspicli_display_help(cmdline_commands, cmdline_commands_size);

   // Profile options
   fprintf(stdout, "\n\nH264 Profile options :\n%s", profile_map[0].mode );

   for (i=1; i<profile_map_size; i++)
   {
      fprintf(stdout, ",%s", profile_map[i].mode);
   }

   // Level options
   fprintf(stdout, "\n\nH264 Level options :\n%s", level_map[0].mode );

   for (i=1; i<level_map_size; i++)
   {
      fprintf(stdout, ",%s", level_map[i].mode);
   }

   // Intra refresh options
   fprintf(stdout, "\n\nH264 Intra refresh options :\n%s", intra_refresh_map[0].mode );

   for (i=1; i<intra_refresh_map_size; i++)
   {
      fprintf(stdout, ",%s", intra_refresh_map[i].mode);
   }

   // Raw output format options
   //fprintf(stdout, "\n\nRaw output format options :\n%s", raw_output_fmt_map[0].mode );
   //
   //for (i=1; i<raw_output_fmt_map_size; i++)
   //{
   //   fprintf(stdout, ",%s", raw_output_fmt_map[i].mode);
   //}

   fprintf(stdout, "\n\n");

   return;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return Non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, const char **argv, MMALCAM_STATE *state)
{
   // Parse the command line arguments.
   // We are looking for --<something> or -<abbreviation of something>

   int valid = 1;
   int i;

   for (i = 1; i < argc && valid; i++)
   {
      int command_id, num_parameters;

      if (!argv[i])
         continue;

      if (argv[i][0] != '-')
      {
         valid = 0;
         continue;
      }

      // Assume parameter is valid until proven otherwise
      valid = 1;

      command_id = raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

      // If we found a command but are missing a parameter, continue (and we will drop out of the loop)
      if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc) )
         continue;

      //  We are now dealing with a command line option
      switch (command_id)
      {
      //case CommandQuality: // Quality = 1-100
      //   if (sscanf(argv[i + 1], "%u", &state->jpeg_quality) == 1)
      //   {
      //      if (state->jpeg_quality > 100)
      //      {
      //         fprintf(stderr, "Setting max JPEG quality = 100\n");
      //         state->jpeg_quality = 100;
      //      }
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;

      case CommandBitrate: // 1-100
         if (sscanf(argv[i + 1], "%u", &state->bitrate) == 1)
         {
            i++;
         }
         else
            valid = 0;

         break;

      case CommandMjpegBitrate: // 1-100
         if (sscanf(argv[i + 1], "%u", &state->mjpeg_bitrate) == 1)
         {
            i++;
         }
         else
            valid = 0;

         break;

      case CommandMjpegWidth:
         if (sscanf(argv[i + 1], "%u", &state->mjpeg_width) == 1)
            i++;
         else
            valid = 0;
         break;

      case CommandMjpegHeight:
         if (sscanf(argv[i + 1], "%u", &state->mjpeg_height) == 1)
            i++;
         else
            valid = 0;
         break;

      //case CommandTimeout: // Time to run viewfinder/capture
      //{
      //   if (sscanf(argv[i + 1], "%d", &state->timeout) == 1)
      //   {
      //      // Ensure that if previously selected a waitMethod we don't overwrite it
      //      if (state->timeout == 0 && state->waitMethod == WAIT_METHOD_NONE)
      //         state->waitMethod = WAIT_METHOD_FOREVER;
      //
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandDemoMode: // Run in demo mode - no capture
      //{
      //   // Demo mode might have a timing parameter
      //   // so check if a) we have another parameter, b) its not the start of the next option
      //   if (i + 1 < argc  && argv[i+1][0] != '-')
      //   {
      //      if (sscanf(argv[i + 1], "%u", &state->demoInterval) == 1)
      //      {
      //         // TODO : What limits do we need for timeout?
      //         if (state->demoInterval == 0)
      //            state->demoInterval = 250; // ms
      //
      //         state->demoMode = 1;
      //         i++;
      //      }
      //      else
      //         valid = 0;
      //   }
      //   else
      //   {
      //      state->demoMode = 1;
      //   }
      //
      //   break;
      //}

      case CommandMjpegFramerate: // fps to record
      {
         if (sscanf(argv[i + 1], "%u", &state->mjpeg_framerate) == 1)
         {
            // TODO : What limits do we need for fps 1 - 30 - 120??
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandFramerate: // fps to record
      {
         if (sscanf(argv[i + 1], "%u", &state->framerate) == 1)
         {
            // TODO : What limits do we need for fps 1 - 30 - 120??
            i++;
         }
         else
            valid = 0;
         break;
      }

      //case CommandPreviewEnc:
      //   state->immutableInput = 0;
      //   break;

      case CommandIntraPeriod: // key frame rate
      {
         if (sscanf(argv[i + 1], "%u", &state->intraperiod) == 1)
            i++;
         else
            valid = 0;
         break;
      }

      case CommandQP: // quantisation parameter
      {
         if (sscanf(argv[i + 1], "%u", &state->quantisationParameter) == 1)
            i++;
         else
            valid = 0;
         break;
      }

      case CommandProfile: // H264 profile
      {
         state->profile = raspicli_map_xref(argv[i + 1], profile_map, profile_map_size);

         if( state->profile == -1)
            state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;

         i++;
         break;
      }

      case CommandInlineHeaders: // H264 inline headers
      {
         state->bInlineHeaders = 1;
         break;
      }

      //case CommandTimed:
      //{
      //   if (sscanf(argv[i + 1], "%u,%u", &state->onTime, &state->offTime) == 2)
      //   {
      //      i++;
      //
      //      if (state->onTime < 1000)
      //         state->onTime = 1000;
      //
      //      if (state->offTime < 1000)
      //         state->offTime = 1000;
      //
      //      state->waitMethod = WAIT_METHOD_TIMED;
      //
      //      if (state->timeout == -1)
      //         state->timeout = 0;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandKeypress:
      //   state->waitMethod = WAIT_METHOD_KEYPRESS;
      //
      //   if (state->timeout == -1)
      //      state->timeout = 0;
      //
      //   break;

      //case CommandSignal:
      //   state->waitMethod = WAIT_METHOD_SIGNAL;
      //   // Reenable the signal
      //   signal(SIGUSR1, default_signal_handler);
      //
      //   if (state->timeout == -1)
      //      state->timeout = 0;
      //
      //   break;

      //case CommandInitialState:
      //{
      //   state->bCapturing = raspicli_map_xref(argv[i + 1], initial_map, initial_map_size);
      //
      //   if( state->bCapturing == -1)
      //      state->bCapturing = 0;
      //
      //   i++;
      //   break;
      //}

      //case CommandSegmentFile: // Segment file in to chunks of specified time
      //{
      //   if (sscanf(argv[i + 1], "%u", &state->segmentSize) == 1)
      //   {
      //      // Must enable inline headers for this to work
      //      state->bInlineHeaders = 1;
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandSegmentWrap: // segment wrap value
      //{
      //   if (sscanf(argv[i + 1], "%u", &state->segmentWrap) == 1)
      //      i++;
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandSegmentStart: // initial segment number
      //{
      //   if((sscanf(argv[i + 1], "%u", &state->segmentNumber) == 1) && (!state->segmentWrap || (state->segmentNumber <= state->segmentWrap)))
      //      i++;
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandSplitWait: // split files on restart
      //{
      //   // Must enable inline headers for this to work
      //   state->bInlineHeaders = 1;
      //   state->splitWait = 1;
      //   break;
      //}

      //case CommandCircular:
      //{
      //   state->bCircularBuffer = 1;
      //   break;
      //}

      case CommandIMV:  // output filename
      {
         state->inlineMotionVectors = 1;
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->imv_filename = (char *)malloc(len + 1);
            vcos_assert(state->imv_filename);
            if (state->imv_filename)
               strncpy(state->imv_filename, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandIntraRefreshType:
      {
         state->intra_refresh_type = raspicli_map_xref(argv[i + 1], intra_refresh_map, intra_refresh_map_size);
         i++;
         break;
      }

      case CommandFlush:
      {
         state->callback_data.flush_buffers = 1;
         break;
      }
      //case CommandSavePTS:  // output filename
      //{
      //   state->save_pts = 1;
      //   int len = strlen(argv[i + 1]);
      //   if (len)
      //   {
      //      state->pts_filename = malloc(len + 1);
      //      vcos_assert(state->pts_filename);
      //      if (state->pts_filename)
      //         strncpy(state->pts_filename, argv[i + 1], len+1);
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}
      //case CommandCodec:  // codec type
      //{
      //   int len = strlen(argv[i + 1]);
      //   if (len)
      //   {
      //      if (len==4 && !strncmp("H264", argv[i+1], 4))
      //         state->encoding = MMAL_ENCODING_H264;
      //      else  if (len==5 && !strncmp("MJPEG", argv[i+1], 5))
      //         state->encoding = MMAL_ENCODING_MJPEG;
      //      else
      //         valid = 0;
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}

      case CommandLevel: // H264 level
      {
         state->level = raspicli_map_xref(argv[i + 1], level_map, level_map_size);

         if( state->level == -1)
            state->level = MMAL_VIDEO_LEVEL_H264_4;

         i++;
         break;
      }

      case CommandV4L2LoopbackDev:  // v4l2loopback device for video output
      {
         int len = strlen(argv[i + 1]);
         if (len)
         {
            state->v4l2loopback_dev = (char *)malloc(len + 1);
            vcos_assert(state->v4l2loopback_dev);
            if (state->v4l2loopback_dev)
               strncpy(state->v4l2loopback_dev, argv[i + 1], len+1);
            i++;
         }
         else
            valid = 0;
         break;
      }

      //case CommandRaw:  // output filename
      //{
      //   state->raw_output = 1;
      //   //state->raw_output_fmt defaults to 0 / yuv
      //   int len = strlen(argv[i + 1]);
      //   if (len)
      //   {
      //      state->raw_filename = malloc(len + 1);
      //      vcos_assert(state->raw_filename);
      //      if (state->raw_filename)
      //         strncpy(state->raw_filename, argv[i + 1], len+1);
      //      i++;
      //   }
      //   else
      //      valid = 0;
      //   break;
      //}

      //case CommandRawFormat:
      //{
      //   state->raw_output_fmt = raspicli_map_xref(argv[i + 1], raw_output_fmt_map, raw_output_fmt_map_size);
      //
      //   if (state->raw_output_fmt == -1)
      //      valid = 0;
      //
      //   i++;
      //   break;
      //}

      //case CommandNetListen:
      //{
      //   state->netListen = true;
      //
      //   break;
      //}
      //case CommandSlices:
      //{
      //   if ((sscanf(argv[i + 1], "%d", &state->slices) == 1) && (state->slices > 0))
      //      i++;
      //   else
      //      valid = 0;
      //   break;
      //}

      case CommandSPSTimings:
      {
         state->addSPSTiming = MMAL_TRUE;

         break;
      }

      case CommandRestartInterval:
      {
         if (sscanf(argv[i + 1], "%u", &state->jpeg_restart_interval) == 1)
         {
            i++;
         }
         else
            valid = 0;
         break;
      }

      default:
      {
         // Try parsing for any image specific parameters
         // result indicates how many parameters were used up, 0,1,2
         // but we adjust by -1 as we have used one already
         const char *second_arg = (i + 1 < argc) ? argv[i + 1] : NULL;
         int parms_used = (raspicamcontrol_parse_cmdline(&state->camera_parameters, &argv[i][1], second_arg));

         // Still unused, try common settings
         if (!parms_used)
            parms_used = raspicommonsettings_parse_cmdline(&state->common_settings, &argv[i][1], second_arg, &application_help_message);

         // Still unused, try preview options
         //if (!parms_used)
         //   parms_used = raspipreview_parse_cmdline(&state->preview_parameters, &argv[i][1], second_arg);

         // If no parms were used, this must be a bad parameter
         if (!parms_used)
            valid = 0;
         else
            i += parms_used - 1;

         break;
      }
      }
   }

   if (!valid)
   {
      fprintf(stderr, "Invalid command line option (%s)\n", argv[i-1]);
      return 1;
   }

   return 0;
}

/**
 * Update any annotation data specific to the video.
 * This simply passes on the setting from cli, or
 * if application defined annotate requested, updates
 * with the H264 parameters
 *
 * @param state Pointer to state control struct
 *
 */
static void update_annotation_data(MMALCAM_STATE *state)
{
   // So, if we have asked for a application supplied string, set it to the H264 or GPS parameters
   if (state->camera_parameters.enable_annotate & ANNOTATE_APP_TEXT)
   {
      char *text;

      if (state->common_settings.gps)
      {
         //text = raspi_gps_location_string();
      }
      else
      {
         const char *refresh = raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size);

         asprintf(&text,  "%dk,%df,%s,%d,%s,%s",
                  state->bitrate / 1000,  state->framerate,
                  refresh ? refresh : "(none)",
                  state->intraperiod,
                  raspicli_unmap_xref(state->profile, profile_map, profile_map_size),
                  raspicli_unmap_xref(state->level, level_map, level_map_size));
      }

      raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, text,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );

      free(text);
   }
   else
   {
      raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, state->camera_parameters.annotate_string,
                                   state->camera_parameters.annotate_text_size,
                                   state->camera_parameters.annotate_text_colour,
                                   state->camera_parameters.annotate_bg_colour,
                                   state->camera_parameters.annotate_justify,
                                   state->camera_parameters.annotate_x,
                                   state->camera_parameters.annotate_y
                                  );
   }
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(MMALCAM_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   status = (MMAL_STATUS_T)raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
   if (status == MMAL_SUCCESS)
      status = (MMAL_STATUS_T)raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
   if (status == MMAL_SUCCESS)
      status = (MMAL_STATUS_T)raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set stereo mode : error %d", status);
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   MMAL_PARAMETER_INT32_T camera_num =
   {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, default_camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->common_settings.width,
         .max_stills_h = state->common_settings.height,
         .stills_yuv422 = 0,
         .one_shot_stills = 0,
         .max_preview_video_w = state->common_settings.width,
         .max_preview_video_h = state->common_settings.height,
         .num_preview_video_frames = 3 + vcos_max(0, (state->framerate-30)/10),
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
      };
      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   // Now set up the port formats

   // Set the encode format on the Preview port
   // HW limitations mean we need the preview to be the same size as the required recorded output

   format = preview_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 50, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 166, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }

   //enable dynamic framerate if necessary
   if (state->camera_parameters.shutter_speed)
   {
      if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
      {
         state->framerate=0;
         if (state->common_settings.verbose)
            fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
      }
   }

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera viewfinder format couldn't be set");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   // Set the encode format on the video  port

   format = video_port->format;
   format->encoding_variant = MMAL_ENCODING_I420;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 50, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 167, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera video format couldn't be set");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera still format couldn't be set");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera component couldn't be enabled");
      if (camera)
         mmal_component_destroy(camera);
      return status;
   }

   // Note: this sets lots of parameters that were not individually addressed before.
   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   state->camera_component = camera;

   update_annotation_data(state);

   if (state->common_settings.verbose)
      fprintf(stderr, "Camera component done\n");

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(MMALCAM_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
//static MMAL_STATUS_T create_splitter_component(MMALCAM_STATE *state)
//{
//   MMAL_COMPONENT_T *splitter = 0;
//   MMAL_PORT_T *splitter_output = NULL;
//   MMAL_ES_FORMAT_T *format;
//   MMAL_STATUS_T status;
//   MMAL_POOL_T *pool;
//   int i;
//
//   if (state->camera_component == NULL)
//   {
//      status = MMAL_ENOSYS;
//      vcos_log_error("Camera component must be created before splitter");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   /* Create the component */
//   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Failed to create splitter component");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   if (!splitter->input_num)
//   {
//      status = MMAL_ENOSYS;
//      vcos_log_error("Splitter doesn't have any input port");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   if (splitter->output_num < 2)
//   {
//      status = MMAL_ENOSYS;
//      vcos_log_error("Splitter doesn't have enough output ports");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   /* Ensure there are enough buffers to avoid dropping frames: */
//   mmal_format_copy(splitter->input[0]->format, state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);
//
//   if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
//      splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
//
//   status = mmal_port_format_commit(splitter->input[0]);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to set format on splitter input port");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   /* Splitter can do format conversions, configure format for its output port: */
//   for (i = 0; i < splitter->output_num; i++)
//   {
//      mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);
//
//      if (i == SPLITTER_OUTPUT_PORT)
//      {
//         format = splitter->output[i]->format;
//
//         switch (state->raw_output_fmt)
//         {
//         case RAW_OUTPUT_FMT_YUV:
//         case RAW_OUTPUT_FMT_GRAY: /* Grayscale image contains only luma (Y) component */
//            format->encoding = MMAL_ENCODING_I420;
//            format->encoding_variant = MMAL_ENCODING_I420;
//            break;
//         case RAW_OUTPUT_FMT_RGB:
//            if (mmal_util_rgb_order_fixed(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT]))
//               format->encoding = MMAL_ENCODING_RGB24;
//            else
//               format->encoding = MMAL_ENCODING_BGR24;
//            format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
//            break;
//         default:
//            status = MMAL_EINVAL;
//            vcos_log_error("unknown raw output format");
//            if (splitter)
//               mmal_component_destroy(splitter);
//            return status;
//         }
//      }
//
//      status = mmal_port_format_commit(splitter->output[i]);
//
//      if (status != MMAL_SUCCESS)
//      {
//         vcos_log_error("Unable to set format on splitter output port %d", i);
//         if (splitter)
//            mmal_component_destroy(splitter);
//         return status;
//      }
//   }
//
//   /* Enable component */
//   status = mmal_component_enable(splitter);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("splitter component couldn't be enabled");
//      if (splitter)
//         mmal_component_destroy(splitter);
//      return status;
//   }
//
//   /* Create pool of buffer headers for the output port to consume */
//   splitter_output = splitter->output[SPLITTER_OUTPUT_PORT];
//   pool = mmal_port_pool_create(splitter_output, splitter_output->buffer_num, splitter_output->buffer_size);
//
//   if (!pool)
//   {
//      vcos_log_error("Failed to create buffer header pool for splitter output port %s", splitter_output->name);
//   }
//
//   state->splitter_pool = pool;
//   state->splitter_component = splitter;
//
//   if (state->common_settings.verbose)
//      fprintf(stderr, "Splitter component done\n");
//
//   return status;
//}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(MMALCAM_STATE *state)
{
   // Get rid of any port buffers first
   if (state->splitter_pool)
   {
      mmal_port_pool_destroy(state->splitter_component->output[SPLITTER_OUTPUT_PORT], state->splitter_pool);
   }

   if (state->splitter_component)
   {
      mmal_component_destroy(state->splitter_component);
      state->splitter_component = NULL;
   }
}

/**
 * Create the ISP component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_isp_component(MMALCAM_STATE *state)
{
   MMAL_COMPONENT_T *isp = 0;
   //MMAL_PORT_T *isp_output = NULL;
   //MMAL_ES_FORMAT_T *format;
   MMAL_STATUS_T status;
   //MMAL_POOL_T *pool;

   if (state->camera_component == NULL)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera component must be created before isp");
      if (isp)
         mmal_component_destroy(isp);
      return status;
   }

   /* Create the component */
   status = mmal_component_create("vc.ril.isp", &isp);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create isp component");
      if (isp)
         mmal_component_destroy(isp);
      return status;
   }

   MMAL_PORT_T *port = isp->output[0];

   port->format->encoding = MMAL_ENCODING_I420;
   port->format->es->video.width = VCOS_ALIGN_UP(state->mjpeg_width, 32);
   port->format->es->video.height = VCOS_ALIGN_UP(state->mjpeg_height, 16);
   port->format->es->video.crop.x = 0;
   port->format->es->video.crop.y = 0;
   port->format->es->video.crop.width = state->mjpeg_width;
   port->format->es->video.crop.height = state->mjpeg_height;
   port->format->es->video.frame_rate.num = state->framerate;
   port->format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;
   status = mmal_port_format_commit(port);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to commit port format on isp output");
      if (isp)
         mmal_component_destroy(isp);
      return status;
   }

   /* Enable component */
   status = mmal_component_enable(isp);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("isp component couldn't be enabled");
      if (isp)
         mmal_component_destroy(isp);
      return status;
   }

   /* Create pool of buffer headers for the output port to consume */
   //isp_output = isp->output[0];
   //pool = mmal_port_pool_create(isp_output, isp_output->buffer_num, isp_output->buffer_size);
   //
   //if (!pool)
   //{
   //   vcos_log_error("Failed to create buffer header pool for isp output port %s", isp_output->name);
   //}
   //
   //state->isp_pool = pool;
   state->isp_component = isp;

   if (state->common_settings.verbose)
      fprintf(stderr, "isp component done\n");

   return status;
}

/**
 * Destroy the isp component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_isp_component(MMALCAM_STATE *state)
{
   // Get rid of any port buffers first
   if (state->isp_pool)
   {
      mmal_port_pool_destroy(state->isp_component->output[1], state->isp_pool);
   }

   if (state->isp_component)
   {
      mmal_component_destroy(state->isp_component);
      state->isp_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_video_encoder_component(MMALCAM_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      if (encoder)
         mmal_component_destroy(encoder);
      state->video_encoder_component = NULL;
      return status;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      if (encoder)
         mmal_component_destroy(encoder);
      state->video_encoder_component = NULL;
      return status;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Only supporting H264 at the moment
   encoder_output->format->encoding = state->video_encoding;

   if(state->video_encoding == MMAL_ENCODING_H264)
   {
      if(state->level == MMAL_VIDEO_LEVEL_H264_4)
      {
         if(state->bitrate > MAX_BITRATE_LEVEL4)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL4;
         }
      }
      else
      {
         if(state->bitrate > MAX_BITRATE_LEVEL42)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
            state->bitrate = MAX_BITRATE_LEVEL42;
         }
      }
   }
   else if(state->video_encoding == MMAL_ENCODING_MJPEG)
   {
      if(state->bitrate > MAX_BITRATE_MJPEG)
      {
         fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
         state->bitrate = MAX_BITRATE_MJPEG;
      }
   }

   encoder_output->format->bitrate = state->bitrate;

   //if (state->video_encoding == MMAL_ENCODING_H264)
   //   encoder_output->buffer_size = encoder_output->buffer_size_recommended << 1;
   //else
      encoder_output->buffer_size = 256<<10;
   fprintf(stderr, "h264 buffer size: %u\n", encoder_output->buffer_size);

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // We need to set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = 0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      if (encoder)
         mmal_component_destroy(encoder);
      state->video_encoder_component = NULL;
      return status;
   }

   // Set the rate control parameter
   if (0)
   {
      MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set ratecontrol");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }

   }

   if (state->video_encoding == MMAL_ENCODING_H264 &&
         state->intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }
   }

   if (state->video_encoding == MMAL_ENCODING_H264 && state->slices > 1 && state->common_settings.width <= 1280)
   {
      int frame_mb_rows = VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4;

      if (state->slices > frame_mb_rows) //warn user if too many slices selected
      {
         fprintf(stderr,"H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state->slices, frame_mb_rows, frame_mb_rows);
         // Continue rather than abort..
      }
      int slice_row_mb = frame_mb_rows/state->slices;
      if (frame_mb_rows - state->slices*slice_row_mb)
         slice_row_mb++; //must round up to avoid extra slice if not evenly divided

      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set number of slices");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }
   }

   if (state->video_encoding == MMAL_ENCODING_H264 &&
       state->quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }
   }

   if (state->video_encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

      if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            state->level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
            status = MMAL_EINVAL;
            if (encoder)
               mmal_component_destroy(encoder);
            state->video_encoder_component = NULL;
            return status;
         }
      }

      param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         if (encoder)
            mmal_component_destroy(encoder);
         state->video_encoder_component = NULL;
         return status;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   if (state->video_encoding == MMAL_ENCODING_H264)
   {
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE HEADER FLAG parameters");
         // Continue rather than abort..
      }

      //set flag for add SPS TIMING
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
         // Continue rather than abort..
      }

      //set INLINE VECTORS flag to request motion vector estimates
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE VECTORS parameters");
         // Continue rather than abort..
      }

      // Adaptive intra refresh settings
      if ( state->intra_refresh_type != -1)
      {
         MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
         param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
         param.hdr.size = sizeof(param);

         // Get first so we don't overwrite anything unexpectedly
         status = mmal_port_parameter_get(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
         }

         param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)state->intra_refresh_type;

         //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
         //   param.cir_mbs = 10;

         status = mmal_port_parameter_set(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to set H264 intra-refresh values");
            if (encoder)
               mmal_component_destroy(encoder);
            state->video_encoder_component = NULL;
            return status;
         }
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      if (encoder)
         mmal_component_destroy(encoder);
      state->video_encoder_component = NULL;
      return status;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->video_encoder_pool = pool;
   state->video_encoder_component = encoder;

   if (state->common_settings.verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_mjpeg_encoder_component(MMALCAM_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      if (encoder)
         mmal_component_destroy(encoder);
      state->mjpeg_encoder_component = NULL;
      return status;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      if (encoder)
         mmal_component_destroy(encoder);
      state->mjpeg_encoder_component = NULL;
      return status;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Only supporting H264 at the moment
   encoder_output->format->encoding = state->mjpeg_encoding;

   if(state->mjpeg_encoding == MMAL_ENCODING_H264)
   {
      if(state->level == MMAL_VIDEO_LEVEL_H264_4)
      {
         if(state->mjpeg_bitrate > MAX_BITRATE_LEVEL4)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->mjpeg_bitrate = MAX_BITRATE_LEVEL4;
         }
      }
      else
      {
         if(state->mjpeg_bitrate > MAX_BITRATE_LEVEL42)
         {
            fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
            state->mjpeg_bitrate = MAX_BITRATE_LEVEL42;
         }
      }
   }
   else if(state->mjpeg_encoding == MMAL_ENCODING_MJPEG)
   {
      if(state->mjpeg_bitrate > MAX_BITRATE_MJPEG)
      {
         fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
         state->mjpeg_bitrate = MAX_BITRATE_MJPEG;
      }
   }

   encoder_output->format->bitrate = state->mjpeg_bitrate;

   if (state->mjpeg_encoding == MMAL_ENCODING_H264)
      encoder_output->buffer_size = encoder_output->buffer_size_recommended;
   else
      encoder_output->buffer_size = 256<<10;
   fprintf(stderr, "mjpeg buffer size: %u\n", encoder_output->buffer_size);

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // We need to set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = 0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      if (encoder)
         mmal_component_destroy(encoder);
      state->mjpeg_encoder_component = NULL;
      return status;
   }

   // Set the rate control parameter
   if (0)
   {
      MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set ratecontrol");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }

   }

   if (state->mjpeg_encoding == MMAL_ENCODING_H264 &&
         state->intraperiod != -1)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set intraperiod");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }
   }

   if (state->mjpeg_encoding == MMAL_ENCODING_H264 && state->slices > 1 && state->common_settings.width <= 1280)
   {
      int frame_mb_rows = VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4;

      if (state->slices > frame_mb_rows) //warn user if too many slices selected
      {
         fprintf(stderr,"H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state->slices, frame_mb_rows, frame_mb_rows);
         // Continue rather than abort..
      }
      int slice_row_mb = frame_mb_rows/state->slices;
      if (frame_mb_rows - state->slices*slice_row_mb)
         slice_row_mb++; //must round up to avoid extra slice if not evenly divided

      status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set number of slices");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }
   }

   if (state->mjpeg_encoding == MMAL_ENCODING_H264 &&
       state->quantisationParameter)
   {
      MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set initial QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }

      MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param2.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set min QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }

      MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
      status = mmal_port_parameter_set(encoder_output, &param3.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set max QP");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }
   }

   if (state->mjpeg_encoding == MMAL_ENCODING_H264)
   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

      if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate > 245760)
      {
         if((VCOS_ALIGN_UP(state->common_settings.width,16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height,16) >> 4) * state->framerate <= 522240)
         {
            fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
            state->level=MMAL_VIDEO_LEVEL_H264_42;
         }
         else
         {
            vcos_log_error("Too many macroblocks/s requested");
            status = MMAL_EINVAL;
            if (encoder)
               mmal_component_destroy(encoder);
            state->mjpeg_encoder_component = NULL;
            return status;
         }
      }

      param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
         if (encoder)
            mmal_component_destroy(encoder);
         state->mjpeg_encoder_component = NULL;
         return status;
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   if (state->mjpeg_encoding == MMAL_ENCODING_H264)
   {
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE HEADER FLAG parameters");
         // Continue rather than abort..
      }

      //set flag for add SPS TIMING
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
         // Continue rather than abort..
      }

      //set INLINE VECTORS flag to request motion vector estimates
      if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
      {
         vcos_log_error("failed to set INLINE VECTORS parameters");
         // Continue rather than abort..
      }

      // Adaptive intra refresh settings
      if ( state->intra_refresh_type != -1)
      {
         MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
         param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
         param.hdr.size = sizeof(param);

         // Get first so we don't overwrite anything unexpectedly
         status = mmal_port_parameter_get(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
         }

         param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)state->intra_refresh_type;

         //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
         //   param.cir_mbs = 10;

         status = mmal_port_parameter_set(encoder_output, &param.hdr);
         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to set H264 intra-refresh values");
            if (encoder)
               mmal_component_destroy(encoder);
            state->mjpeg_encoder_component = NULL;
            return status;
         }
      }
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      if (encoder)
         mmal_component_destroy(encoder);
      state->mjpeg_encoder_component = NULL;
      return status;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->mjpeg_encoder_pool = pool;
   state->mjpeg_encoder_component = encoder;

   if (state->common_settings.verbose)
      fprintf(stderr, "Encoder component done\n");

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_video_encoder_component(MMALCAM_STATE *state)
{
   // Get rid of any port buffers first
   if (state->video_encoder_pool)
   {
      mmal_port_pool_destroy(state->video_encoder_component->output[0], state->video_encoder_pool);
   }

   if (state->video_encoder_component)
   {
      mmal_component_destroy(state->video_encoder_component);
      state->video_encoder_component = NULL;
   }
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_mjpeg_encoder_component(MMALCAM_STATE *state)
{
   // Get rid of any port buffers first
   if (state->mjpeg_encoder_pool)
   {
      mmal_port_pool_destroy(state->mjpeg_encoder_component->output[0], state->mjpeg_encoder_pool);
   }

   if (state->mjpeg_encoder_component)
   {
      mmal_component_destroy(state->mjpeg_encoder_component);
      state->mjpeg_encoder_component = NULL;
   }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successful.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
//static MMAL_STATUS_T create_still_encoder_component(MMALCAM_STATE *state)
//{
//   MMAL_COMPONENT_T *encoder = 0;
//   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
//   MMAL_STATUS_T status;
//   MMAL_POOL_T *pool;
//
//   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to create JPEG encoder component");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   if (!encoder->input_num || !encoder->output_num)
//   {
//      status = MMAL_ENOSYS;
//      vcos_log_error("JPEG encoder doesn't have input/output ports");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   encoder_input = encoder->input[0];
//   encoder_output = encoder->output[0];
//
//   // We want same format on input and output
//   mmal_format_copy(encoder_output->format, encoder_input->format);
//
//   // Specify out output format
//   encoder_output->format->encoding = state->still_encoding;
//
//   encoder_output->buffer_size = encoder_output->buffer_size_recommended;
//
//   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
//      encoder_output->buffer_size = encoder_output->buffer_size_min;
//
//   encoder_output->buffer_num = encoder_output->buffer_num_recommended;
//
//   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
//      encoder_output->buffer_num = encoder_output->buffer_num_min;
//
//   // Commit the port changes to the output port
//   status = mmal_port_format_commit(encoder_output);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to set format on still encoder output port");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   // Set the JPEG quality level
//   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->jpeg_quality);
//
//   if (status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to set JPEG quality");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   // Set the JPEG restart interval
//   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, state->jpeg_restart_interval);
//
//   if (state->jpeg_restart_interval && status != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to set JPEG restart interval");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   // Set up any required thumbnail
//   {
//      MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = {{MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(MMAL_PARAMETER_THUMBNAIL_CONFIG_T)}, 0, 0, 0, 0};
//
//      if ( state->thumbnailConfig.enable &&
//            state->thumbnailConfig.width > 0 && state->thumbnailConfig.height > 0 )
//      {
//         // Have a valid thumbnail defined
//         param_thumb.enable = 1;
//         param_thumb.width = state->thumbnailConfig.width;
//         param_thumb.height = state->thumbnailConfig.height;
//         param_thumb.quality = state->thumbnailConfig.quality;
//      }
//      status = mmal_port_parameter_set(encoder->control, &param_thumb.hdr);
//   }
//
//   //  Enable component
//   status = mmal_component_enable(encoder);
//
//   if (status  != MMAL_SUCCESS)
//   {
//      vcos_log_error("Unable to enable video encoder component");
//      if (encoder)
//         mmal_component_destroy(encoder);
//      return status;
//   }
//
//   /* Create pool of buffer headers for the output port to consume */
//   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
//
//   if (!pool)
//   {
//      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
//   }
//
//   state->still_encoder_pool = pool;
//   state->still_encoder_component = encoder;
//
//   if (state->common_settings.verbose)
//      fprintf(stderr, "Encoder component done\n");
//
//   return status;
//}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_still_encoder_component(MMALCAM_STATE *state)
{
   // Get rid of any port buffers first
   if (state->still_encoder_pool)
   {
      mmal_port_pool_destroy(state->still_encoder_component->output[0], state->still_encoder_pool);
   }

   if (state->still_encoder_component)
   {
      mmal_component_destroy(state->still_encoder_component);
      state->still_encoder_component = NULL;
   }
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
//static void still_encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
//{
//   int complete = 0;
//   // We pass our file handle and other stuff in via the userdata field.
//   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
//
//   //fprintf(stderr, "still callback\n");
//   if (pData)
//   {
//      int bytes_written = buffer->length;
//      if (buffer->length && pData->file_handle)
//      {
//         mmal_buffer_header_mem_lock(buffer);
//         bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
//         mmal_buffer_header_mem_unlock(buffer);
//      }
//
//      // We need to check we wrote what we wanted - it's possible we have run out of storage.
//      //if (bytes_written != buffer->length)
//      //{
//      //   vcos_log_error("Unable to write buffer to file - aborting");
//      //   complete = 1;
//      //}
//
//      // Now flag if we have completed
//      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)) {
//         fflush(pData->file_handle);
//         fprintf(stderr, "jpeg encode complete\n");
//         complete = 1;
//      }
//   }
//   else
//   {
//      vcos_log_error("Received a encoder buffer callback with no state");
//   }
//
//   // release buffer back to the pool
//   mmal_buffer_header_release(buffer);
//
//   // and send one back to the port (if still open)
//   if (port->is_enabled)
//   {
//      MMAL_STATUS_T status = MMAL_SUCCESS;
//      MMAL_BUFFER_HEADER_T *new_buffer;
//
//      new_buffer = mmal_queue_get(pData->pstate->still_encoder_pool->queue);
//
//      if (new_buffer)
//      {
//         status = mmal_port_send_buffer(port, new_buffer);
//      }
//      if (!new_buffer || status != MMAL_SUCCESS)
//         vcos_log_error("Unable to return a buffer to the still encoder port");
//
//      if (complete)
//      {
//         MMAL_PORT_T *camera_still_port = pData->pstate->camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
//         //if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, pData->pstate->bCapturing) != MMAL_SUCCESS)
//         //{
//         //   vcos_log_error("%s: Failed to start capture", __func__);
//         //}
//      }
//   }
//
//   //if (complete)
//   //   vcos_semaphore_post(&(pData->complete_semaphore));
//}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   static int64_t last_second = -1;

   // We pass our file handle and other stuff in via the userdata field.
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_written = buffer->length;
      int64_t current_time = get_microseconds64()/1000;

      vcos_assert(pData->pstate->videoOutput);
      if(pData->pstate->inlineMotionVectors) vcos_assert(pData->imv_file_handle);

      if (buffer->length)
      {
         mmal_buffer_header_mem_lock(buffer);
         if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
         {
            if(pData->pstate->inlineMotionVectors)
            {
               bytes_written = fwrite(buffer->data, 1, buffer->length, pData->imv_file_handle);
               if(pData->flush_buffers) fflush(pData->imv_file_handle);
            }
            else
            {
               //We do not want to save inlineMotionVectors...
               bytes_written = buffer->length;
            }
         }
         else
         {
            //fprintf(stderr, "write video data callback\n");
            bytes_written = pData->pstate->videoOutput->write((char*)buffer->data, buffer->length);
            //bytes_written = buffer->length;
         }

         mmal_buffer_header_mem_unlock(buffer);

         if (bytes_written != buffer->length)
         {
            vcos_log_error("video: Failed to write buffer data (%d from %d)", bytes_written, buffer->length);
            // Let's not abort for now
            //vcos_log_error("video: Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
            //pData->abort = 1;
         }
      }

      // See if the second count has changed and we need to update any annotation
      if (current_time/1000 != last_second)
      {
         update_annotation_data(pData->pstate);
         last_second = current_time/1000;
      }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->video_encoder_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void mjpeg_encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   static int64_t base_time =  -1;
   static int receiving_frame = 0;
   MMAL_BUFFER_HEADER_T *new_buffer;

   if (base_time == -1)
      base_time = get_microseconds64()/1000;

   // We pass our file handle and other stuff in via the userdata field.
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_written = buffer->length;
      if (buffer->length)
      {
         if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
         {
            // We do not want to save inlineMotionVectors...
         }
         else
         {
            //fprintf(stderr, "write mjpeg data callback\n");
            if (buffer->length && pData->file_handle)
            {
                if (receiving_frame)
                {
                    mmal_buffer_header_mem_lock(buffer);
                    bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
                    fflush(pData->file_handle);
                    fdatasync(fileno(pData->file_handle));
                    mmal_buffer_header_mem_unlock(buffer);
                }
                if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
                {
                    if (receiving_frame)
                        receiving_frame = 0;
                    int64_t current_time = get_microseconds64()/1000;
                    if (current_time - base_time > pData->mjpeg_frame_duration_ms)
                    {
                        receiving_frame = 1;
                        base_time = current_time;
                    }
                }
            }
         }
         //if (bytes_written != buffer->length)
         //{
         //   vcos_log_error("mjpeg: Failed to write buffer data (%d from %d)", bytes_written, buffer->length);
         //   // Let's not abort for now
         //   //vcos_log_error("mjpeg: Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
         //   //pData->abort = 1;
         //}
      }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);
   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;
      new_buffer = mmal_queue_get(pData->pstate->mjpeg_encoder_pool->queue);
      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);
      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the mjpeg encoder port");
   }
}
//static void mjpeg_encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
//{
//   MMAL_BUFFER_HEADER_T *new_buffer;
//   // We pass our file handle and other stuff in via the userdata field.
//   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
//
//   if (pData)
//   {
//      int bytes_written = buffer->length;
//      if (buffer->length)
//      {
//         if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
//         {
//            // We do not want to save inlineMotionVectors...
//         }
//         else
//         {
//            fprintf(stderr, "write mjpeg data callback\n");
//            if (buffer->length && pData->file_handle)
//            {
//               mmal_buffer_header_mem_lock(buffer);
//               bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
//               fflush(pData->file_handle);
//               fdatasync(fileno(pData->file_handle));
//               mmal_buffer_header_mem_unlock(buffer);
//            }
//         }
//         if (bytes_written != buffer->length)
//         {
//            vcos_log_error("mjpeg: Failed to write buffer data (%d from %d)", bytes_written, buffer->length);
//            // Let's not abort for now
//            //vcos_log_error("mjpeg: Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
//            //pData->abort = 1;
//         }
//      }
//   }
//   else
//   {
//      vcos_log_error("Received a encoder buffer callback with no state");
//   }
//
//   // release buffer back to the pool
//   mmal_buffer_header_release(buffer);
//   // and send one back to the port (if still open)
//   if (port->is_enabled)
//   {
//      MMAL_STATUS_T status;
//      new_buffer = mmal_queue_get(pData->pstate->mjpeg_encoder_pool->queue);
//      if (new_buffer)
//         status = mmal_port_send_buffer(port, new_buffer);
//      if (!new_buffer || status != MMAL_SUCCESS)
//         vcos_log_error("Unable to return a buffer to the mjpeg encoder port");
//   }
//}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
//static void mjpeg_encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
//{
//   MMAL_BUFFER_HEADER_T *new_buffer;
//   // We pass our file handle and other stuff in via the userdata field.
//   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
//   MMAL_PORT_T *isp_output_port = pData->pstate->isp_component->output[0];
//
//   fprintf(stderr, "mjpeg input port buffer callback\n");
//
//   // release buffer back to the pool
//   mmal_buffer_header_release(buffer);
//   // and send one back to the port (if still open)
//   if (isp_output_port->is_enabled)
//   {
//      MMAL_STATUS_T status;
//      new_buffer = mmal_queue_get(pData->pstate->isp_pool->queue);
//      if (new_buffer)
//         status = mmal_port_send_buffer(isp_output_port, new_buffer);
//      if (!new_buffer || status != MMAL_SUCCESS)
//         vcos_log_error("Unable to return a buffer to the isp port");
//   }
//}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
//static void isp_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
//{
//   MMAL_BUFFER_HEADER_T *new_buffer;
//   static int64_t base_time =  -1;
//   static int64_t last_second = -1;
//   static int receiving_frame = 0;
//
//   // All our segment times based on the receipt of the first encoder callback
//   if (base_time == -1)
//      base_time = get_microseconds64()/1000;
//
//   // We pass our file handle and other stuff in via the userdata field.
//   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
//
//   if (pData)
//   {
//      int64_t current_time = get_microseconds64()/1000;
//      if (buffer->length)
//      {
//         if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
//         {
//            //We do not want to save inlineMotionVectors...
//         }
//         else
//         {
//            fprintf(stderr, "isp data callback\n");
//            if (buffer->length && pData->file_handle)
//            {
//                if ((receiving_frame) || (current_time - base_time > pData->mjpeg_frame_duration_ms))
//                {
//                    fprintf(stderr, "send buffer to mjpeg\n");
//                    receiving_frame = 1;
//                    base_time = current_time;
//                    if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
//                    {
//                        // this is the end of frame
//                        receiving_frame = 0;
//                    }
//                    // send this buffer to mjpeg encoder
//                    //MMAL_PORT_T *mjpeg_encoder_input_port = pData->pstate->mjpeg_encoder_component->input[0];
//                    //if (mjpeg_encoder_input_port->is_enabled)
//                    //{
//                    //    MMAL_STATUS_T status;
//                    //    new_buffer = mmal_queue_get(pData->pstate->isp_pool->queue);
//                    //    if (new_buffer)
//                    //        status = mmal_port_send_buffer(mjpeg_encoder_input_port, new_buffer);
//                    //    if (!new_buffer || status != MMAL_SUCCESS)
//                    //        vcos_log_error("Unable to send buffer to the mjpeg encoder port");
//                    //    return;
//                    //}
//                }
//            }
//         }
//      }
//   }
//   else
//   {
//      vcos_log_error("Received a encoder buffer callback with no state");
//   }
//
//   // release buffer back to the pool
//   mmal_buffer_header_release(buffer);
//   // and send one back to the port (if still open)
//   if (port->is_enabled)
//   {
//      MMAL_STATUS_T status;
//      new_buffer = mmal_queue_get(pData->pstate->isp_pool->queue);
//      if (new_buffer)
//         status = mmal_port_send_buffer(port, new_buffer);
//      if (!new_buffer || status != MMAL_SUCCESS)
//         vcos_log_error("Unable to return a buffer to the isp port");
//   }
//}








/* ---------------------------------------------------------------------------
**  SIGINT handler
** -------------------------------------------------------------------------*/
void sighandler(int)
{
    printf("SIGINT\n");
    stop =1;
}

/* ---------------------------------------------------------------------------
**  main
** -------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
#define mmal_cleanup \
{ \
      mmal_status_to_int(status); \
      if (state.common_settings.verbose) \
         fprintf(stderr, "Closing down\n"); \
      check_disable_port(camera_still_port); \
      check_disable_port(video_encoder_output_port); \
      check_disable_port(mjpeg_encoder_output_port); \
      check_disable_port(still_encoder_output_port); \
      check_disable_port(isp_output_port); \
      if (state.video_encoder_connection) \
         mmal_connection_destroy(state.video_encoder_connection); \
      if (state.mjpeg_encoder_connection) \
         mmal_connection_destroy(state.mjpeg_encoder_connection); \
      if (state.still_encoder_connection) \
         mmal_connection_destroy(state.still_encoder_connection); \
      if (state.splitter_connection) \
         mmal_connection_destroy(state.splitter_connection); \
      if (state.isp_connection) \
         mmal_connection_destroy(state.isp_connection); \
      if (state.callback_data.file_handle && state.callback_data.file_handle != stdout) \
         fclose(state.callback_data.file_handle); \
      if (state.callback_data.imv_file_handle && state.callback_data.imv_file_handle != stdout) \
         fclose(state.callback_data.imv_file_handle); \
      if (state.video_encoder_component) \
         mmal_component_disable(state.video_encoder_component); \
      if (state.mjpeg_encoder_component) \
         mmal_component_disable(state.mjpeg_encoder_component); \
      if (state.still_encoder_component) \
         mmal_component_disable(state.still_encoder_component); \
      if (state.splitter_component) \
         mmal_component_disable(state.splitter_component); \
      if (state.isp_component) \
         mmal_component_disable(state.isp_component); \
      if (state.camera_component) \
         mmal_component_disable(state.camera_component); \
      destroy_video_encoder_component(&state); \
      destroy_mjpeg_encoder_component(&state); \
      destroy_still_encoder_component(&state); \
      destroy_splitter_component(&state);      \
      destroy_isp_component(&state);      \
      destroy_camera_component(&state);        \
      if (state.common_settings.verbose) \
         fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n"); \
}


    MMALCAM_STATE state;
    int exit_code = 0;

    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_PORT_T *camera_preview_port = NULL;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_PORT_T *video_encoder_input_port = NULL;
    MMAL_PORT_T *video_encoder_output_port = NULL;
    MMAL_PORT_T *mjpeg_encoder_input_port = NULL;
    MMAL_PORT_T *mjpeg_encoder_output_port = NULL;
    MMAL_PORT_T *still_encoder_input_port = NULL;
    MMAL_PORT_T *still_encoder_output_port = NULL;
    MMAL_PORT_T *isp_input_port = NULL;
    MMAL_PORT_T *isp_output_port = NULL;
    //MMAL_PORT_T *isp_2nd_output_port = NULL;
    //MMAL_PORT_T *splitter_input_port = NULL;
    //MMAL_PORT_T *splitter_output_port = NULL;
    //MMAL_PORT_T *splitter_preview_port = NULL;

    bcm_host_init();

    // Register our application with the logging system
    vcos_log_register("MultiStream", VCOS_LOG_CATEGORY);

    // default is memory map; IOTYPE_READWRITE use write interface?
    V4l2Access::IoType ioTypeOut = V4l2Access::IOTYPE_MMAP;

    set_app_name(argv[0]);

    // Do we have any parameters
    if (argc == 1)
    {
        display_valid_parameters(basename((char*)get_app_name()), &application_help_message);
        exit(1);
    }

    default_status(&state);

    // Parse the command line and put options in to our status structure
    if (parse_cmdline(argc, (const char**)argv, &state))
    {
        status = (MMAL_STATUS_T)-1;
        exit(1);
    }

    // Setup for sensor specific parameters, only set W/H settings if zero on entry
    get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                        &state.common_settings.width, &state.common_settings.height);

    if (state.common_settings.verbose)
    {
        print_app_details(stderr);
        dump_status(&state);
    }
    //state.mjpeg_bitrate = (int)((int64_t)state.mjpeg_bitrate * state.bitrate / state.mjpeg_framerate);
    //fprintf(stderr, "MJPEG internal bitrate %d\n", state.mjpeg_bitrate);

    check_camera_model(state.common_settings.cameraNum);

    // initialize log4cpp
    //initLogger(state.common_settings.verbose);

    // init V4L2 output interface
    V4L2DeviceParameters outparam(state.v4l2loopback_dev, V4L2_PIX_FMT_H264, state.common_settings.width, state.common_settings.height, 0, state.common_settings.verbose);
    state.videoOutput = V4l2Output::create(outparam, ioTypeOut);
    if (state.videoOutput == NULL)
    {
        vcos_log_error("%s: Cannot create V4L2 output interface for device: %s", __func__, state.v4l2loopback_dev);
        exit(2);
    }

   // OK, we have a nice set of parameters. Now set up our components
   // We have three components. Camera, Preview and encoder.

    if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
        exit_code = 2;
    }
    else if ((status = create_isp_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create isp component", __func__);
        destroy_camera_component(&state);
        exit_code = 2;
    }
    else if ((status = create_video_encoder_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create video encode component", __func__);
        destroy_isp_component(&state);
        destroy_camera_component(&state);
        exit_code = 2;
    }
    else if ((status = create_mjpeg_encoder_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create mjpeg encode component", __func__);
        destroy_isp_component(&state);
        destroy_camera_component(&state);
        destroy_video_encoder_component(&state);
        exit_code = 2;
    }
    //else if ((status = create_still_encoder_component(&state)) != MMAL_SUCCESS)
    //{
    //    vcos_log_error("%s: Failed to create still encode component", __func__);
    //    destroy_isp_component(&state);
    //    destroy_camera_component(&state);
    //    destroy_video_encoder_component(&state);
    //    destroy_mjpeg_encoder_component(&state);
    //    exit_code = 2;
    //}
    else
    {
        if (state.common_settings.verbose)
            fprintf(stderr, "Starting component connection stage\n");

        camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
        camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        //preview_input_port  = state.preview_parameters.preview_component->input[0];
        isp_input_port  = state.isp_component->input[0];
        isp_output_port = state.isp_component->output[0];
        //isp_2nd_output_port = state.isp_component->output[1];
        video_encoder_input_port  = state.video_encoder_component->input[0];
        video_encoder_output_port = state.video_encoder_component->output[0];
        mjpeg_encoder_input_port  = state.mjpeg_encoder_component->input[0];
        mjpeg_encoder_output_port = state.mjpeg_encoder_component->output[0];
        //still_encoder_input_port  = state.still_encoder_component->input[0];
        //still_encoder_output_port = state.still_encoder_component->output[0];

        // disable EXIF
        mmal_port_parameter_set_boolean(still_encoder_output_port, MMAL_PARAMETER_EXIF_DISABLE, 1);

        // Now connect the camera to the encoder
        if (state.common_settings.verbose)
            fprintf(stderr, "camera video port ---> video encoder\n");
        status = connect_ports(camera_video_port, video_encoder_input_port, &state.video_encoder_connection);
        if (status != MMAL_SUCCESS)
        {
            state.video_encoder_connection = NULL;
            vcos_log_error("%s: Failed to connect camera video port to video encoder input", __func__);
            mmal_cleanup;
        }
        // Now connect the preview to the isp
        if (state.common_settings.verbose)
            fprintf(stderr, "camera preview port ---> isp\n");
        status = connect_ports(camera_preview_port, isp_input_port, &state.isp_connection);
        if (status != MMAL_SUCCESS)
        {
            state.isp_connection = NULL;
            vcos_log_error("%s: Failed to connect camera preview port to isp input", __func__);
            mmal_cleanup;
        }
        // Now connect the isp to the mjpeg encoder
        if (state.common_settings.verbose)
            fprintf(stderr, "isp ---> mjpeg encoder\n");
        status = connect_ports(isp_output_port, mjpeg_encoder_input_port, &state.mjpeg_encoder_connection);
        if (status != MMAL_SUCCESS)
        {
            state.mjpeg_encoder_connection = NULL;
            vcos_log_error("%s: Failed to connect isp to mjpeg encoder input", __func__);
            mmal_cleanup;
        }

        //if (state.common_settings.verbose)
        //    fprintf(stderr, "camera preview port ---> still encoder\n");
        //status = connect_ports(camera_preview_port, still_encoder_input_port, &state.still_encoder_connection);
        //if (status != MMAL_SUCCESS)
        //{
        //    state.still_encoder_connection = NULL;
        //    vcos_log_error("%s: Failed to connect camera preview port to still encoder input", __func__);
        //    mmal_cleanup;
        //}
        //if (state.common_settings.verbose)
        //    fprintf(stderr, "camera still port ---> still encoder\n");
        //status = connect_ports(camera_still_port, still_encoder_input_port, &state.still_encoder_connection);
        //if (status != MMAL_SUCCESS)
        //{
        //    state.still_encoder_connection = NULL;
        //    vcos_log_error("%s: Failed to connect camera still port to still encoder input", __func__);
        //    mmal_cleanup;
        //}

        // Set up our userdata - this is passed though to the callback where we need the information.
        if (state.common_settings.verbose)
            fprintf(stderr, "Set up user data\n");
        state.callback_data.pstate = &state;
        state.callback_data.abort = 0;
        state.callback_data.mjpeg_frame_duration_ms = 1000 / state.mjpeg_framerate;
        //VCOS_STATUS_T vcos_status = vcos_semaphore_create(&state.callback_data.complete_semaphore, "RaspiStill-sem", 0);
        //vcos_assert(vcos_status == VCOS_SUCCESS);

        state.callback_data.file_handle = NULL;

        if (state.common_settings.filename)
        {
            if (state.common_settings.filename[0] == '-')
            {
                state.callback_data.file_handle = stdout;
            }
            else
            {
                state.callback_data.file_handle = fopen(state.common_settings.filename, "wb");
            }

            if (!state.callback_data.file_handle)
            {
                // Notify user, carry on but discarding encoded output buffers
                vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, state.common_settings.filename);
            }
            else
            {
                // make output file/pipe non-blocking
                //int fd = fileno(state.callback_data.file_handle);
                //int flags;
                //flags = fcntl(fd, F_GETFL, 0);
                //flags |= O_NONBLOCK;
                //fcntl(fd, F_SETFL, flags);
                // disable buffering
                //setvbuf(state.callback_data.file_handle, NULL, _IONBF, 0);
            }
        }

        state.callback_data.imv_file_handle = NULL;

        if (state.imv_filename)
        {
            if (state.imv_filename[0] == '-')
            {
                state.callback_data.imv_file_handle = stdout;
            }
            else
            {
                state.callback_data.imv_file_handle = fopen(state.imv_filename, "wb");
            }

            if (!state.callback_data.imv_file_handle)
            {
                // Notify user, carry on but discarding encoded output buffers
                fprintf(stderr, "Error opening output file: %s\nNo output file will be generated\n",state.imv_filename);
                state.inlineMotionVectors=0;
            }
        }

        //state.callback_data.pts_file_handle = NULL;
        //state.callback_data.raw_file_handle = NULL;

        // Set up our userdata - this is passed though to the callback where we need the information.
        video_encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;
        mjpeg_encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;
        //mjpeg_encoder_input_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;
        //still_encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;
        //isp_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

        // Enable the encoder output port and tell it its callback function
        if (state.common_settings.verbose)
            fprintf(stderr, "Enabling video encoder output port\n");
        status = mmal_port_enable(video_encoder_output_port, video_encoder_buffer_callback);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Failed to setup video encoder output");
            mmal_cleanup;
        }

        // Enable the isp output port and tell it its callback function
        //if (state.common_settings.verbose)
        //    fprintf(stderr, "Enabling isp output port\n");
        //status = mmal_port_enable(isp_output_port, isp_buffer_callback);
        //if (status != MMAL_SUCCESS)
        //{
        //    vcos_log_error("Failed to setup isp output");
        //    mmal_cleanup;
        //}

        // Enable the encoder output port and tell it its callback function
        if (state.common_settings.verbose)
            fprintf(stderr, "Enabling mjpeg encoder output port\n");
        status = mmal_port_enable(mjpeg_encoder_output_port, mjpeg_encoder_buffer_callback);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Failed to setup mjpeg encoder output");
            mmal_cleanup;
        }

        // Enable the encoder output port and tell it its callback function
        //if (state.common_settings.verbose)
        //    fprintf(stderr, "Enabling mjpeg encoder input port\n");
        //status = mmal_port_enable(mjpeg_encoder_input_port, mjpeg_encoder_input_buffer_callback);
        //if (status != MMAL_SUCCESS)
        //{
        //    vcos_log_error("Failed to setup mjpeg encoder input");
        //    mmal_cleanup;
        //}

        //// Enable the encoder output port and tell it its callback function
        //if (state.common_settings.verbose)
        //    fprintf(stderr, "Enabling still encoder output port\n");
        //status = mmal_port_enable(still_encoder_output_port, still_encoder_buffer_callback);
        //if (status != MMAL_SUCCESS)
        //{
        //    vcos_log_error("Failed to setup still encoder output");
        //    mmal_cleanup;
        //}

        int num;
        int q;

        // Send all the buffers to the video encoder output port
        num = mmal_queue_length(state.video_encoder_pool->queue);
        for (q=0; q<num; q++)
        {
           MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_encoder_pool->queue);
           if (!buffer)
              vcos_log_error("Unable to get a required buffer %d from pool queue", q);
           if (mmal_port_send_buffer(video_encoder_output_port, buffer)!= MMAL_SUCCESS)
              vcos_log_error("Unable to send a buffer to video encoder output port (%d)", q);
        }

        // Send all the buffers to the isp output port
        //num = mmal_queue_length(state.isp_pool->queue);
        //for (q=0; q<num; q++)
        //{
        //   MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.isp_pool->queue);
        //   if (!buffer)
        //      vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        //   if (mmal_port_send_buffer(isp_output_port, buffer)!= MMAL_SUCCESS)
        //      vcos_log_error("Unable to send a buffer to isp output port (%d)", q);
        //}

        // Send all the buffers to the mjpeg encoder output port
        num = mmal_queue_length(state.mjpeg_encoder_pool->queue);
        for (q=0; q<num; q++)
        {
           MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.mjpeg_encoder_pool->queue);
           if (!buffer)
              vcos_log_error("Unable to get a required buffer %d from pool queue", q);
           if (mmal_port_send_buffer(mjpeg_encoder_output_port, buffer)!= MMAL_SUCCESS)
              vcos_log_error("Unable to send a buffer to mjpeg encoder output port (%d)", q);
        }

        //// Send all the buffers to the still encoder output port
        //num = mmal_queue_length(state.still_encoder_pool->queue);
        //for (q=0; q<num; q++)
        //{
        //   MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.still_encoder_pool->queue);
        //   if (!buffer)
        //      vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        //   if (mmal_port_send_buffer(still_encoder_output_port, buffer)!= MMAL_SUCCESS)
        //      vcos_log_error("Unable to send a buffer to still encoder output port (%d)", q);
        //}

        // Change state
        state.bCapturing = !state.bCapturing;

        while (!state.callback_data.abort)
        {
           if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, state.bCapturing) != MMAL_SUCCESS)
           {
              // How to handle?
           }
           //if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, state.bCapturing) != MMAL_SUCCESS)
           //{
           //   vcos_log_error("%s: Failed to start capture", __func__);
           //}

           if (state.common_settings.verbose)
           {
              if (state.bCapturing)
                 fprintf(stderr, "Starting video capture\n");
              else
                 fprintf(stderr, "Pausing video capture\n");
           }

           // We never return from this. Expect a ctrl-c to exit or abort.
           while (!state.callback_data.abort)
              // Have a sleep so we don't hog the CPU.
              vcos_sleep(ABORT_INTERVAL);
        }

        if (state.common_settings.verbose)
           fprintf(stderr, "Finished capture\n");

        mmal_cleanup;
   }

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);

   delete state.videoOutput;

   return exit_code;
}
