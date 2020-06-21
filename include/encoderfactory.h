/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** encoder.h
** 
** -------------------------------------------------------------------------*/

#pragma once

#include "encoder.h"

#ifdef HAVE_X264   
#include "x264encoder.h"
#endif
#ifdef HAVE_X265
#include "x265encoder.h"
#endif
#ifdef HAVE_VPX   
#include "vpxencoder.h"
#endif
#ifdef HAVE_JPEG  
#include "jpegencoder.h"
#endif

class EncoderFactory {
    public:
    static Encoder* Create(int format, int width, int height, const std::map<std::string,std::string> & opt, int verbose) {
        Encoder* encoder = NULL;
        switch (format) {
#ifdef HAVE_X265         
            case V4L2_PIX_FMT_HEVC: encoder = new X265Encoder(format, width, height, opt, verbose); break;
#endif            
#ifdef HAVE_X264           
            case V4L2_PIX_FMT_H264: encoder = new X264Encoder(format, width, height, opt, verbose); break;
#endif            
#ifdef HAVE_VPX            
            case V4L2_PIX_FMT_VP8: encoder = new VpxEncoder(format, width, height, opt, verbose); break;
            case V4L2_PIX_FMT_VP9: encoder = new VpxEncoder(format, width, height, opt, verbose); break;
#endif            
#ifdef HAVE_JPEG            
            case V4L2_PIX_FMT_JPEG: encoder = new JpegEncoder(format, width, height, opt, verbose); break;
#endif            
        }
        return encoder;
    }

    static std::list<int> SupportedFormat() {
        std::list<int> formatList;
#ifdef HAVE_X265         
        formatList.push_back(V4L2_PIX_FMT_HEVC);
#endif            
#ifdef HAVE_X264           
        formatList.push_back(V4L2_PIX_FMT_H264);
#endif            
#ifdef HAVE_VPX        
        formatList.push_back(V4L2_PIX_FMT_VP8);    
        formatList.push_back(V4L2_PIX_FMT_VP9);  
#endif            
#ifdef HAVE_JPEG 
        formatList.push_back(V4L2_PIX_FMT_JPEG);            
#endif              
        return formatList;      
    }

    EncoderFactory & get() {
        static EncoderFactory instance;
        return instance;
    }
};