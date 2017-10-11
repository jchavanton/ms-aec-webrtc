#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/mscodecutils.h"

#ifdef BUILD_AEC
extern MSFilterDesc ms_webrtc_aec_desc;
#endif
#ifdef BUILD_AEC_DA
extern MSFilterDesc ms_webrtc_aec_da_desc;
#endif

#ifndef VERSION
#define VERSION "debug"
#endif

#ifdef _MSC_VER
#define MS_PLUGIN_DECLARE(type) __declspec(dllexport) type
#else
#define MS_PLUGIN_DECLARE(type) type
#endif

MS_PLUGIN_DECLARE ( void ) libmsaecwebrtc_init(MSFactory* factory) {
	WebRtcSpl_Init();
#ifdef WEBRTC_HAS_NEON
	ms_message("libmsaecwebrtc " VERSION " WEBRTC_HAS_NEON");
#endif
#ifdef BUILD_AEC
	ms_factory_register_filter(factory, &ms_webrtc_aec_desc);
	ms_message("libmsaecwebrtc " VERSION " AECM mobile plugin loaded");
#endif
#ifdef BUILD_AEC_DA
	ms_factory_register_filter(factory, &ms_webrtc_aec_da_desc);
	ms_message("libmsaecwebrtc " VERSION " AEC delay agnostic plugin loaded");
#endif
}
