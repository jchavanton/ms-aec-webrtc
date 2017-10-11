
#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/msticker.h"
#include "ortp/b64.h"
#include <aec_core.h>
#include <echo_cancellation.h>
#include "mediastreamer2/flowcontrol.h"

#ifdef WIN32
#include <malloc.h> /* for alloca */
#endif


//#define EC_DUMP 1
#ifdef ANDROID
#define EC_DUMP_PREFIX "/sdcard"
#else
#define EC_DUMP_PREFIX "/dynamic/tests"
#endif

typedef struct stream_sync_info {
	int last_tick;
	int no_farend;
	int to_much_farend;
	int farend_ms;
	int farend_buff_ms;
	int nearend_ms;
	int no_underrun;
} stream_sync_info_t;

static const int framesize = 80;
static const int flow_control_interval_ms = 5000;
static const int farend_buff_threshold_ms = 40;
#define MAX_FRAMESIZE_PER_TICK 2 /* the maximum amount of framesize we will process in one tick, to recover when we accumulate samples when the ticker is late or the sound card is generating jitter */
#define GET_DELAY_METRICS 1
#define BUFF_LEN 160
typedef struct WebRTCAECState {
	void *aecInst;
	MSBufferizer farend_samples;
	MSBufferizer nearend_samples;
	int channel_count;
	int framesize;
	int samplerate;
	int bytes_per_ms;
	int delay_ms;
	bool_t echo_started;
	int echo_found;
	MSAudioFlowController afc;
	uint64_t flow_control_time;
	int aec_process_count;
	stream_sync_info_t stream_sync_info;
	char *state_str;
#ifdef EC_DUMP
	FILE *echofile;
	FILE *reffile;
	FILE *cleanfile;
#endif
	bool_t bypass_mode;
	int16_t * farend_samples_int;
	int16_t * nearend_samples_int;
	float farend_samples_flt[BUFF_LEN];
	float nearend_samples_flt[BUFF_LEN];
	float out_samples[BUFF_LEN];
	const float * nearend_samples_ptr;
	float * out_samples_ptr;
} WebRTCAECState;


static void webrtc_aec_init(MSFilter *f) {
	WebRTCAECState *s = (WebRTCAECState *) ms_new(WebRTCAECState, 1);
	s->samplerate = 8000;
	s->bytes_per_ms = s->samplerate/1000*2;
	ms_bufferizer_init(&s->nearend_samples);
	ms_bufferizer_init(&s->farend_samples);
	s->delay_ms = 0;
	s->echo_found = 0;
	s->aec_process_count = 0;
	s->aecInst = NULL;
	s->state_str = NULL;
	s->channel_count = 1;
	s->framesize = framesize;
	s->bypass_mode = FALSE;
	s->echo_started = FALSE;
	memset(&s->stream_sync_info,0,sizeof(stream_sync_info_t));
#ifdef EC_DUMP
	{
		char *fname = ms_strdup_printf("%s/mswebrtcaec-%p-echo.raw", EC_DUMP_PREFIX, f);
		s->echofile = fopen(fname, "w");
		ms_free(fname);
		fname = ms_strdup_printf("%s/mswebrtcaec-%p-ref.raw", EC_DUMP_PREFIX, f);
		s->reffile = fopen(fname, "w");
		ms_free(fname);
		fname = ms_strdup_printf("%s/mswebrtcaec-%p-clean.raw", EC_DUMP_PREFIX, f);
		s->cleanfile = fopen(fname, "w");
		ms_free(fname);
	}
#endif
	f->data = s;
}

static void webrtc_aec_uninit(MSFilter *f)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
#ifdef EC_DUMP
	if (s->echofile)
		fclose(s->echofile);
	if (s->reffile)
		fclose(s->reffile);
#endif
	if (s->state_str) ms_free(s->state_str);
	ms_free(s);
}

static void webrtc_aec_preprocess(MSFilter *f)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	AecConfig config;
	int error_code;
	s->bytes_per_ms = s->samplerate/1000*2;

	s->framesize=(framesize*s->samplerate)/8000;
	ms_message("Initializing WebRTC delay agnostic echo canceler with framesize[%i] sample_rate[%d] echo_delay_ms[%i]", s->framesize, s->samplerate, s->delay_ms);

	if ((s->aecInst = WebRtcAec_Create()) == NULL) {
		s->bypass_mode = TRUE;
		ms_error("WebRtcAec_Create(): error, entering bypass mode");
		return;
	}
	if ((error_code = WebRtcAec_Init(s->aecInst, s->samplerate, s->samplerate)) < 0) { // params: (1) AEC instance, (2) sampling frequency of data, (3) soundcard sampling frequency
		if (error_code == AEC_BAD_PARAMETER_ERROR) {
			ms_error("WebRtcAec_Init(): WebRTC echo canceller does not support %d samplerate", s->samplerate);
		}
		s->bypass_mode = TRUE;
		ms_error("Entering bypass mode");
		return;
	}
	config.nlpMode = kAecNlpAggressive; // kAecNlpModerate | kAecNlpConservative | kAecNlpAggressive
	config.skewMode = kAecFalse;
#if GET_DELAY_METRICS
	config.metricsMode = kAecTrue;
	config.delay_logging = kAecTrue;
#else
	config.metricsMode = kAecFalse;
	config.delay_logging = kAecFalse;
#endif

	if (WebRtcAec_set_config(s->aecInst, config)!=0){
		ms_error("WebRtcAec_set_config(): failed.");
	}

	WebRtcAec_enable_delay_agnostic(WebRtcAec_aec_core(s->aecInst), 1);
	ms_audio_flow_controller_init(&s->afc);
	s->flow_control_time = f->ticker->time;
}

/*	inputs[0]= reference signal from far end (sent to soundcard)
 *	inputs[1]= near speech & echo signal (read from soundcard)
 *	outputs[0]=  is a copy of inputs[0] to be sent to soundcard
 *	outputs[1]=  near end speech, echo removed - towards far end
*/
static void webrtc_aec_process(MSFilter *f)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	int nbytes = s->framesize * 2;
	int status=0;
	uint8_t *farend_samples, *nearend_samples;
	int16_t *processed_samples;
	mblk_t *tmp_mblk;
	int index=0;

	if (s->bypass_mode) { /* Not using AEC error ? */
		while ((tmp_mblk = ms_queue_get(f->inputs[0])) != NULL) {
			ms_queue_put(f->outputs[0], tmp_mblk);
		}
		while ((tmp_mblk = ms_queue_get(f->inputs[1])) != NULL) {
			ms_queue_put(f->outputs[1], tmp_mblk);
		}
		return;
	}

	while ( !ms_queue_empty(f->inputs[1]) ) {
		ms_bufferizer_put_from_queue(&s->nearend_samples, f->inputs[1]); /* nearend signal, with echo */
		if (!s->echo_started) s->echo_started=TRUE;
	}

	if (f->inputs[0] != NULL) { /* reading farend signal */
		if(!s->echo_started){ // try to sync with the local sound card "reading"
			ms_warning("Getting reference signal but no echo to synchronize on.");
			ms_queue_flush(f->inputs[0]);
		} else {
			while ((tmp_mblk = ms_queue_get(f->inputs[0])) != NULL) {
				if(s->echo_started) { /* start to process farend signal */
					tmp_mblk=ms_audio_flow_controller_process(&s->afc,tmp_mblk);
					if (tmp_mblk) { /* in case the flow controller got rid of the packet */
						ms_bufferizer_put(&s->farend_samples, tmp_mblk);
					}
				}
			}
		}
	}

	farend_samples = (uint8_t *) alloca(nbytes);
	nearend_samples = (uint8_t *) alloca(nbytes);

	int i = 0;
	while (i < MAX_FRAMESIZE_PER_TICK && ms_bufferizer_get_avail(&s->nearend_samples) >= nbytes) {
		i++;
		s->nearend_samples_int = NULL;
		s->farend_samples_int = NULL;

		/* 1- preparing farend samples */
		if (ms_bufferizer_get_avail(&s->farend_samples) <= nbytes) { /* no enough far end signal using zeroes */
			s->stream_sync_info.no_farend += nbytes;
			memset(farend_samples, 0, nbytes);
		} else {
			ms_bufferizer_read(&s->farend_samples, farend_samples, nbytes);
		}
		s->farend_samples_int = (int16_t*) farend_samples;
		for (index=0 ; index < s->framesize; index++) {
			s->farend_samples_flt[index] = (float) s->farend_samples_int[index];
		}

	{	/* output the samples to be played by the sound card, same ones we are giving the the AEC */
		mblk_t *farend_out = allocb(nbytes, 0);
		memcpy(farend_out->b_wptr, farend_samples, nbytes);
		farend_out->b_wptr += nbytes;
		ms_queue_put(f->outputs[0], farend_out);
	}

		/* 2- preparing nearend samples */
		ms_bufferizer_read(&s->nearend_samples, nearend_samples, nbytes);
		s->nearend_samples_int = (int16_t*) nearend_samples;
		for (index=0 ; index < s->framesize; index++) {
			s->nearend_samples_flt[index] = (float) s->nearend_samples_int[index];
		}

#ifdef EC_DUMP
			if (s->reffile)
				fwrite(farend_samples, nbytes, 1, s->reffile);
			if (s->echofile)
				fwrite(nearend_samples, nbytes, 1, s->echofile);
#endif

		if (s->farend_samples_int) {
			if (WebRtcAec_BufferFarend(s->aecInst, s->farend_samples_flt, s->framesize) !=0 ){
				ms_error("webrtc_aec_process: WebRtcAec_BufferFarend() failed.");
			}
			s->stream_sync_info.farend_ms += s->framesize *1000/s->samplerate;
		}

		mblk_t *oecho = allocb(nbytes, 0);
		s->nearend_samples_ptr = &s->nearend_samples_flt[0];
		s->out_samples_ptr = &s->out_samples[0];
	/*
	    1- void* aecInst,
	    2- const float* const* nearend,
	    3- size_t num_bands,
	    4- float* const* out,
	    5- size_t nrOfSamples,
	    6- int16_t msInSndCardBuf,
	    7- int32_t skew
	                                             (1)                       (2)               (3)                  (4)           (5)          (6) (7)    */
			if (WebRtcAec_Process(s->aecInst, &s->nearend_samples_ptr, s->channel_count, &s->out_samples_ptr, s->framesize, s->delay_ms,  0) !=0 ){
				ms_error("webrtc_aec_process: WebRtcAec_Process() failed.");
			}
			s->stream_sync_info.nearend_ms += s->framesize *1000/s->samplerate;

			processed_samples = (int16_t*)oecho->b_wptr;
			for (index = 0; index < s->framesize; index++) {
				*processed_samples = (int16_t)s->out_samples_ptr[index];
				processed_samples++;
			}
			oecho->b_wptr = (uint8_t*)processed_samples;
	
#ifdef EC_DUMP
			if (s->cleanfile)
				fwrite(oecho->b_wptr, nbytes, 1, s->cleanfile);
#endif
			ms_queue_put(f->outputs[1], oecho);

			if (WebRtcAec_get_echo_status(s->aecInst, &status)) {
				ms_error("webrtc_aec_process: WebRtcAec_get_echo_status() failed.");
			} else if (status == 1) {
				s->echo_found++;
			}
			s->aec_process_count++;
	}

		if ( (int)f->ticker->time - s->stream_sync_info.last_tick >= flow_control_interval_ms) {
			int median, std;
			float fraction_poor_delays;
			s->stream_sync_info.last_tick = (int)f->ticker->time;
			status = WebRtcAec_GetDelayMetrics(s->aecInst, &median, &std, &fraction_poor_delays);

			if (s->stream_sync_info.no_farend == 0) {
				s->stream_sync_info.no_underrun++;
			} else {
				s->stream_sync_info.no_underrun=0;
			}

			if (status != 0) {
				ms_error("webrtc_aec_process: WebRtcAec_GetDelayMetrics: error");
			}
			s->stream_sync_info.farend_buff_ms = (int)ms_bufferizer_get_avail(&s->farend_samples)/s->bytes_per_ms;
			ms_message("webrtc_aec_process: echo[%d/%d]median[%d]std[%d]fpd[%.4f] farend:buff[%dms]no[%dms]too_much[%dms]out[%dms]"
                                   " nearend:buff[%dms]out[%dms] no_underrun[%d]",
					s->echo_found, s->aec_process_count, median, std, fraction_poor_delays,
					s->stream_sync_info.farend_buff_ms,
					s->stream_sync_info.no_farend/s->bytes_per_ms,
					s->stream_sync_info.to_much_farend/s->bytes_per_ms,
					s->stream_sync_info.farend_ms,
					(int)ms_bufferizer_get_avail(&s->nearend_samples)/s->bytes_per_ms,
					s->stream_sync_info.nearend_ms, s->stream_sync_info.no_underrun
			);
			s->echo_found = 0;
			s->aec_process_count = 0;
			s->stream_sync_info.no_farend = 0;
			s->stream_sync_info.to_much_farend = 0;
		}
		if(s->stream_sync_info.farend_buff_ms > farend_buff_threshold_ms && (uint32_t) (f->ticker->time - s->flow_control_time) >= flow_control_interval_ms) {
			int purge = ((int)ms_bufferizer_get_avail(&s->farend_samples) / 10 * s->stream_sync_info.no_underrun);
			if (((int)ms_bufferizer_get_avail(&s->farend_samples)) - purge < s->framesize) { // always keep at least 10ms
				purge = (int)ms_bufferizer_get_avail(&s->farend_samples) - s->framesize;
			}
			ms_warning("webrtc_aec_process: we are accumulating too much reference signal, need to throw out[%i] samples extra_buffer[%dms]",
                                    purge, s->stream_sync_info.farend_buff_ms);
			ms_audio_flow_controller_set_target(&s->afc, purge, (flow_control_interval_ms * s->samplerate) / 1000);
			s->flow_control_time = f->ticker->time;
		}

}

static void webrtc_aec_postprocess(MSFilter *f)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	ms_bufferizer_flush(&s->nearend_samples);
	ms_bufferizer_flush(&s->farend_samples);
	if (s->aecInst != NULL) {
		WebRtcAec_Free(s->aecInst);
		s->aecInst = NULL;
	}
}

static int webrtc_aec_set_sr(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	int requested_sr = *(int *) arg;
	int sr = requested_sr;

	if (requested_sr != 8000) {
		sr = 8000;
		ms_message("Webrtc aec-da does not support sampling rate %i, using %i instead", requested_sr, sr);
	}
	s->samplerate = sr;
	return 0;
}

static int webrtc_aec_get_sr(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	*(int *) arg=s->samplerate;
	return 0;
}

static int webrtc_aec_set_framesize(MSFilter *f, void *arg)
{
	/* Do nothing because the WebRTC echo canceller only accept specific values: 80 and 160. We use 80 at 8khz, and 160 at 16khz */
	return 0;
}

static int webrtc_aec_set_delay(MSFilter *f, void *arg)
{
	/* Used for initialization since delay agnostic is taking 5-10 seconds */
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	s->delay_ms = *(int *) arg;
	return 0;
}

static int webrtc_aec_set_tail_length(MSFilter *f, void *arg)
{
	/* Do nothing because this is not needed by the WebRTC echo canceller. */
	return 0;
}
static int webrtc_aec_set_bypass_mode(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	s->bypass_mode = *(bool_t *) arg;
	ms_message("set EC bypass mode to [%i]", s->bypass_mode);
	return 0;
}
static int webrtc_aec_get_bypass_mode(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	*(bool_t *) arg = s->bypass_mode;
	return 0;
}

static int webrtc_aec_set_state(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	s->state_str = ms_strdup((const char *) arg);
	return 0;
}

static int webrtc_aec_get_state(MSFilter *f, void *arg)
{
	WebRTCAECState *s = (WebRTCAECState *) f->data;
	*(char **) arg = s->state_str;
	return 0;
}

static MSFilterMethod webrtc_aec_methods[] = {
	{	MS_FILTER_SET_SAMPLE_RATE		,	webrtc_aec_set_sr 		},
	{	MS_FILTER_GET_SAMPLE_RATE		,	webrtc_aec_get_sr 		},
	{	MS_ECHO_CANCELLER_SET_TAIL_LENGTH	,	webrtc_aec_set_tail_length	},
	{	MS_ECHO_CANCELLER_SET_DELAY		,	webrtc_aec_set_delay		},
	{	MS_ECHO_CANCELLER_SET_FRAMESIZE		,	webrtc_aec_set_framesize	},
	{	MS_ECHO_CANCELLER_SET_BYPASS_MODE	,	webrtc_aec_set_bypass_mode	},
	{	MS_ECHO_CANCELLER_GET_BYPASS_MODE	,	webrtc_aec_get_bypass_mode	},
	{	MS_ECHO_CANCELLER_GET_STATE_STRING	,	webrtc_aec_get_state		},
	{	MS_ECHO_CANCELLER_SET_STATE_STRING	,	webrtc_aec_set_state		}
};



#define MS_WEBRTC_AEC_DA_NAME        "MSWebRTCDAAEC"
#define MS_WEBRTC_AEC_DA_DESCRIPTION "Delay agnostic echo canceller using WebRTC library."
#define MS_WEBRTC_AEC_DA_CATEGORY    MS_FILTER_OTHER
#define MS_WEBRTC_AEC_DA_ENC_FMT     NULL
#define MS_WEBRTC_AEC_DA_NINPUTS     2
#define MS_WEBRTC_AEC_DA_NOUTPUTS    2
#define MS_WEBRTC_AEC_DA_FLAGS       0

#ifdef _MSC_VER

MSFilterDesc ms_webrtc_aec_da_desc = {
	MS_FILTER_PLUGIN_ID,
	MS_WEBRTC_AEC_DA_NAME,
	MS_WEBRTC_AEC_DA_DESCRIPTION,
	MS_WEBRTC_AEC_DA_CATEGORY,
	MS_WEBRTC_AEC_DA_ENC_FMT,
	MS_WEBRTC_AEC_DA_NINPUTS,
	MS_WEBRTC_AEC_DA_NOUTPUTS,
	webrtc_aec_init,
	webrtc_aec_preprocess,
	webrtc_aec_process,
	webrtc_aec_postprocess,
	webrtc_aec_uninit,
	webrtc_aec_methods,
	MS_WEBRTC_AEC_DA_FLAGS
};

#else

MSFilterDesc ms_webrtc_aec_da_desc = {
	.id = MS_FILTER_PLUGIN_ID,
	.name = MS_WEBRTC_AEC_DA_NAME,
	.text = MS_WEBRTC_AEC_DA_DESCRIPTION,
	.category = MS_WEBRTC_AEC_DA_CATEGORY,
	.enc_fmt = MS_WEBRTC_AEC_DA_ENC_FMT,
	.ninputs = MS_WEBRTC_AEC_DA_NINPUTS,
	.noutputs = MS_WEBRTC_AEC_DA_NOUTPUTS,
	.init = webrtc_aec_init,
	.preprocess = webrtc_aec_preprocess,
	.process = webrtc_aec_process,
	.postprocess = webrtc_aec_postprocess,
	.uninit = webrtc_aec_uninit,
	.methods = webrtc_aec_methods,
	.flags = MS_WEBRTC_AEC_DA_FLAGS
};

#endif

MS_FILTER_DESC_EXPORT(ms_webrtc_aec_da_desc)

