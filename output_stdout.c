/*
 *  Squeezelite - lightweight headless squeezebox emulator
 *
 *  (c) Adrian Smith 2012-2015, triode1@btinternet.com
 *      Ralph Irving 2015-2025, ralph_irving@hotmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// Stdout output with in-band format signaling for squeeze2diretta v2.0

#include "squeezelite.h"

#define FRAME_BLOCK MAX_SILENCE_FRAMES

static log_level loglevel;

static bool running = true;

extern struct outputstate output;
extern struct buffer *outputbuf;

#define LOCK   mutex_lock(outputbuf->mutex)
#define UNLOCK mutex_unlock(outputbuf->mutex)

extern u8_t *silencebuf;
#if DSD
extern u8_t *silencebuf_dsd;
#endif

// buffer to hold output data so we can block on writing outside of output lock, allocated on init
static u8_t *buf;
static unsigned buffill;
static int bytes_per_frame;

// ================================================================
// In-band format header for squeeze2diretta v2.0
// Written to stdout only when the audio format changes (or for
// the first track). Same-format gapless tracks flow without any
// header, ensuring uninterrupted audio. The wrapper reads this
// 16-byte header synchronously, eliminating the stderr race.
// ================================================================
struct __attribute__((packed)) sq_format_header {
	u8_t  magic[4];       // "SQFH" (0x53, 0x51, 0x46, 0x48)
	u8_t  version;        // Protocol version: 1
	u8_t  channels;       // Number of channels (2 for stereo)
	u8_t  bit_depth;      // PCM: 16/24/32, Native DSD: 1, DoP: 24
	u8_t  dsd_format;     // 0=PCM, 1=DOP, 2=DSD_U32_LE, 3=DSD_U32_BE
	u32_t sample_rate;    // Sample/frame rate in Hz (little-endian)
	u8_t  reserved[4];    // Reserved for future use, zero-filled
};

#define SQ_HEADER_VERSION 1

// Build format header from current output state (must be called under LOCK)
static void build_format_header(struct sq_format_header *hdr) {
	memset(hdr, 0, sizeof(*hdr));
	memcpy(hdr->magic, "SQFH", 4);
	hdr->version = SQ_HEADER_VERSION;
	hdr->channels = 2;
	hdr->sample_rate = output.current_sample_rate;

#if DSD
	switch (output.outfmt) {
	case PCM:
		hdr->dsd_format = 0;
		break;
	case DOP:
	case DOP_S24_LE:
	case DOP_S24_3LE:
		hdr->dsd_format = 1;
		hdr->bit_depth = 24;
		break;
	case DSD_U32_LE:
		hdr->dsd_format = 2;
		hdr->bit_depth = 1;
		break;
	case DSD_U32_BE:
		hdr->dsd_format = 3;
		hdr->bit_depth = 1;
		break;
	default:
		hdr->dsd_format = 0;
		break;
	}
	if (hdr->dsd_format == 0)
#endif
	{
		switch (output.format) {
		case S16_LE:  hdr->bit_depth = 16; break;
		case S24_3LE: hdr->bit_depth = 24; break;
		case S24_LE:  hdr->bit_depth = 24; break;
		default:      hdr->bit_depth = 32; break;
		}
	}
}

static int _stdout_write_frames(frames_t out_frames, bool silence, s32_t gainL, s32_t gainR, u8_t flags,
								s32_t cross_gain_in, s32_t cross_gain_out, s32_t **cross_ptr) {

	u8_t *obuf;

	if (!silence) {

		if (output.fade == FADE_ACTIVE && output.fade_dir == FADE_CROSS && *cross_ptr) {
			_apply_cross(outputbuf, out_frames, cross_gain_in, cross_gain_out, cross_ptr);
		}

		obuf = outputbuf->readp;

	} else {

		obuf = silencebuf;
	}

	IF_DSD(
		   if (output.outfmt != PCM) {
			   if (silence) {
				   obuf = silencebuf_dsd;
			   }
			   if (output.outfmt == DOP)
				   update_dop((u32_t *)obuf, out_frames, output.invert && !silence);
			   else if (output.invert && !silence)
				   dsd_invert((u32_t *)obuf, out_frames);
		   }
	)

	_scale_and_pack_frames(buf + buffill * bytes_per_frame, (s32_t *)(void *)obuf, out_frames, gainL, gainR, flags, output.format);

	buffill += out_frames;

	return (int)out_frames;
}

static void *output_thread(void *vargp) {
	bool first_track_seen = false;
	bool header_emitted = false;

	// Track last emitted format for gapless: skip header when format unchanged
	u32_t last_sample_rate = 0;
	u8_t  last_bit_depth = 0;
	u8_t  last_dsd_format = 0;

	LOCK;

	switch (output.format) {
	case S32_LE:
		bytes_per_frame = 4 * 2; break;
	case S24_3LE:
		bytes_per_frame = 3 * 2; break;
	case S16_LE:
		bytes_per_frame = 2 * 2; break;
	default:
		bytes_per_frame = 4 * 2; break;
		break;
	}

	UNLOCK;

	while (running) {

		LOCK;

		output.device_frames = 0;
		output.updated = gettime_ms();
		output.frames_played_dmp = output.frames_played;

		_output_frames(FRAME_BLOCK);

		// Detect new track boundary and prepare format header.
		// output.track_started is set by _output_frames() when it processes
		// a track boundary (output.c line 155). It is later cleared by
		// slimproto after reporting to the server. Both accesses are
		// protected by the outputbuf mutex, so we safely read it here.
		bool header_pending = false;
		struct sq_format_header hdr;

		if (output.track_started && !header_emitted) {
			build_format_header(&hdr);

			// Only emit header if format changed (or first track).
			// Same-format gapless tracks flow without interruption.
			if (!first_track_seen ||
				hdr.sample_rate != last_sample_rate ||
				hdr.bit_depth != last_bit_depth ||
				hdr.dsd_format != last_dsd_format) {
				header_pending = true;
				last_sample_rate = hdr.sample_rate;
				last_bit_depth = hdr.bit_depth;
				last_dsd_format = hdr.dsd_format;
			}

			first_track_seen = true;
			header_emitted = true;
		}
		if (!output.track_started) {
			header_emitted = false;
		}

		UNLOCK;

		if (!first_track_seen) {
			// Suppress pre-track silence so the first bytes on stdout
			// are always a format header (wrapper expects "SQFH" magic)
			buffill = 0;
			usleep(10000);
			continue;
		}

		// Write any remaining audio from the previous track
		if (buffill) {
			fwrite(buf, bytes_per_frame, buffill, stdout);
			fflush(stdout);
			buffill = 0;
		} else if (!header_pending) {
			// No audio data and no header to emit — avoid busy-wait
			usleep(10000);
		}

		// Write format header for the new track (after old-track audio)
		if (header_pending) {
			fwrite(&hdr, sizeof(hdr), 1, stdout);
			fflush(stdout);
		}
	}

	return 0;
}

static thread_type thread;

void output_init_stdout(log_level level, unsigned output_buf_size, char *params, unsigned rates[], unsigned rate_delay) {
	loglevel = level;

	LOG_INFO("init output stdout");

	buf = malloc(FRAME_BLOCK * BYTES_PER_FRAME);
	if (!buf) {
		LOG_ERROR("unable to malloc buf");
		return;
	}
	buffill = 0;

	memset(&output, 0, sizeof(output));

	output.format = S32_LE;
	output.start_frames = FRAME_BLOCK * 2;
	output.write_cb = &_stdout_write_frames;
	output.rate_delay = rate_delay;

	if (params) {
		if (!strcmp(params, "32"))	output.format = S32_LE;
		if (!strcmp(params, "24")) output.format = S24_3LE;
		if (!strcmp(params, "16")) output.format = S16_LE;
	}

	// ensure output rate is specified to avoid test open
	if (!rates[0]) {
		rates[0] = 44100;
	}

	output_init_common(level, "-", output_buf_size, rates, 0);

#if LINUX || OSX || FREEBSD
	pthread_attr_t attr;
	pthread_attr_init(&attr);
#ifdef PTHREAD_STACK_MIN
	pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + OUTPUT_THREAD_STACK_SIZE);
#endif
	pthread_create(&thread, &attr, output_thread, NULL);
	pthread_attr_destroy(&attr);
#endif
#if WIN
	thread = CreateThread(NULL, OUTPUT_THREAD_STACK_SIZE, (LPTHREAD_START_ROUTINE)&output_thread, NULL, 0, NULL);
#endif
}

void output_close_stdout(void) {
	LOG_INFO("close output");

	LOCK;
	running = false;
	UNLOCK;

	free(buf);

	output_close_common();
}
