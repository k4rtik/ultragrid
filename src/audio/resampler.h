#ifdef HAVE_CONFIG_H
#include "config.h"
#include "config_win32.h"
#include "config_unix.h"
#endif

#include "audio/audio.h"

#ifdef __cplusplus
extern "C" {
#endif

struct resampler;

struct resampler *resampler_init(int dst_sample_rate);
void              resampler_done(struct resampler *);
const audio_frame2 *resampler_resample(struct resampler *, const audio_frame2 *);

#ifdef __cplusplus
}
#endif

