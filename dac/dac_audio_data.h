#ifndef __DAC_AUDIO_DATA__
#define __DAC_AUDIO_DATA__

#include <stdint.h>

/* 音频参数 */
#define DAC_AUDIO_DATA_SAMPLE_RATE    4000
#define DAC_AUDIO_DATA_NUM_CHANNELS   1
#define DAC_AUDIO_DATA_NUM_SAMPLES    456278

extern const uint8_t dac_audio_data_data[DAC_AUDIO_DATA_NUM_SAMPLES];


#endif