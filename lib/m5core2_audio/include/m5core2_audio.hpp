#pragma once
#include <Arduino.h>
typedef void (*m5core2_audio_record_callback)(const uint16_t *data, size_t size, void* state);
class m5core2_audio {
public:
    inline bool initialized() const;
    bool initialize();
    // generate a sine based tone at the specified hz, and a volume multiplier between 0 and 1
    bool sinw(float frequency,float volume);
    // DOESN'T WORK?! generate a square based tone at the specified hz, and a volume multiplier between 0 and 1
    //bool sqrw(float frequency,float volume);
    // generate a sawtooth based tone at the specified hz, and a volume multiplier between 0 and 1
    bool saww(float frequency,float volume);
    // generate a triangle based tone at the specified hz, and a volume multiplier between 0 and 1
    bool triw(float frequency,float volume);
    
    // stops all sound from playing
    bool stop();

    bool start_recording(m5core2_audio_record_callback callback,void* state);
    bool stop_recording();
    // generate a square based tone at the specified hz, and a volume multiplier between 0 and 1
    //bool sqrw(float frequency,float volume);
    
};