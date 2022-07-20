#include <m5core2_audio.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#define SAMPLE_RATE     (44100)
#define DMA_BUF_LEN     (32)
#define DMA_NUM_BUF     (2)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (440.0f)
#define TWOPI           (6.28318531f)
static bool m5core2_audio_initialized = false;
static TaskHandle_t m5core2_audio_task;
static QueueHandle_t m5core2_audio_queue;
static uint16_t m5core2_audio_out_buf[DMA_BUF_LEN];
struct m5core2_audio_queue_message {
    int cmd;
    uint32_t data[2];
};
m5core2_audio_queue_message m5core2_audio_msg;

void m5core2_audio_sin(m5core2_audio_queue_message& msg) {
    struct sdata {
        float frequency;
        float volume;
    };
    // Accumulated phase
    float p = 0.0f;
    float samp = 0.0f;
    size_t bytes_written;
    void* pv;
    sdata s = *(sdata*)msg.data;
    size_t sz = sizeof(m5core2_audio_queue_message);
    
    while(!xQueueReceive(m5core2_audio_queue,&msg,0)) {
        
        for (int i=0; i < DMA_BUF_LEN; i++) {
            // Scale sine sample to 0-1 for internal DAC
            // (can't output negative voltage)
            float f;
            switch(msg.cmd) {
                case 2:
                    // sine wave  
                    f = (sinf(p) + 1.0f) * 0.5f;
                    break;
                case 3:
                    // square wave
                    // DOESN'T WORK
                    f = p>PI;
                    break;
                case 4:
                    // saw wave
                    f=(p/TWO_PI);
                    break;
            }
            
            
            // Increment and wrap phase
            p += TWOPI * s.frequency / SAMPLE_RATE;
            if (p >= TWOPI)
                p -= TWOPI;
            
            // Scale to 16-bit integer range
            samp = f* 65535.0f * s.volume;

            m5core2_audio_out_buf[i] = (uint16_t)samp ;//<< 16;
        }
        // Write with max delay. We want to push buffers as fast as we
        // can into DMA memory. If DMA memory isn't transmitted yet this
        // will yield the task until the interrupt fires when DMA buffer has 
        // space again. If we aren't keeping up with the real-time deadline,
        // audio will glitch and the task will completely consume the CPU,
        // not allowing any task switching interrupts to be processed.
        i2s_write((i2s_port_t)I2S_NUM, m5core2_audio_out_buf, sizeof(m5core2_audio_out_buf), &bytes_written, portMAX_DELAY);

        // You could put a taskYIELD() here to ensure other tasks always have a chance to run.
        vTaskDelay(1);
    }
    
    /*if(m5core2_audio_msg.cmd==2) {
        i2s_zero_dma_buffer((i2s_port_t)I2S_NUM);
    }*/
    xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
}
void m5core2_audio_task_fn(void* state) {
    m5core2_audio_queue_message msg;
    msg.cmd = 0;
    size_t sz;
    void *pv;
m5core2_audio_task_fn_restart:
    if(m5core2_audio_queue!=nullptr) {
        sz = sizeof(m5core2_audio_queue_message);
        if(xQueueReceive(m5core2_audio_queue,&msg,portMAX_DELAY)) {
            switch(msg.cmd) {
                case 0:
                    goto m5core2_audio_task_fn_restart;
                case 1:
                    i2s_zero_dma_buffer((i2s_port_t)I2S_NUM);
                    goto m5core2_audio_task_fn_restart;
                default:
                    m5core2_audio_sin(msg);
                    break;
                    
                    
            }
        }
    }
    vTaskDelay(1);
    goto m5core2_audio_task_fn_restart;
}
bool m5core2_audio::initialized() const {
    return m5core2_audio_initialized;
}
bool m5core2_audio::initialize() {
    if(!m5core2_audio_initialized) {
        i2s_config_t i2s_config;
        memset(&i2s_config,0,sizeof(i2s_config_t));
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.sample_rate = SAMPLE_RATE;
        i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
        i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
        i2s_config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
        i2s_config.dma_buf_count = DMA_NUM_BUF;
        i2s_config.dma_buf_len = DMA_BUF_LEN;
        i2s_config.use_apll = true;
        i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2;
        i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0, NULL);
        i2s_pin_config_t pins = {
            .mck_io_num = 0, // Unused
            .bck_io_num = 12,
            .ws_io_num = 0,
            .data_out_num = 2,
            .data_in_num = I2S_PIN_NO_CHANGE};
        i2s_set_pin((i2s_port_t)0, &pins);
        m5core2_audio_queue = xQueueCreate(1,sizeof(m5core2_audio_queue_message));
        // m5core2_audio_msg is global
        if(m5core2_audio_queue == nullptr) {
        
            return false;
        }
        
        // Highest possible priority for realtime audio task
        xTaskCreate(m5core2_audio_task_fn, "m5core2_audio", 1024, nullptr, configMAX_PRIORITIES - 1,&m5core2_audio_task);
        
        m5core2_audio_msg.cmd = 0;
        m5core2_audio_initialized = true;
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        
    }
    return true;
}

bool m5core2_audio::stop() {
    m5core2_audio_msg.cmd = 1;
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}
bool m5core2_audio::sinw(float frequency,float volume = 1.0) {
    if(volume==0.0) {
        return true;
    }
    struct start_data {
        float frequency;
        float volume;
    } ;
    start_data sd;
    sd.frequency = frequency;
    sd.volume = volume;
    m5core2_audio_msg.cmd = 2;
    memcpy(m5core2_audio_msg.data,&sd,sizeof(start_data));
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}
/*bool m5core2_audio::sqrw(float frequency,float volume = 1.0) {
    if(volume==0.0) {
        return true;
    }
    struct start_data {
        float frequency;
        float volume;
    } ;
    start_data sd;
    sd.frequency = frequency;
    sd.volume = volume;
    m5core2_audio_msg.cmd = 3;
    memcpy(m5core2_audio_msg.data,&sd,sizeof(start_data));
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}*/
bool m5core2_audio::saww(float frequency,float volume = 1.0) {
    if(volume==0.0) {
        return true;
    }
    struct start_data {
        float frequency;
        float volume;
    } ;
    start_data sd;
    sd.frequency = frequency;
    sd.volume = volume;
    m5core2_audio_msg.cmd = 4;
    memcpy(m5core2_audio_msg.data,&sd,sizeof(start_data));
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}