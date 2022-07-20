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
#define SAMPLE_RATE_INPUT     (44100)
#define DMA_BUF_LEN     (32)
#define DMA_BUF_LEN_INPUT     (512)
#define DMA_NUM_BUF     (2)
#define I2S_NUM         (0)
#define TWOPI           (6.28318531f)
static int m5core2_audio_init_mode = -1;
static TaskHandle_t m5core2_audio_task;
static QueueHandle_t m5core2_audio_queue;
static m5core2_audio_record_callback m5core2_audio_record_fn;
static void* m5core2_audio_record_state;
static uint16_t m5core2_audio_out_buf[DMA_BUF_LEN];
static uint16_t m5core2_audio_in_buf[DMA_BUF_LEN_INPUT];
struct m5core2_audio_queue_message {
    int cmd;
    uint32_t data[2];
};
m5core2_audio_queue_message m5core2_audio_msg;

bool m5core2_audio_shape(unsigned shape,float frequency, float volume) {
    struct start_data {
        float frequency;
        float volume;
    };
    if(shape>3) {
        shape = 0;
    }
    start_data sd;
    sd.frequency = frequency;
    sd.volume = volume;
    m5core2_audio_msg.cmd = shape+10;
    memcpy(m5core2_audio_msg.data,&sd,sizeof(start_data));
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}
bool m5core2_audio_i2s_initialize(int mode) {
    if(mode==m5core2_audio_init_mode) {
        return true;
    }
    
    if(m5core2_audio_init_mode!=0||mode==0) {
        Serial.println("driver uninstalled");
        i2s_driver_uninstall((i2s_port_t)I2S_NUM);
    }
    i2s_config_t i2s_config;
    i2s_pin_config_t pins;
    switch(mode) {
        case 0:
            break;
        case 1: // output     
            memset(&i2s_config,0,sizeof(i2s_config_t));
            i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
            i2s_config.sample_rate = SAMPLE_RATE;
            i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
            i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
            i2s_config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
            i2s_config.dma_buf_count = DMA_NUM_BUF;
            i2s_config.dma_buf_len = DMA_BUF_LEN;
            i2s_config.use_apll = true;
            i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2;
            i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0, NULL);
            pins = {
                .mck_io_num = 0, // Unused
                .bck_io_num = 12,
                .ws_io_num = 0,
                .data_out_num = 2,
                .data_in_num = I2S_PIN_NO_CHANGE};
            i2s_set_pin((i2s_port_t)I2S_NUM, &pins);

            break;
        case 2: // input
            memset(&i2s_config,0,sizeof(i2s_config_t));
            i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
            i2s_config.sample_rate = 64000;
            i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
            i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
            i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
            i2s_config.dma_buf_count = DMA_NUM_BUF;
            i2s_config.dma_buf_len = DMA_BUF_LEN_INPUT;
            i2s_config.use_apll = false;
            i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
            i2s_driver_install((i2s_port_t)I2S_NUM, &i2s_config, 0,  NULL);
            pins = {
                .mck_io_num = 0, // Unused
                .bck_io_num = 12,
                .ws_io_num = 0,
                .data_out_num = I2S_PIN_NO_CHANGE,
                .data_in_num = 34};
            i2s_set_pin((i2s_port_t)I2S_NUM, &pins);
            break;
    }
    m5core2_audio_init_mode=mode;
    return true;
}
void m5core2_audio_sin(m5core2_audio_queue_message& msg) {
    struct sdata {
        float frequency;
        float volume;
    };
    // Accumulated phase
    float p = 0.0f;
    float p2 = 0.0f;
    
    float samp = 0.0f;
    size_t bytes_written;
    void* pv;
    sdata s = *(sdata*)msg.data;
    const float pdc = TWOPI*s.frequency/SAMPLE_RATE;
    float pd = pdc;
    size_t sz = sizeof(m5core2_audio_queue_message);
    
    while(!xQueueReceive(m5core2_audio_queue,&msg,0)) {
        
        for (int i=0; i < DMA_BUF_LEN; i++) {
            // offset values by 1 and then scale to half since they can't be negative
            float f;
            switch(msg.cmd) {
                case 10:
                    // sine wave  
                    f = (sinf(p) + 1.0f) * 0.5f;
                    break;
                case 11:
                    // square wave
                    // DOESN'T WORK
                    f = p>PI;
                    break;
                case 12:
                    // saw wave
                    f=(p2>=0.0);
                    break;
                case 13:
                    // triangle wave
                    f=((p2/PI)+1.0)*.5;
                    break;
            }
            
            
            // Increment and wrap phase
            p += pdc;
            if (p >= TWOPI) {
                p -= TWOPI;
            }
            if(p2+pd<=-PI || p2+pd>=PI) {
                pd=-pd;
            }
            p2+=pd;
            // Scale to 16-bit integer range
            samp = f* 65535.0f * s.volume;
            m5core2_audio_out_buf[i] = (uint16_t)samp ;//<< 16;
        }
        // time critical
        i2s_write((i2s_port_t)I2S_NUM, m5core2_audio_out_buf, sizeof(m5core2_audio_out_buf), &bytes_written, portMAX_DELAY);

        // You could put a taskYIELD() here to ensure other tasks always have a chance to run.
        vTaskDelay(0);
    }

    xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
}
void m5core2_audio_rec() {
    m5core2_audio_queue_message msg;
    size_t read;
    while(!xQueueReceive(m5core2_audio_queue,&msg,0)) {
        
        // time critical
        i2s_read((i2s_port_t)I2S_NUM,m5core2_audio_in_buf,sizeof(m5core2_audio_in_buf),&read,portMAX_DELAY);
        m5core2_audio_record_fn(m5core2_audio_in_buf,read,m5core2_audio_record_state);
        // You could put a taskYIELD() here to ensure other tasks always have a chance to run.
        vTaskDelay(0);
    }

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
                case 3:
                    m5core2_audio_rec();
                default:
                    m5core2_audio_sin(msg);
                    break;
                    
                    
            }
        }
    }
    vTaskDelay(0);
    goto m5core2_audio_task_fn_restart;
}
bool m5core2_audio::initialized() const {
    return m5core2_audio_init_mode;
}
bool m5core2_audio::initialize() {
    if(m5core2_audio_init_mode==-1) {
        m5core2_audio_queue = xQueueCreate(1,sizeof(m5core2_audio_queue_message));
        // m5core2_audio_msg is global
        if(m5core2_audio_queue == nullptr) {
        
            return false;
        }
        
        // Highest possible priority for realtime audio task
        xTaskCreate(m5core2_audio_task_fn, "m5core2_audio", 1024*4, nullptr, configMAX_PRIORITIES - 1,&m5core2_audio_task);
        
        m5core2_audio_msg.cmd = 0;
        m5core2_audio_init_mode = 0;
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
    if(m5core2_audio_init_mode!=1) {
        m5core2_audio_i2s_initialize(1);
    }
    return m5core2_audio_shape(0,frequency,volume);
}
bool m5core2_audio::triw(float frequency,float volume = 1.0) {
    if(m5core2_audio_init_mode!=1) {
        m5core2_audio_i2s_initialize(1);
    }
    return m5core2_audio_shape(3,frequency,volume);
}

/*bool m5core2_audio::sqrw(float frequency,float volume = 1.0) {
    return m5core2_audio_shape(1,frequency,volume);
}*/
bool m5core2_audio::saww(float frequency,float volume = 1.0) {
    if(m5core2_audio_init_mode!=1) {
        m5core2_audio_i2s_initialize(1);
    }
    return m5core2_audio_shape(2,frequency,volume);
}
bool m5core2_audio::start_recording(m5core2_audio_record_callback callback,void* state=nullptr) {
    if(callback==nullptr || m5core2_audio_queue==nullptr) {
        return false;
    } 
    if(m5core2_audio_init_mode==2) {
        if(!stop_recording()) {
            return false;
        }
    } else {
        m5core2_audio_i2s_initialize(2);
    }
    m5core2_audio_record_fn = callback;
    m5core2_audio_record_state = state;
    m5core2_audio_msg.cmd = 3;
    xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
    return true;
}
bool m5core2_audio::stop_recording() {
    m5core2_audio_msg.cmd = 0;
    if(m5core2_audio_queue!=nullptr) {
        xQueueSend(m5core2_audio_queue,&m5core2_audio_msg,portMAX_DELAY);
        return true;
    }
    return false;
}