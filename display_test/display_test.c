#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "ssd1306.h"
#include "font.h"
#include "application.h"
#include "hardware/dma.h"
#include "fourier/include/kiss_fftr.h"
#include <math.h>
#include <time.h>
// #include "fourier/include/fft.h"

// SAMPLING RATE : 40000 Hz

/**
 * TODO:
 * Windowing
 * Create FFT implementation
 *
 * */

#define SDA 12
#define SCL 13

#define L_IN 27
#define R_IN 26

#define DOMAIN_SWITCH 28
#define SCALE_SWITCH 18

#define SAMPLE_SIZE 256 // Look at this
#define CLOCK_DIV 2400

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAX_SCALE_FACTOR 4

#define BIN_COUNT 10

void setup_gpios(void);
void init_display(ssd1306_t *disp);
void init_status_led(void);
void init_project(ssd1306_t *disp);
void convert_adc_to_voltage(double *returner, uint16_t *buffer);
bool create_waveform(ssd1306_t *disp, uint16_t *raw_buffer);
void sampleADC(uint16_t *capture_buf);
void compute_fft(kiss_fftr_cfg *kiss_config, uint16_t *capture_buffer, kiss_fft_cpx *output_buffer);
bool create_spectrum(ssd1306_t *disp, kiss_fft_cpx *fft_output);
void compute_max_min(kiss_fft_cpx *fft_output, float *max, float *min);
void init_buttons();
void handle_scale_switch(kiss_fftr_cfg *kiss_config);

static uint dma_chan;
static dma_channel_config dma_config;

static absolute_time_t curr_time_domain;
static absolute_time_t prev_time_domain = 0;

static absolute_time_t curr_time_scale;
static absolute_time_t prev_time_scale = 0;

static volatile DomainState domain_state = FREQUENCY;
static volatile int scale_factor = 1;

// static  int sample_size = 128;

static volatile bool scale_changed = false;

void gpio_callback(uint gpio, uint32_t events)
{
    if(curr_time_domain - curr_time_scale > 200000){ //both buttons pressed, return to defaults


    }
    switch (gpio)
    {
    case DOMAIN_SWITCH:
        curr_time_domain = time_us_64();
        if (curr_time_domain - prev_time_domain > 200000)
        {
            domain_state = (domain_state + 1) % 2;
            printf("Domain Toggled\n");
            // printf("Domain: %d\n", domain_state);
            prev_time_domain = curr_time_domain;
        }
        break;
    case SCALE_SWITCH:
        curr_time_scale = time_us_64();
        if (curr_time_scale - prev_time_scale > 200000)
        {
            scale_changed = true;
            scale_factor = scale_factor << 1;
            if (scale_factor > MAX_SCALE_FACTOR)
            {
                scale_factor = 1;
            }
            prev_time_scale = curr_time_scale;
        }
        break;
    }
 //both buttons pressed
}

int main()
{
    ssd1306_t disp;
    init_project(&disp);
    ApplicationState appState = SAMPLING;

    uint16_t raw_voltages[SAMPLE_SIZE * MAX_SCALE_FACTOR];
    double converted_voltages[SAMPLE_SIZE * MAX_SCALE_FACTOR];
    bool finished = false;

    dma_chan = dma_claim_unused_channel(true);

    dma_config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);

    // kiss_fft_cpx fft_output[SAMPLE_SIZE];
    // kiss_fftr_cfg kiss_config = kiss_fftr_alloc(SAMPLE_SIZE, false, NULL, NULL);

    kiss_fft_cpx fft_output[MAX_SCALE_FACTOR * SAMPLE_SIZE];

    kiss_fftr_cfg kiss_config = kiss_fftr_alloc(SAMPLE_SIZE * scale_factor, false, NULL, NULL);

    float max, min;

    /**
     * Main Loop
     */
    while (1)
    {
        switch (appState)
        {
        case SAMPLING:
            printf("SAMPLING\n");
            // finished = sampleADC(raw_voltages);
            sampleADC(raw_voltages);
            convert_adc_to_voltage(converted_voltages, raw_voltages);
            if (domain_state == FREQUENCY)
                appState = CALCULATING;
            else
                appState = DISPLAY_WAVEFORM;
            finished = false;
            ssd1306_clear(&disp);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            break;

        case CALCULATING:
            printf("COMPUTING\n");
            handle_scale_switch(&kiss_config);
            compute_fft(&kiss_config, raw_voltages, fft_output);
            appState = DISPLAY_SPECTRUM;
            break;

        case DISPLAY_WAVEFORM:

            printf("DISPLAYING WAVEFORM\n");
            finished = create_waveform(&disp, raw_voltages);
            if (finished)
            {
                char str[10];
                sprintf(str, "x%d", scale_factor);
                ssd1306_draw_string(&disp, 115, 0, 1, str);
                ssd1306_show(&disp);
                appState = SAMPLING;
                finished = false;
                gpio_put(PICO_DEFAULT_LED_PIN, true); // turn LED on for sampling
            }
            break;

        case DISPLAY_SPECTRUM:

            printf("DISPLAYING SPECTRUM\n");
            finished = create_spectrum(&disp, fft_output);
            if (finished)
            {
                char str[10];
                sprintf(str, "x%d", scale_factor);
                ssd1306_draw_string(&disp, 115, 0, 1, str);
                ssd1306_show(&disp);
                appState = SAMPLING;
                finished = false;
                gpio_put(PICO_DEFAULT_LED_PIN, true);
            }
            break;

        default:
            break;
        }
    }
}

void sampleADC(uint16_t *capture_buf)
{
    adc_fifo_drain();
    adc_run(false);
    dma_channel_configure(dma_chan, &dma_config,
                          capture_buf,                // dst
                          &adc_hw->fifo,              // src
                          SAMPLE_SIZE * scale_factor, // transfer count
                          true                        // start immediately
    );
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
}

void convert_adc_to_voltage(double *returner, uint16_t *buffer)
{
    static int i = 0;
    returner[i] = (buffer[i] - 1757.55636364) * (3.3 / 4096.0);
    i = i + 1;
    if (i >= SAMPLE_SIZE - 1)
    {
        i = 0;
    }
}

#define DC_OFFSET 920
bool create_waveform(ssd1306_t *disp, uint16_t *raw_buffer)
{
    static int i = 0;
    ssd1306_draw_line(disp, i, SCREEN_HEIGHT - 1, i, (DC_OFFSET - raw_buffer[i * scale_factor] / 2)*1.5);

    // // printf("Buffer: %d \n", raw_buffer[i * 2]);
    i = i + 1;
    if (i >= SCREEN_WIDTH - 1)
    {
        i = 0;
        return true;
    }
    return false;
}

float find_minimum(float mag)
{
    static float min_val = -80;
    if (mag < min_val && mag != -80)
    {
        min_val = mag;
    }
    return min_val;
}

float find_maximum(float mag, int i)
{

    static float max_val = -80;
    if (mag > max_val)
    {
        max_val = mag;
    }
    if (i >= 127)
    {
        max_val = -80;
    }
    return max_val;
}

bool create_spectrum(ssd1306_t *disp, kiss_fft_cpx *fft_output)
{

    static int i = 0;
    float magnitude = sqrtf(fft_output[i].r * fft_output[i].r + fft_output[i].i * fft_output[i].i) / (SAMPLE_SIZE * scale_factor) * 1.0;
    if (magnitude == 0)
    {
        magnitude = 0.0001;
    }
    float magnitude_db = (20 * log10(magnitude));
    int frequency = 10000.0 / SAMPLE_SIZE * i; // 1/N needed if looking at frequency domain amplitudes
    // printf("Magnitude for %d: %0.4f\n", frequency, magnitude);
    // printf("Decibels for %d: %0.4f\n", frequency, magnitude_db);

    ssd1306_draw_line(disp, i, ((SCREEN_HEIGHT - 1) - (magnitude_db + 75) * 2), i, SCREEN_HEIGHT - 1);
    i = i + 1;
    if (i >= SCREEN_WIDTH - 1)
    {
        i = 0;
        return true;
    }
    return false;
    
}

void init_project(ssd1306_t *disp)
{
    init_status_led();
    stdio_init_all();
    setup_gpios();
    init_display(disp);

    sleep_ms(100);

    adc_init();
    adc_gpio_init(R_IN);
    adc_gpio_init(L_IN);
    adc_select_input(0);
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false);
    adc_set_clkdiv(CLOCK_DIV);
    init_buttons();
}

void init_display(ssd1306_t *disp)
{
    disp->external_vcc = false;
    ssd1306_init(disp, 128, 64, 0x3C, i2c0);
    ssd1306_clear(disp);
}

void setup_gpios(void)
{
    i2c_init(i2c0, 400000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
}

void init_status_led(void)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);
}

float compute_average(uint16_t *capture_buffer)
{
    int sum = 0;
    for (int i = 0; i < SAMPLE_SIZE * scale_factor; i++)
    {
        sum += capture_buffer[i];
    }
    return sum * 1.0 / (SAMPLE_SIZE * scale_factor);
}

void compute_fft(kiss_fftr_cfg *kiss_config, uint16_t *capture_buffer, kiss_fft_cpx *output_buffer)
{
    float avg = compute_average(capture_buffer);

    float fft_buffer[SAMPLE_SIZE * scale_factor];

    for (int i = 0; i < SAMPLE_SIZE * scale_factor; i++)
    {
        fft_buffer[i] = (float)(capture_buffer[i] - avg) / 2048.0f;
    }

    kiss_fftr(*kiss_config, fft_buffer, output_buffer);
}

void init_buttons()
{
    gpio_init(DOMAIN_SWITCH);
    gpio_set_dir(DOMAIN_SWITCH, GPIO_IN);
    gpio_pull_up(DOMAIN_SWITCH);

    gpio_init(SCALE_SWITCH);
    gpio_set_dir(SCALE_SWITCH, GPIO_IN);
    gpio_pull_up(SCALE_SWITCH);

    // gpio_set_irq_enabled_with_callback(DOMAIN_SWITCH, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(SCALE_SWITCH, GPIO_IRQ_EDGE_FALL, true, &scale_switch_callback);

    gpio_set_irq_callback(&gpio_callback);
    gpio_set_irq_enabled(DOMAIN_SWITCH, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(SCALE_SWITCH, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void handle_scale_switch(kiss_fftr_cfg *kiss_config)
{
    if (scale_changed)
    {
        printf("SWITCH HANDLED");
        free(*kiss_config);
        *kiss_config = kiss_fftr_alloc(SAMPLE_SIZE * scale_factor, false, NULL, NULL);
        scale_changed = false;
    }
}
