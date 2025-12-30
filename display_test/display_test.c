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
// #include "fourier/include/fft.h"

// SAMPLING RATE : 40000 Hz

#define SDA 12
#define SCL 13

#define L_IN 27
#define R_IN 26

#define SAMPLE_SIZE 256
#define CLOCK_DIV 4800

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

static uint dma_chan;
static dma_channel_config dma_config;

int main()
{
    ssd1306_t disp;
    init_project(&disp);
    ApplicationState appState = SAMPLING;

    uint16_t raw_voltages[SAMPLE_SIZE];
    double converted_voltages[SAMPLE_SIZE];
    bool finished = false;

    dma_chan = dma_claim_unused_channel(true);

    dma_config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);

    kiss_fft_cpx fft_output[SAMPLE_SIZE];
    kiss_fftr_cfg kiss_config = kiss_fftr_alloc(SAMPLE_SIZE, false, NULL, NULL);

    /**
     * Main Loop
     */
    while (1)
    {
        switch (appState)
        {
        case SAMPLING:
            // printf("SAMPLING\n");
            // finished = sampleADC(raw_voltages);
            sampleADC(raw_voltages);
            convert_adc_to_voltage(converted_voltages, raw_voltages);
            appState = CALCULATING;
            finished = false;
            ssd1306_clear(&disp);
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            break;

        case CALCULATING:
            // printf("COMPUTING\n");
            compute_fft(&kiss_config, raw_voltages, fft_output);
            appState = DISPLAY_SPECTRUM;
            break;

        case DISPLAY_WAVEFORM:

            // printf("DISPLAYING WAVEFORM\n");
            finished = create_waveform(&disp, raw_voltages);
            if (finished)
            {
                ssd1306_show(&disp);
                appState = SAMPLING;
                finished = false;
                gpio_put(PICO_DEFAULT_LED_PIN, true); // turn LED on for sampling
            }
            break;

        case DISPLAY_SPECTRUM:

            // printf("DISPLAYING SPECTRUM");
            finished = create_spectrum(&disp, fft_output);

            if (finished)
            {
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

// bool sampleADC(int *buffer)
// {
//     static int i = 0;
//     buffer[i] = adc_read();
//     sleep_us(50); // 50 us, sampling frequency of 20khz
//     i = i + 1;
//     if (i >= SAMPLE_SIZE - 1)
//     {
//         i = 0;
//         return true;
//     }
//     return false;
// }

void sampleADC(uint16_t *capture_buf)
{
    adc_fifo_drain();
    adc_run(false);
    dma_channel_configure(dma_chan, &dma_config,
                          capture_buf,   // dst
                          &adc_hw->fifo, // src
                          SAMPLE_SIZE,   // transfer count
                          true           // start immediately
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

bool create_waveform(ssd1306_t *disp, uint16_t *raw_buffer)
{
    static int i = 0;
    ssd1306_draw_line(disp, i, 63, i, 930 - raw_buffer[i * 2] / 2);
    // printf("Buffer: %d \n", raw_buffer[i * 2]);
    i = i + 1;
    if (i >= 127)
    {
        i = 0;
        return true;
    }
    return false;
}

bool create_spectrum(ssd1306_t *disp, kiss_fft_cpx *fft_output)
{
    static int i = 0;
    float magnitude = sqrtf (fft_output[i].r * fft_output[i].r + fft_output[i].i * fft_output[i].i) / SAMPLE_SIZE * 1.0;
    float magnitude_db = (20 * log10(magnitude));
    int frequency = 10000.0/SAMPLE_SIZE * i; //1/N needed if looking at frequency domain amplitudes
    printf("Magnitude for %d: %d\n", frequency, magnitude);
    printf("Decibels for %d: %d\n", frequency, magnitude_db);
    ssd1306_draw_line(disp, i, 63 - magnitude_db * 4, i, 63);
    i = i + 1;
    if (i >= 127)
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
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        sum += capture_buffer[i];
    }
    return sum * 1.0 / SAMPLE_SIZE;
}

void compute_fft(kiss_fftr_cfg *kiss_config, uint16_t *capture_buffer, kiss_fft_cpx *output_buffer)
{
    float avg = compute_average(capture_buffer);

    float fft_buffer[SAMPLE_SIZE];

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        fft_buffer[i] = (float)(capture_buffer[i] - avg);
    }

    kiss_fftr(*kiss_config, fft_buffer, output_buffer);
}
