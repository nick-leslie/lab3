#define SAMPLING_INT_PRIORITY 0  // button interrupt priority (higher number is lower priority)
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates
#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define PIXELS_PER_DIV 20

void sample_init();
void update_draw_buffer();
void ADC_ISR(void);
void display_task();
void waveform_task();
void processing_task();
void user_input_task();
uint32_t cpu_load_count(void);
