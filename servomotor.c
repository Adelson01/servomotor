#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h" // <<< 1. INCLUÍDO para usar funções de tempo

// Definições
#define VRX_PIN 27      // GPIO para eixo X
#define SERVO_GPIO 20

// --- Configuração do PWM ---
#define PWM_FREQ 50      // 50 Hz para o servo
#define PERIOD_US 20000  // 20ms em microssegundos

// --- Limites do Servo ---
// Valores típicos de pulso em microssegundos (µs)
#define SERVO_MIN_US 500   // Posição de 0 graus
#define SERVO_MAX_US 2400  // Posição de 180 graus

long map(long value, long fromLow, long fromHigh, long toLow, long toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void set_servo_pulse(uint gpio, uint16_t pulse_width_us) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    uint32_t wrap = 39062;
    pwm_set_wrap(slice_num, wrap);
    pwm_set_clkdiv(slice_num, 64.0f);

    uint32_t level = (uint32_t)((pulse_width_us / 20000.0f) * wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), level);

    pwm_set_enabled(slice_num, true);
}

int main() {
    stdio_init_all();

    printf("Iniciando controle de servo com joystick...\n");

    // --- Inicialização do ADC ---
    adc_init();
    adc_gpio_init(VRX_PIN);
    adc_select_input(1);

    // --- Inicialização do PWM para o Servo ---
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);

    // <<< 2. VARIÁVEIS PARA CONTROLE DE TEMPO DA IMPRESSÃO
    // Pega o tempo atual em milissegundos para iniciar a contagem
    uint32_t last_print_time_ms = to_ms_since_boot(get_absolute_time());
    // Define o intervalo desejado em milissegundos
    const uint32_t print_interval_ms = 1000; // 5 segundos

    while (true) {
        // 1. LER a posição do joystick
        uint16_t adc_x = adc_read();

        // 2. TRADUZIR a leitura do ADC para a largura de pulso do servo
        uint16_t pulse_width = map(adc_x, 0, 4095, SERVO_MIN_US, SERVO_MAX_US);

        // 3. AGIR: Enviar o pulso para o servo
        set_servo_pulse(SERVO_GPIO, pulse_width);


        // <<< 3. LÓGICA PARA IMPRIMIR A CADA 5 SEGUNDOS
        // Pega o tempo atual
        uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());

        // Verifica se a diferença entre o tempo atual e o da última impressão
        // é maior ou igual ao intervalo desejado.
        if (current_time_ms - last_print_time_ms >= print_interval_ms) {
            // Se for, imprime os valores
            printf("ADC: %4d -> Pulso: %4d us\n", adc_x, pulse_width);
            // E atualiza o tempo da última impressão para o tempo atual
            last_print_time_ms = current_time_ms;
        }


        // 4. PAUSAR para sincronizar com o servo (continua sendo importante)
        sleep_ms(20);
    }
}