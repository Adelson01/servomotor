Controle de Servo Motor com Joystick no Raspberry Pi Pico
Este projeto demonstra como controlar a posição de um servo motor utilizando um joystick analógico conectado a um Raspberry Pi Pico. O programa lê a posição do eixo X do joystick através de um conversor analógico-digital (ADC) e converte esse valor em um sinal de modulação por largura de pulso (PWM) para comandar o servo.

Este código foi desenvolvido para ser utilizado com o simulador online Wokwi.

Equipe e Contribuições
Este projeto foi uma colaboração dos seguintes membros:

Danielle: Responsável pela pesquisa e implementação da configuração do PWM (Modulação por Largura de Pulso) para o controle preciso do servo motor.

Guilherme: Responsável pela configuração do Conversor Analógico-Digital (ADC), permitindo a leitura correta dos dados do joystick.

Adelson: Responsável pela integração do código do ADC com o PWM, criando a lógica principal do programa e unindo as partes para o funcionamento completo do sistema.

Componentes Utilizados
Raspberry Pi Pico W

Servo Motor (padrão, como o SG90)

Módulo de Joystick Analógico

Montagem do Circuito
A conexão entre os componentes foi realizada da seguinte forma:

Componente

Pino

Conexão no Raspberry Pi Pico

Joystick

GND

GND

VCC

3V3

HORZ (Eixo X)

GP27 (ADC1)

Servo Motor

GND (fio marrom)

GND

V+ (fio vermelho)

VBUS

PWM (fio laranja)

GP20


Exportar para as Planilhas
Lógica do Projeto
O objetivo do projeto é traduzir o movimento físico do joystick em um movimento angular correspondente no servo motor. A lógica para isso segue os seguintes passos:

Leitura Analógica: O Raspberry Pi Pico utiliza seu Conversor Analógico-Digital (ADC) para ler a tensão no pino GP27, que está conectado ao eixo horizontal (HORZ) do joystick. Essa leitura resulta em um valor digital entre 0 (posição mínima) e 4095 (posição máxima).

Mapeamento de Valores: O servo motor não entende a escala de 0 a 4095. Ele é controlado pela duração (largura) de um pulso elétrico, medido em microssegundos (µs), enviado a uma frequência de 50 Hz.

Um pulso de ~500 µs corresponde à posição de 0 graus.

Um pulso de ~2400 µs corresponde à posição de 180 graus.
A função map() é usada para converter (mapear) a faixa de valores do ADC (0 - 4095) para a faixa de largura de pulso do servo (500 - 2400 µs).

Geração de PWM: Com base no valor de pulso calculado, o Pico gera um sinal PWM no pino GP20. Este sinal tem uma frequência constante de 50 Hz, mas sua largura de pulso varia conforme o joystick é movido. Essa variação na largura do pulso comanda o servo motor a se mover para a posição desejada.

Loop Contínuo: Todo o processo ocorre dentro de um loop infinito (while (true)), garantindo que o servo reaja em tempo real ao movimento do joystick.

Análise do Código
O código-fonte (main.c) é estruturado para inicializar os periféricos do Pico e executar a lógica de controle em um loop.

C

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"

// Definições dos pinos e parâmetros
#define VRX_PIN 27      // GPIO do eixo X do Joystick (ADC1)
#define SERVO_GPIO 20   // GPIO para o sinal do Servo

#define SERVO_MIN_US 500  // Largura do pulso para 0 graus
#define SERVO_MAX_US 2400 // Largura do pulso para 180 graus

/*
 * Função para mapear um valor de uma faixa para outra.
 * Ex: Mapeia o valor do ADC (0-4095) para o pulso do servo (500-2400us).
 */
long map(long value, long fromLow, long fromHigh, long toLow, long toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/*
 * Configura e envia o sinal PWM para o servo.
 * Esta função ajusta o hardware do Pico para gerar um pulso
 * com a largura (em microssegundos) especificada.
 */
void set_servo_pulse(uint gpio, uint16_t pulse_width_us) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    
    // Configura o clock do PWM para ter a precisão necessária
    uint32_t wrap = 39062;
    pwm_set_wrap(slice_num, wrap);
    pwm_set_clkdiv(slice_num, 64.0f);

    // Calcula o nível do PWM para gerar a largura de pulso desejada
    uint32_t level = (uint32_t)((pulse_width_us / 20000.0f) * wrap);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), level);

    pwm_set_enabled(slice_num, true);
}


int main() {
    // Inicializa a comunicação serial para debug
    stdio_init_all();

    // --- Inicialização do ADC ---
    adc_init();
    adc_gpio_init(VRX_PIN);
    adc_select_input(1); // Seleciona o canal ADC1 (GP27)

    // --- Inicialização do PWM para o Servo ---
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);
    
    // Variáveis para controlar a impressão no monitor serial a cada segundo
    uint32_t last_print_time_ms = to_ms_since_boot(get_absolute_time());
    const uint32_t print_interval_ms = 1000; // 1 segundo

    while (true) {
        // 1. LER a posição do joystick
        uint16_t adc_x = adc_read();

        // 2. TRADUZIR a leitura do ADC para a largura de pulso do servo
        uint16_t pulse_width = map(adc_x, 0, 4095, SERVO_MIN_US, SERVO_MAX_US);

        // 3. AGIR: Enviar o pulso para o servo
        set_servo_pulse(SERVO_GPIO, pulse_width);

        // 4. LÓGICA PARA IMPRIMIR A CADA SEGUNDO
        // Isso evita sobrecarregar o monitor serial com mensagens
        uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
        if (current_time_ms - last_print_time_ms >= print_interval_ms) {
            printf("ADC: %4d -> Pulso: %4d us\n", adc_x, pulse_width);
            last_print_time_ms = current_time_ms;
        }

        // 5. PAUSAR para sincronizar com o ciclo do servo (50 Hz = 20 ms)
        sleep_ms(20);
    }
}
Detalhes do Código:
#define: Constantes são usadas para facilitar a leitura e a modificação dos pinos e parâmetros do servo.

map(): Função de utilidade que realiza a conversão matemática entre as faixas de valores do ADC e do PWM.

set_servo_pulse(): É o coração do controle do servo. Ela configura o periférico PWM do Pico (chamado de slice) com um divisor de clock e um valor de "wrap" para criar uma frequência de 50 Hz. Em seguida, ajusta o level do canal para definir a largura do pulso.

main():

Inicialização: Prepara o ADC e o pino GPIO para a função PWM.

Controle de Tempo de Impressão: Uma lógica simples usando to_ms_since_boot garante que os valores de debug (ADC e Pulso) sejam impressos no monitor serial apenas uma vez por segundo, tornando a saída legível.

Loop Principal: Lê o ADC, mapeia o valor, atualiza o servo e faz uma pequena pausa (sleep_ms(20)). Essa pausa é importante para sincronizar o loop com o período do sinal do servo (20 ms), otimizando o uso do processador.
