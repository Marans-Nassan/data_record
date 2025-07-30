#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

// Definições de portas I2C e pinos para sensor e display
#define i2c_port i2c0
#define i2c_port_display i2c1
#define i2c_sda 0          // Pino SDA para I2C do sensor
#define i2c_scl 1          // Pino SCL para I2C do sensor
#define i2c_sda_display 14 // Pino SDA para I2C do display
#define i2c_scl_display 15 // Pino SCL para I2C do display
#define endereco_display 0x3c // Endereço I2C do OLED

// Definições de pinos GPIO
#define bot_a 5          // Pino do botão A
#define bot_b 6          // Pino do botão B
#define green_led 11     // Pino do LED verde
#define blue_led 12      // Pino do LED azul
#define red_led 13       // Pino do LED vermelho
#define buzz_a 21        // Pino do buzzer

// Dimensões do display
#define DISP_W 128       // Largura do OLED
#define DISP_H 64        // Altura do OLED

// Habilita interrupções GPIO para os pinos informados
#define interrupcoes(botoes) gpio_set_irq_enabled_with_callback(botoes, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

// Endereço I2C do MPU6050
static int addr = 0x68; // Endereço do barramento I2C do MPU6050

// Estrutura de controle do display SSD1306
ssd1306_t ssd;

// Flags e temporização para registro de dados
static bool logger_enabled;           // Habilita/desabilita logging
static const uint32_t period = 1000; // Período de registro em milissegundos
static absolute_time_t next_log_time; // Próximo tempo agendado para registro

// Buffers de dados brutos do IMU
int16_t acceleration[3], gyro[3], temp; // Leituras de aceleração, giroscópio e temperatura

// Buffer para nome de arquivo de log
static char filename[20]; // Armazena nome único para arquivo CSV

// Flags para controle do cartão SD e captura
volatile bool sd_montado = false;     // Flag de cartão SD montado
volatile bool stop_capture = false;   // Requisição para parar captura
volatile bool adentrando_a = false;   // Estado de pressão do botão A
volatile bool adentrando_b = false;   // Estado de pressão do botão B
volatile bool capture_running = false; // Flag de captura em andamento
bool alteracao = false;                // Flag de atualização do display

// Slice PWM para buzzer
uint8_t slice = 0; // Número do slice PWM para o buzzer

// Buffers de texto para display
char display_s[120]; // Texto atual a ser exibido no OLED
char display_padrao[] = {
    "1.Montar SD    "
    "2.Desmontar SD "
    "3.Listar Dir   "
    "4.Ultimo arquiv"
    "5.Esp.Livre    "
    "6.Capturar data"
    "7.Formatar SD  "
}; // Strings do menu padrão concatenadas

// Protótipos de funções com explicação de propósito
void display(void); // Core1: atualiza continuamente o display OLED conforme flag 'alteracao'
void init_led(void); // Inicializa os GPIOs dos LEDs
void init_bot(void); // Inicializa os GPIOs dos botões com pull-ups
void bot_a_irq(void); // Trata ações do botão A fora da ISR
void bot_b_irq(void); // Trata ações do botão B fora da ISR
void i2c_sensor(void); // Configura I2C para sensor MPU6050
void i2c_display(void); // Configura I2C para display SSD1306
void oled_config(void); // Inicializa e limpa o display SSD1306
void pwm_setup(void); // Configura PWM para buzzer
void pwm_beep(uint gpio, float duty, uint8_t times, float sec, bool ramp, bool use_end, bool end_high); // Gera sequência de bipes no buzzer
void gpio_irq_handler(uint gpio, uint32_t events); // Callback de IRQ GPIO para botões

// Funções utilitárias do SD
static sd_card_t *sd_get_by_name(const char *const name); // Retorna struct do SD pelo nome
static FATFS *sd_get_fs_by_name(const char *name); // Retorna objeto FATFS pelo nome de drive

// Handlers de comandos
static void run_setrtc(void);  // Ajusta RTC via entrada serial
static void run_format(void);  // Formata cartão SD
static void run_mount(void);   // Monta cartão SD
static void run_unmount(void); // Desmonta cartão SD
static void run_getfree(void); // Verifica espaço livre no SD
static void run_ls(void);      // Lista diretório
static void run_cat(void);     // Exibe conteúdo de arquivo

// Funções auxiliares para captura de dados
void generate_unique_filename(void);         // Gera nome único log_NNN.csv
void capture_data_and_save(void);           // Captura dados IMU e grava em CSV
void read_file(const char *filename);        // Lê e imprime conteúdo de arquivo

static void run_help(void);  // Imprime menu de comandos disponíveis
static void process_stdio(int cRxedChar); // Analisa e executa comandos seriais
static void mpu6050_reset(void); // Reseta sensor MPU6050
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp); // Lê dados brutos do IMU


typedef void (*p_fn_t)();
typedef struct{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}};

int main(){
    alteracao = true;
    snprintf(display_s, sizeof(display_s), "Inicializando");
    stdio_init_all();
    multicore_launch_core1(display);
    init_led();
    init_bot();
    pwm_setup();
    i2c_sensor();
    gpio_put(green_led, 1);
    gpio_put(red_led, 1);
    sleep_ms(5000);
    time_init();
    interrupcoes(bot_a);
    interrupcoes(bot_b);
    gpio_put(green_led, 0);
    gpio_put(red_led, 0);
    bi_decl(bi_2pins_with_func(i2c_sda, i2c_scl, GPIO_FUNC_I2C));
    stdio_flush();
    run_help();
    mpu6050_reset();
    alteracao = false;
    snprintf(display_s, sizeof(display_s), "%s", display_padrao);
    while (true){
        mpu6050_read_raw(acceleration, gyro, &temp);
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar) process_stdio(cRxedChar);

        if (cRxedChar == '1'){ // Monta o SD card se pressionar '1'
            printf("\nMontando o SD...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Montando o SD  ");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 1);
            pwm_beep(buzz_a, 0.5f, 1, 0.25f, false, false, false);
            run_mount();
            sleep_ms(100);
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '2'){ // Desmonta o SD card se pressionar '2'
            printf("\nDesmontando o SD. Aguarde...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Desmontando SD ");            
            gpio_put(green_led, 0);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            pwm_beep(buzz_a, 0.5f, 2, 0.25f, false, false, false);
            run_unmount();
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '3'){ // Lista diretórios e os arquivos se pressionar '3'
            printf("\nListagem de arquivos no cartão SD.\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "List. arquivos ");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            pwm_beep(buzz_a, 0.5f, 1, 0.1f, false, false, false);
            run_ls();
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '4'){ // Exibe o conteúdo do último arquivo capturado na sessão arquivo ao pressionar '4'
            printf("\nExibindo conteúdo do último arquivo...");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Ultimo arquivo ");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 1);
            gpio_put(red_led, 0);
            pwm_beep(buzz_a, 0.5f, 2, 0.1f, false, false, false);
            read_file(filename);
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("Escolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '5'){ // Obtém o espaço livre no SD card se pressionar '5'
            printf("\nObtendo espaço livre no SD.\n\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Checando espaço");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 1);
            gpio_put(red_led, 0);   
            pwm_beep(buzz_a, 0.5f, 1, 0.7f, false, false, false); 
            run_getfree();
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '6'){ // Captura dados e salva no arquivo se pressionar '6'
            printf("\nCapturando os dados...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Captura de dado");
            gpio_put(green_led, 0);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 1);
            pwm_beep(buzz_a, 0.5f, 1, 1.2f, false, false, false);
            generate_unique_filename();
            capture_data_and_save();
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '7'){ // Formata o SD card se pressionar '7'
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Formatando SD  ");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 1);
            gpio_put(red_led, 1);
            pwm_beep(buzz_a, 0.8f, 3, 1.0f, false, false, false);
            run_format();
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (8 = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        }
        if (cRxedChar == '8') run_help(); // Exibe os comandos disponíveis no serial monitor se pressionar '8'
        bot_a_irq();
        bot_b_irq();
        sleep_ms(500);
    }
    return 0;
}

void display(void){
    i2c_display();
    oled_config();
        while(true){
        if(alteracao){
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, display_s, 0, 25);  
            ssd1306_send_data(&ssd);
        }else {
            ssd1306_draw_string(&ssd, display_s, 0, 0);  
            ssd1306_send_data(&ssd);
        }
    }
}

void init_led(void){
    for(uint8_t leds = 11; leds <14; leds++){
        gpio_init(leds);
        gpio_set_dir(leds, GPIO_OUT);
        gpio_put(leds, 0);
    }
}

void init_bot(void){
    for(uint8_t bots = 5 ; bots < 7; bots++){
        gpio_init(bots);
        gpio_set_dir(bots, GPIO_IN);
        gpio_pull_up(bots);
    }
}

void bot_a_irq(void){
    if(adentrando_a){
        if(!capture_running){
            capture_running = true;
            stop_capture = false;
            printf("\nCapturando os dados...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Captura de dado");
            gpio_put(green_led, 0);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 1);
            pwm_beep(buzz_a, 0.5f, 1, 1.2f, false, false, false);
            generate_unique_filename();
            capture_data_and_save();
            capture_running = false;
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);                
            printf("\nEscolha o comando (h = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
        } 
    }
    adentrando_a = false;
}

void bot_b_irq(void){
    if(adentrando_b){
        if(!sd_montado){
            printf("\nMontando o SD...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Montando o SD  ");
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 1);
            pwm_beep(buzz_a, 0.5f, 1, 0.5f, false, false, false);
            run_mount();
            sleep_ms(100);
            gpio_put(green_led, 1);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            printf("\nEscolha o comando (h = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
            sd_montado = true;
        } else {
            printf("\nDesmontando o SD. Aguarde...\n");
            alteracao = true;
            snprintf(display_s, sizeof(display_s), "Desmontando SD ");
            gpio_put(green_led, 0);
            gpio_put(blue_led, 0);
            gpio_put(red_led, 0);
            pwm_beep(buzz_a, 0.5f, 2, 0.5f, false, false, false);
            run_unmount();
            printf("\nEscolha o comando (h = help):  ");
            alteracao = false;
            snprintf(display_s, sizeof(display_s), "%s", display_padrao);
            sd_montado = false;
        }
    }
    adentrando_b = false;
}

void i2c_sensor(void){
    i2c_init(i2c_port, 400 * 1000);
    gpio_set_function(i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda);
    gpio_pull_up(i2c_scl);
}

void i2c_display(void){
    i2c_init(i2c_port_display, 400 * 1000);
    gpio_set_function(i2c_sda_display, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_display, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_display);
    gpio_pull_up(i2c_scl_display);
}

void oled_config(void){
    ssd1306_init(&ssd, DISP_W, DISP_H, false, endereco_display, i2c_port_display);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void pwm_setup(void){
    gpio_set_function(buzz_a, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(buzz_a);
    pwm_set_clkdiv(slice, 32.0f);
    pwm_set_wrap(slice, 7812);
    pwm_set_enabled(slice, true);
}
void pwm_beep(uint gpio, float duty, uint8_t times, float sec, bool ramp, bool use_end, bool end_high) {
    uint32_t wrap  = 7812;
    const int steps = 100;
    float total_ms = sec * 1000.0f;
    float phase_ms = ramp ? (total_ms / 2.0f) : total_ms;
    float delay_ms = phase_ms / steps;

    if (times == 0 && !ramp) {
        pwm_set_chan_level(slice, PWM_CHAN_A, duty * wrap);
        return;
    }

    for (int t = 0; t < (times>0?times:1); t++) {
        if (ramp) {
            // up
            for (int i = 0; i <= steps; i++) {
                pwm_set_gpio_level(buzz_a, duty * i/steps * wrap);
                sleep_ms((uint)delay_ms);
            }
            // down
            for (int i = steps; i >= 0; i--) {
                 pwm_set_gpio_level(buzz_a, duty * i/steps * wrap);
                sleep_ms((uint)delay_ms);
            }
            if (use_end) {
                if (end_high) {
                    // sobe de novo e termina no pico
                    for (int i = 0; i <= steps; i++) {
                         pwm_set_gpio_level(buzz_a, duty * i/steps * wrap);
                        sleep_ms((uint)delay_ms);
                    }
                    break;
                } else {
                    break; // termina em baixo (já está em zero)
                }
            }
        } else {
             pwm_set_gpio_level(buzz_a, duty * wrap);
            sleep_ms((uint)total_ms);
             pwm_set_gpio_level(buzz_a, 0);
        }
        sleep_ms(100);
    }
    // se não usar modo de término com alto, garante nível baixo
    if (!(ramp && use_end && end_high))
         pwm_set_gpio_level(buzz_a, 0);
}

void gpio_irq_handler(uint gpio, uint32_t events){
    uint64_t current_time = to_ms_since_boot(get_absolute_time());
    static uint64_t last_time_a = 0 , last_time_b = 0;
    if(gpio == bot_a && (current_time - last_time_a > 300)){
        if(capture_running){
            stop_capture = true;
            adentrando_a = false;
        } else adentrando_a = true;

        last_time_a = current_time;
    } else if(gpio == bot_b &&(current_time - last_time_b > 300)){
        adentrando_b = true;
        last_time_b = current_time;
    }
}

static sd_card_t *sd_get_by_name(const char *const name){
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name)) 
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static FATFS *sd_get_fs_by_name(const char *name){
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static void run_setrtc(void){
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr){
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr){
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr){
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr){
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr){
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr){
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

static void run_format(void){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_mount(void){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs){
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr){
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}
static void run_unmount(void){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs){
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr){
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}
static void run_getfree(void){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs){
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr){
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}
static void run_ls(void){
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0]){
        p_dir = arg1;
    } else{
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr){
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr){
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    while (fr == FR_OK && fno.fname[0]){
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
}
static void run_cat(void){
    char *arg1 = strtok(NULL, " ");
    if (!arg1){
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr){
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil)){
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}

// Função para capturar dados e salvar no arquivo *.csv
void generate_unique_filename(void) {
    int index = 0;
    FIL file;
    FRESULT res;
    do {
        snprintf(filename, sizeof(filename), "log_%03d.csv", index++);
        res = f_open(&file, filename, FA_READ);
        if (res != FR_OK) break; // Arquivo não existe, pode usar
        f_close(&file);
    } while (index < 1000);
}

void capture_data_and_save(void){
    char buffer[80];
    char header[] = "id,ax,ay,az,gx,gy,gz,temp\n";
    printf("\nCapturando dados. Aguarde finalização...\n");

    FIL file;
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK){
        printf("\n[ERRO] Não foi possível abrir o arquivo para escrita. Monte o cartão.\n");
        return;
    }
    // Escreve cabeçalho
    UINT bw;
    res = f_write(&file, header, strlen(header), &bw);
    if (res != FR_OK){
        printf("[ERRO] Falha ao escrever cabeçalho.\n");
        f_close(&file);
        return;
    }
    f_sync(&file);

    for (int i = 0; i < 128; i++){
        if (stop_capture) {
            f_sync(&file);  // garante persistência
            printf("\n[INFO] Captura interrompida pelo usuário.\n");
            break;
        }

        mpu6050_read_raw(acceleration, gyro, &temp);

        float temperature = (temp / 340.0f) + 36.53f;
        int len = sprintf(buffer, "%d,%ld,%ld,%ld,%ld,%ld,%ld,%.1f\n",
                          i + 1,
                          acceleration[0], acceleration[1], acceleration[2],
                          gyro[0], gyro[1], gyro[2],
                          temperature);

        res = f_write(&file, buffer, len, &bw);
        if (res != FR_OK){
            printf("[ERRO] Falha ao escrever no arquivo.\n");
            break;
        }

        f_sync(&file);  // opcional: garante gravação a cada iteração
        sleep_ms(100);
    }

    f_close(&file);
    printf("\nDados %s no arquivo %s.\n\n",
           stop_capture ? "parciais salvos" : "completos salvos",
           filename);

    stop_capture = false;  // reset para próxima captura
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename){
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK){
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");

        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0){
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

static void run_help(void){
    printf("\nComandos disponíveis:\n\n");
    printf("Digite '1' para montar o cartão SD\n");
    printf("Digite '2' para desmontar o cartão SD\n");
    printf("Digite '3' para listar arquivos\n");
    printf("Digite '4' para mostrar conteúdo do arquivo\n");
    printf("Digite '5' para obter espaço livre no cartão SD\n");
    printf("Digite '6' para capturar dados e salvar no arquivo\n");
    printf("Digite '7' para formatar o cartão SD\n");
    printf("Digite '8' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}


static void process_stdio(int cRxedChar){
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r'){
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd)){
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn){
            size_t i;
            for (i = 0; i < count_of(cmds); ++i){
                if (0 == strcmp(cmds[i].command, cmdn)){
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i) printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    } else {
        if (cRxedChar == '\b' || cRxedChar == (char)127){
            if (ix > 0){
                ix--;
                cmd[ix] = '\0';
            }
        } else{
            if (ix < sizeof cmd - 1){
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}
    
static void mpu6050_reset(void){
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_port, addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(i2c_port, addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp){
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_port, addr, &val, 1, true);
    i2c_read_blocking(i2c_port, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x43;
    i2c_write_blocking(i2c_port, addr, &val, 1, true);
    i2c_read_blocking(i2c_port, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    val = 0x41;
    i2c_write_blocking(i2c_port, addr, &val, 1, true);
    i2c_read_blocking(i2c_port, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}