# SD Card Data Logger para Raspberry Pi Pico W

## Descrição do Projeto

Sistema embarcado em C para Raspberry Pi Pico W (RP2040) que: monta/desmonta cartão SD, lista diretórios, formata, captura dados de sensor IMU (MPU6050) em CSV, e exibe informações em OLED.

## Objetivo Geral

Implementar interface de linha de comando via UART para operações em cartão SD e captura de dados do MPU6050, com feedback visual (LEDs), sonoro (buzzer) e display OLED.

## Descrição Funcional

* **Inicialização**: configura I2C (sensor e display), PWM (buzzer), GPIO (botões e LEDs), RTC e display OLED em segunda core.
* **Comandos UART** (teclas '1' a '8'):
* **Gráficos**
Dentro da pasta ArquivoDados há um código em python que permite visualizar os arquivos em gráficos. Basta só dar o comando python plotadados.py e ele ativa. Se quiser definir qual arquivo apresentar o gráfico, é necessário alterar o nome no código,  na linha 45.

  1. **Montar SD**: `mount` monta o cartão.
  2. **Desmontar SD**: `unmount` desmonta o cartão.
  3. **Listar**: `ls` exibe arquivos e diretórios.
  4. **Exibir arquivo**: `cat <filename>` mostra conteúdo.
  5. **Espaço livre**: `getfree` informa KB total e disponível.
  6. **Capturar dados**: gera nome único, lê 128 amostras do MPU6050 e grava `log_NNN.csv` (inclui cabeçalho `id,ax,ay,az,gx,gy,gz,temp`).
  7. **Formatar**: `format` formata cartão SD.
  8. **Ajuda**: `help` exibe menu.
* **Botões físicos**:

  * **Botão A**: inicia/parar captura de dados (interrupção GPIO).
  * **Botão B**: monta/desmonta SD (interrupção GPIO).
* **Display OLED**: exibe menu padrão ou status de operação via `display()` na core secundária.
* **LEDs**: verdes/vermelho/azul indicam status de operação.
* **Buzzer**: bipes para confirmação, usando PWM com padrões configuráveis.
* **RTC**: usado para timestamp opcional (comando `setrtc DD MM YY hh mm ss`).

## Detalhes dos Periféricos

| Periférico         | Uso                                               |
| ------------------ | ------------------------------------------------- |
| I2C (i2c0)         | MPU6050 (addr 0x68)                               |
| I2C (i2c1)         | SSD1306 OLED (GPIO14 SDA, GPIO15 SCL)             |
| SPI PIO            |                                                   |
| GPIO Botões A/B    | GPIO5, GPIO6 com pull-up e IRQ (falling)          |
| GPIO LEDs          | GPIO11 (verde), 12 (azul), 13 (vermelho)          |
| GPIO Buzzer        | GPIO21 via PWM slice configurado em `pwm_setup()` |
| SD Card            | Interface SPI via lib `sd_card.h` e FatFS         |
| UART Serial        | `stdio_init_all()` para CLI                       |

## Instalação e Execução

### Pré-requisitos

* Raspberry Pi Pico W conectado via USB.
* Raspberry Pi Pico SDK, CMake e toolchain instalados.

### Configuração

1. Clone ou copie o código fonte para seu workspace.
2. Verifique dependências: `hw_config.h`, `sd_card.h`, `ff.h`, `lib/ssd1306.h`, `font.h`, `rtc.h`, `my_debug.h`.

### Compilação

```bash
mkdir build && cd build
cmake ..
make
```

### Deploy

1. Segure BOOTSEL e conecte o Pico ao PC.
2. Copie `your_project.uf2` para o drive do Pico.
3. O dispositivo reiniciará e estará pronto para uso.

## Autor

Hugo Martins Santana (TIC370101267)
