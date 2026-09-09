#include "transmit.h"
#include <errno.h>
#include <math.h>

extern DMA_HandleTypeDef hdma_usart2_rx;
extern osMessageQueueId_t Queue_communicationHandle;
static uint8_t rx1[UART_FRAME_SIZE], rx2[UART_FRAME_SIZE];
static char line1[UART_FRAME_SIZE], line2[UART_FRAME_SIZE];
static unsigned used1, used2;
static int discard1, discard2;

int _write(int file, char *data, int len) {
    (void)file;
    return HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100) == HAL_OK ? len : -1;
}

void transmit_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(HUART, rx1, sizeof(rx1));
    __HAL_DMA_DISABLE_IT(HDMA_HUART_RX, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2, sizeof(rx2));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/* Assemble CR/LF-delimited frames across DMA callbacks. Two UARTs never
 * share buffers. Discard overlong lines through their next terminator. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    uint8_t *rx;
    char *line;
    unsigned *used;
    int *discard;
    DMA_HandleTypeDef *dma;
    if (huart == HUART) {
        rx = rx1; line = line1; used = &used1; discard = &discard1; dma = HDMA_HUART_RX;
    } else if (huart == &huart2) {
        rx = rx2; line = line2; used = &used2; discard = &discard2; dma = &hdma_usart2_rx;
    } else return;
    BaseType_t wake = pdFALSE;
    if (size > UART_FRAME_SIZE) size = UART_FRAME_SIZE;
    for (unsigned i = 0; i < size; ++i) {
        char c = (char)rx[i];
        if (c == '\r' || c == '\n') {
            if (!*discard && *used > 0) {
                line[*used] = '\0';
                xQueueSendToBackFromISR(Queue_communicationHandle, line, &wake);
            }
            *used = 0;
            *discard = 0;
        } else if ((unsigned char)c < 32 || (unsigned char)c > 126) {
            *discard = 1;
        } else if (!*discard) {
            if (*used < UART_FRAME_SIZE - 1) line[(*used)++] = c;
            else *discard = 1;
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx, UART_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    portYIELD_FROM_ISR(wake);
}

/* Invalid tokens/extra parameters invalidate the whole command. */
CommandResult parse_command(const char *input) {
    CommandResult res = {0};
    if (!input) return res;
    while (isspace((unsigned char)*input)) ++input;
    unsigned length = 0;
    while (*input && !isspace((unsigned char)*input)) {
        if (length >= sizeof(res.command) - 1) return (CommandResult){0};
        res.command[length++] = (char)toupper((unsigned char)*input++);
    }
    if (!length) return res;
    while (*input) {
        while (isspace((unsigned char)*input)) ++input;
        if (!*input) break;
        if (res.param_count == 6) return (CommandResult){0};
        char *end;
        errno = 0;
        double value = strtod(input, &end);
        if (end == input || errno == ERANGE || !isfinite(value)
                || (*end && !isspace((unsigned char)*end))) return (CommandResult){0};
        res.params[res.param_count++] = value;
        input = end;
    }
    return res;
}
