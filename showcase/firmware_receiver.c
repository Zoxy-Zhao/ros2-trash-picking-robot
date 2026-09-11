/* Host-only adapter around the real transmit.c DMA framing and parser.
 * No FreeRTOS motor task, PWM, UART device or hardware acknowledgement. */
#include "transmit.h"

UART_HandleTypeDef huart1 = {1}, huart2 = {2};
DMA_HandleTypeDef hdma_usart1_rx, hdma_usart2_rx;
osMessageQueueId_t Queue_communicationHandle;
static uint8_t *rx[3];
static int invalid;

int HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *p, int n, int t) {
    (void)u; (void)p; (void)n; (void)t; return HAL_OK;
}
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, unsigned size) {
    (void)size; rx[u->id] = p; return HAL_OK;
}
int xQueueSendToBackFromISR(void *queue, const void *p, BaseType_t *wake) {
    (void)queue; *wake = 0;
    CommandResult c = parse_command((const char *)p);
    int valid = (!strcmp(c.command, "ARM6") && c.param_count == 6) ||
                (!strcmp(c.command, "CLAW") && c.param_count == 1 && (c.params[0] == 0 || c.params[0] == 1));
    if (!valid) { invalid = 1; return 0; }
    printf("{\"op\":\"%s\",\"params\":[", c.command);
    for (int i=0; i<c.param_count; ++i) printf("%s%.12g", i ? "," : "", c.params[i]);
    puts("]}");
    return 1;
}
int main(void) {
    transmit_Init();
    unsigned size = 0;
    int byte;
    /* Intentionally fragment the stream into seven-byte DMA events. */
    while ((byte = getchar()) != EOF) {
        rx[1][size++] = (uint8_t)byte;
        if (size == 7) { HAL_UARTEx_RxEventCallback(&huart1, (uint16_t)size); size = 0; }
    }
    if (size) HAL_UARTEx_RxEventCallback(&huart1, (uint16_t)size);
    return invalid ? 2 : 0;
}
