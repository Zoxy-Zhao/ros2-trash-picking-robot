#include "transmit.h"
#include "arm6_config.h"
#include <assert.h>

UART_HandleTypeDef huart1 = {1}, huart2 = {2};
DMA_HandleTypeDef hdma_usart1_rx, hdma_usart2_rx;
osMessageQueueId_t Queue_communicationHandle;
static uint8_t *rx[3];
static char received[16][64];
static int count;

int HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *p, int n, int t) {
    (void)u; (void)p; (void)n; (void)t; return HAL_OK;
}
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, unsigned size) {
    assert(size == 64); rx[u->id] = p; return HAL_OK;
}
int xQueueSendToBackFromISR(void *queue, const void *p, BaseType_t *wake) {
    (void)queue; assert(wake != NULL); *wake = 1;
    assert(count < 16); memcpy(received[count++], p, 64); return 1;
}
static void feed(UART_HandleTypeDef *u, const char *text) {
    size_t length = strlen(text); assert(length <= 64);
    memcpy(rx[u->id], text, length);
    HAL_UARTEx_RxEventCallback(u, (uint16_t)length);
}
int main(void) {
    CommandResult command = parse_command(" ARM6 -12 23 34 -45 56 67\r\n");
    assert(!strcmp(command.command, "ARM6") && command.param_count == 6);
    assert(command.params[0] == -12 && command.params[5] == 67);
    const char *invalid[] = {"ARM6 1 2 3 4 5 6 7", "ARM6 1x", "ARM6 nan", "ARM6 inf", "ARM6 .", "ARM6 +", "ARM6 1e999"};
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(parse_command(invalid[i]).command[0] == 0);
    assert(!strcmp(parse_command("Claw 1").command, "CLAW"));
    assert(parse_command("   ").param_count == 0);
    transmit_Init();
    feed(&huart1, "ARM6 1 2");
    feed(&huart2, "CLAW 1\r\n");
    feed(&huart1, " 3 4 5 6\r\nSTOP\n");
    assert(count == 3);
    assert(!strcmp(received[0], "CLAW 1"));
    assert(!strcmp(received[1], "ARM6 1 2 3 4 5 6"));
    assert(!strcmp(received[2], "STOP"));
    char overflow[65]; memset(overflow, 'A', 64); overflow[64] = 0;
    feed(&huart1, overflow); feed(&huart1, "overflow\nCLAW 0\n");
    assert(count == 4 && !strcmp(received[3], "CLAW 0"));
    for (unsigned i = 0; i < 6; ++i) {
        assert(ARM6_CHANNELS[i] != 5 && ARM6_TRAVEL_DEG[i] > 0);
        for (unsigned j = i+1; j < 6; ++j) assert(ARM6_CHANNELS[i] != ARM6_CHANNELS[j]);
    }
    puts("Firmware parser, DMA framing and channel mapping checks passed");
    return 0;
}
