#include "app.h"
#include "main.h"
#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_uart.h"
#include <string.h>

extern "C" {
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
}

void can_start() {
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = 0;                        // フィルターID(上位16ビット)
    filter.FilterIdLow = 0;                         // フィルターID(下位16ビット)
    filter.FilterMaskIdHigh = 0;                    // フィルターマスク(上位16ビット)
    filter.FilterMaskIdLow = 0;                     // フィルターマスク(下位16ビット)
    filter.FilterScale = CAN_FILTERSCALE_32BIT;     // フィルタースケール
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // フィルターに割り当てるFIFO
    filter.FilterBank = 0;                          // フィルターバンクNo
    filter.FilterMode = CAN_FILTERMODE_IDMASK;      // フィルターモード
    filter.SlaveStartFilterBank = 14;               // スレーブCANの開始フィルターバンクNo
    filter.FilterActivation = ENABLE;               // フィルター無効／有効

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    // CANスタート
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
    // 割り込み有効
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}
void transmit_can_message(uint32_t id, uint8_t data[8]) {
    // 送信用インスタンス等
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    // 送信メールボックスに空きがあったら送信開始
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) {
        // 送信用インスタンスの設定
        tx_header.StdId = id; // 受取手のCANのID
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.IDE = CAN_ID_STD;
        tx_header.DLC = 8; // データ長を8byteに設定
        tx_header.TransmitGlobalTime = DISABLE;
        // 各データ
        tx_data[0] = data[0];
        tx_data[1] = data[1];
        tx_data[2] = data[2];
        tx_data[3] = data[3];
        tx_data[4] = data[4];
        tx_data[5] = data[5];
        tx_data[6] = data[6];
        tx_data[7] = data[7];
        // CANメッセージを送信
        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
            Error_Handler();
        }
    }
}
bool can_received = false;
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_received = true;
}
struct CanRxMessage {
    uint32_t id;
    uint32_t dlc;
    uint8_t data[8];
};
CanRxMessage read_can() {
    if (!can_received)
        return {};
    can_received = false;

    CanRxMessage msg;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        msg.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId; // ID
        msg.dlc = rx_header.DLC;                                                    // DLC
        msg.data[0] = rx_data[0];                                                   // Data
        msg.data[1] = rx_data[1];
        msg.data[2] = rx_data[2];
        msg.data[3] = rx_data[3];
        msg.data[4] = rx_data[4];
        msg.data[5] = rx_data[5];
        msg.data[6] = rx_data[6];
        msg.data[7] = rx_data[7];
    }
    return msg;
}

uint8_t uart_rx_buf[256];   // UART受信バッファ
volatile uint16_t head = 0; // 書き込み位置（割り込み側）
volatile uint16_t tail = 0; // 読み込み位置（メイン側）
uint8_t uart_rx;            // 現在の受信データ（割り込み処理中に更新される）
void uart_start() {
    HAL_UART_Receive_IT(&huart2, &uart_rx, 1); // UART受信割り込みを有効化
}
int uart_available(void) {
    return head != tail;
}
uint8_t uart_read(void) {
    uint8_t data = uart_rx_buf[tail];
    tail = (tail + 1) % 256;
    return data;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uint16_t next = (head + 1) % 256;

        // バッファ満杯でなければ書く
        if (next != tail) {
            uart_rx_buf[head] = uart_rx;
            head = next;
        }

        // 次の受信を再開
        HAL_UART_Receive_IT(&huart2, &uart_rx, 1);
    }
}

extern "C" void setup() {
    can_start();
    uart_start();
}

extern "C" void loop() {
    if (can_received) {
        CanRxMessage msg = read_can();
        transmit_can_message(msg.id, msg.data);
    }
    if (uart_available()) {
        uint8_t data[256];
        while (uart_available()) {
            static uint8_t i = 0;
            data[i] = uart_read();
            i++;
        }
        HAL_UART_Transmit(
            &huart2,
            data,
            strlen((char *)data),
            HAL_MAX_DELAY);
    }
}
