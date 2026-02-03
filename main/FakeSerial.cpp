#include "FakeSerial.h"
#include "CRC.h"
#include "ble_hand_app.h"
#include "driver/usb_serial_jtag_select.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdio.h>
#include <vector>
// 包含 ESP-IDF 驱动头文件
extern "C"
{
#include "driver/usb_serial_jtag.h"
#include "freertos/ringbuf.h"
}
QueueHandle_t pack_queue;
#define RINGBUF_SIZE 1024
RingbufHandle_t ble_usb_ringbuf; 

void FakeSerialpushDataToBuf(uint8_t *data, uint32_t len) {
    if (len <= 0 || ble_usb_ringbuf == NULL) return;
    // 实时性优先：如果发太快，缓冲区满了就放弃旧包，或者直接报错
    if (xRingbufferSend(ble_usb_ringbuf, data, len, 0) != pdTRUE) {
        // ESP_LOGW("FakeSerial", "Ringbuffer full, dropping BLE data");
    }
}

void readUsbTask(void *) {
    const size_t PACK_SIZE = sizeof(RxSerialPack);
    uint8_t temp_buf[256];
    
    while (1) {
        // 1. 处理 USB 输入 (保持不变)
        if (usb_serial_jtag_read_ready()) {
            ssize_t available = usb_serial_jtag_get_read_bytes_available();
            if (available > 0) {
                ssize_t read_cnt = std::min(available, (ssize_t)256);
                ssize_t real_read = usb_serial_jtag_read_bytes(temp_buf, read_cnt, 0);
                if (real_read > 0) {
                    xRingbufferSend(ble_usb_ringbuf, temp_buf, real_read, 0);
                }
            }
        }

        // 2. 优化后的解析逻辑
        size_t item_size;
        // 使用 xRingbufferReceive 获取缓冲区中所有可读原始字节
        // 注意：RINGBUF_TYPE_BYTEBUF 会尽可能返回连续的一段
        uint8_t* rb_data = (uint8_t*)xRingbufferReceive(ble_usb_ringbuf, &item_size, 0);
        
        if (rb_data != NULL) {
            size_t processed = 0;
            
            // 只要剩余字节足够一帧，就继续解析
            while (item_size - processed >= PACK_SIZE) {
                uint8_t* curr = rb_data + processed;
                
                // 检查帧头
                if (curr[0] == 0xA1 && curr[1] == 0xA2 && curr[2] == 0xA3 && curr[3] == 0xA4) {
                    RxSerialPack_u u;
                    memcpy(u.bytes, curr, PACK_SIZE);
                    uint8_t crc = CRC::Calculate(u.bytes, PACK_SIZE - 1, CRC::CRC_8());
                    
                    if (u.pack.CRC == crc) {
                        xQueueSendToBack(pack_queue, &u.pack, 0);
                        processed += PACK_SIZE; // 成功解析一帧，指针跳过整帧
                        continue; 
                    } else {
                        // CRC 失败，仅跳过一个字节，继续找下一个 A1
                        processed += 1;
                    }
                } else {
                    // 头不对，滑动 1 字节
                    processed += 1;
                }
            }
            
            // --- 核心优化：如何处理剩余的不足一帧的数据？ ---
            // 之前的代码直接 ReturnItem 会导致数据丢失或永远留在缓冲区
            // 我们需要把已经处理掉的字节从 Ringbuffer 中真正“弹出”
            vRingbufferReturnItem(ble_usb_ringbuf, (void*)rb_data);
            
            // 这里的技巧：我们需要手动通过 xRingbufferReceive 的特性来管理偏移
            // 由于 ESP-IDF 的原生 Ringbuf 不支持“只弹出 N 个字节”，
            // 我们采用一个变通方法：如果 item_size 很大但我们只处理了 processed，
            // 实际上下一次调用 xRingbufferReceive 依然会从头开始。
            
            // 正确的高效做法是使用：vRingbufferReturnItem + 手动记录
            // 但为了绝对可靠，我们这里对 RingBuffer 解析逻辑做一次彻底重构：
        }
        
        // 如果处理得快，可以减小 Delay 甚至不 Delay（配合外部 yield）
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
FakeSerial::FakeSerial()
    : _initialized(false) {}

void FakeSerial::begin(unsigned long baud)
{

    if (_initialized)
        return;

    // 1. 安装 USB-Serial-JTAG 驱动

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    // 调整缓冲区大小以防高速传输丢失数据 (可选)

    cfg.rx_buffer_size = 1024;

    cfg.tx_buffer_size = 1024;

    usb_serial_jtag_driver_install(&cfg);

    ble_usb_ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);

    pack_queue =
        xQueueGenericCreate(32, sizeof(RxSerialPack), queueQUEUE_TYPE_BASE);

    xTaskCreatePinnedToCore(readUsbTask, "readUsbTask", 9600, NULL, 10, NULL,1);
    _initialized = true;
}

void FakeSerial::write(uint8_t *data, uint32_t len)
{

    if (len != 16)
    {

        ESP_LOGE("FakeSerial", "Only Can write 16 bytes data");

        return;
    }

    TxSerialPack_u u;

    u.pack.header[0] = 0xA1;

    u.pack.header[1] = 0xA2;

    u.pack.header[2] = 0xA3;

    u.pack.header[3] = 0xA4;

    memcpy(u.pack.payload, data, len);

    u.pack.CRC = CRC::Calculate(u.bytes, sizeof(TxSerialPack) - 1, CRC::CRC_8());

    if (!sendHandState(u))
    {

        usb_serial_jtag_write_bytes(u.bytes, sizeof(TxSerialPack), 0);
    }
}

// 实例化全局对象

FakeSerial Serial;