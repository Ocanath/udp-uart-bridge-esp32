#pragma once
#include <stddef.h>
#include <stdint.h>

// Hardware pins
#define NRST_PIN   1
#define LED_PIN    21
#define BOOT_PIN   9

// UART2
#define UART2_RX_PIN     4
#define UART2_TX_PIN     5
#define DEFAULT_BAUDRATE 2000000

// Network
#define IPV4_ADDR_ANY  0x00000000UL

// LED blink periods (ms)
enum { PERIOD_CONNECTED = 50, PERIOD_DISCONNECTED = 1000 };

// I2S microphone GPIO numbers (plain integers — no IDF enum dependency)
#define SAMPLE_RATE               16000
#define I2S_MIC_SERIAL_CLOCK      14
#define I2S_MIC_LEFT_RIGHT_CLOCK  13
#define I2S_MIC_SERIAL_DATA       12

// I2S audio frame
#define NUM_BYTES_HEADER        6
#define NUM_BYTES_SAMPLE_BUFFER (512 * sizeof(int32_t))
#define AUDIO_UDP_PORT          6767

// Forwarding buffer sizes
#define PAYLOAD_BUFFER_SIZE  128
#define UDP_PKT_BUF_SIZE     256

// Buffer type for null-terminated UART framing
typedef struct buffer_t
{
    unsigned char *buf;
    size_t         size;
    size_t         len;
} buffer_t;
