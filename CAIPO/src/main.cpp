#include <Arduino.h>
#include <Update.h>

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

// ====================================================
// App Info
// ====================================================

#define APP_VERSION "1.0.0"

// ====================================================
// Firmware Update Protocol Constants
// ====================================================

#define FWUP_SOF_VALUE             0x31505546UL   // "FUP1" little-endian on wire
#define FWUP_PROTOCOL_VERSION      1
#define FWUP_MAX_PAYLOAD_SIZE      1024

// Packet types
#define FWUP_PACKET_HELLO          0x01
#define FWUP_PACKET_START          0x02
#define FWUP_PACKET_DATA           0x03
#define FWUP_PACKET_END            0x04
#define FWUP_PACKET_ABORT          0x05
#define FWUP_PACKET_REBOOT         0x06
#define FWUP_PACKET_ROLLBACK       0x07

#define FWUP_PACKET_ACK            0x80
#define FWUP_PACKET_HELLO_RESPONSE 0x81
#define FWUP_PACKET_ERROR          0xFF

// ====================================================
// Protocol Structures
// ====================================================

struct __attribute__((packed)) FwupPacketHeader
{
    uint32_t sof;
    uint8_t  protocol_version;
    uint8_t  packet_type;
    uint16_t flags;
    uint32_t sequence;
    uint32_t payload_length;
    uint32_t payload_crc32;
};

struct __attribute__((packed)) FwupStartRequest
{
    uint32_t image_size;
    uint32_t image_crc32;
};

struct __attribute__((packed)) FwupAckResponse
{
    int32_t  status;
    uint32_t detail;
};

struct __attribute__((packed)) FwupHelloResponse
{
    uint32_t protocol_version;
    uint32_t max_payload_size;
    uint32_t running_partition_subtype;
    uint32_t rollback_state;
    char     app_version[32];
};

// ====================================================
// OTA State
// ====================================================

static bool ota_session_active = false;

static uint32_t expected_image_size = 0;
static uint32_t expected_image_crc32 = 0;
static uint32_t received_image_size = 0;
static uint32_t calculated_image_crc32 = 0;

static uint8_t payload_buffer[FWUP_MAX_PAYLOAD_SIZE];

// ====================================================
// CRC32
// Python zlib.crc32 ile uyumlu.
// ====================================================

static uint32_t fwup_crc32_update(uint32_t previous_crc, const uint8_t *data, uint32_t length)
{
    uint32_t crc = previous_crc ^ 0xFFFFFFFFUL;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint32_t mask = -(crc & 1UL);
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

// ====================================================
// Serial Helpers
// ====================================================

static bool serial_read_exact(uint8_t *buffer, uint32_t length, uint32_t timeout_ms)
{
    uint32_t received = 0;
    uint32_t start_time = millis();

    while (received < length)
    {
        while (Serial.available() > 0 && received < length)
        {
            int byte_value = Serial.read();

            if (byte_value >= 0)
            {
                buffer[received++] = (uint8_t)byte_value;
            }
        }

        if ((millis() - start_time) > timeout_ms)
        {
            return false;
        }

        delay(1);
    }

    return true;
}

static void serial_write_exact(const uint8_t *buffer, uint32_t length)
{
    Serial.write(buffer, length);
    Serial.flush();
}

// ====================================================
// Packet TX
// ====================================================

static void fwup_send_packet(uint8_t packet_type,
                             uint32_t sequence,
                             const void *payload,
                             uint32_t payload_length)
{
    FwupPacketHeader header;

    header.sof = FWUP_SOF_VALUE;
    header.protocol_version = FWUP_PROTOCOL_VERSION;
    header.packet_type = packet_type;
    header.flags = 0;
    header.sequence = sequence;
    header.payload_length = payload_length;
    header.payload_crc32 = 0;

    if (payload != nullptr && payload_length > 0)
    {
        header.payload_crc32 = fwup_crc32_update(
            0,
            (const uint8_t *)payload,
            payload_length
        );
    }

    serial_write_exact((const uint8_t *)&header, sizeof(header));

    if (payload != nullptr && payload_length > 0)
    {
        serial_write_exact((const uint8_t *)payload, payload_length);
    }
}

static void fwup_send_ack(uint32_t sequence, int32_t status, uint32_t detail)
{
    FwupAckResponse response;

    response.status = status;
    response.detail = detail;

    fwup_send_packet(
        FWUP_PACKET_ACK,
        sequence,
        &response,
        sizeof(response)
    );
}

static void fwup_send_error(uint32_t sequence, int32_t status, uint32_t detail)
{
    FwupAckResponse response;

    response.status = status;
    response.detail = detail;

    fwup_send_packet(
        FWUP_PACKET_ERROR,
        sequence,
        &response,
        sizeof(response)
    );
}

// ====================================================
// Packet RX
// SOF arayarak senkron olur.
// Böylece boot log veya Serial print varsa parser bozulmaz.
// ====================================================

static bool fwup_read_packet(FwupPacketHeader *header, uint8_t *payload)
{
    static uint8_t sof_window[4] = {0};

    const uint8_t expected_sof[4] = {
        (uint8_t)(FWUP_SOF_VALUE & 0xFF),
        (uint8_t)((FWUP_SOF_VALUE >> 8) & 0xFF),
        (uint8_t)((FWUP_SOF_VALUE >> 16) & 0xFF),
        (uint8_t)((FWUP_SOF_VALUE >> 24) & 0xFF)
    };

    while (Serial.available() > 0)
    {
        int byte_value = Serial.read();

        if (byte_value < 0)
        {
            return false;
        }

        sof_window[0] = sof_window[1];
        sof_window[1] = sof_window[2];
        sof_window[2] = sof_window[3];
        sof_window[3] = (uint8_t)byte_value;

        if (memcmp(sof_window, expected_sof, sizeof(expected_sof)) == 0)
        {
            header->sof = FWUP_SOF_VALUE;

            uint8_t *header_bytes = (uint8_t *)header;

            bool header_ok = serial_read_exact(
                header_bytes + 4,
                sizeof(FwupPacketHeader) - 4,
                3000
            );

            if (!header_ok)
            {
                return false;
            }

            if (header->protocol_version != FWUP_PROTOCOL_VERSION)
            {
                fwup_send_error(header->sequence, -100, header->protocol_version);
                return false;
            }

            if (header->payload_length > FWUP_MAX_PAYLOAD_SIZE)
            {
                fwup_send_error(header->sequence, -101, header->payload_length);
                return false;
            }

            if (header->payload_length > 0)
            {
                bool payload_ok = serial_read_exact(
                    payload,
                    header->payload_length,
                    3000
                );

                if (!payload_ok)
                {
                    fwup_send_error(header->sequence, -102, header->payload_length);
                    return false;
                }

                uint32_t calculated_payload_crc32 = fwup_crc32_update(
                    0,
                    payload,
                    header->payload_length
                );

                if (calculated_payload_crc32 != header->payload_crc32)
                {
                    fwup_send_error(
                        header->sequence,
                        -103,
                        calculated_payload_crc32
                    );

                    return false;
                }
            }

            return true;
        }
    }

    return false;
}

// ====================================================
// OTA / Rollback Helpers
// ====================================================

static uint32_t fwup_get_running_partition_subtype()
{
    const esp_partition_t *running_partition = esp_ota_get_running_partition();

    if (running_partition == nullptr)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)running_partition->subtype;
}

static uint32_t fwup_get_rollback_state()
{
    const esp_partition_t *running_partition = esp_ota_get_running_partition();

    if (running_partition == nullptr)
    {
        return 0xFFFFFFFFUL;
    }

    esp_ota_img_states_t ota_state;

    esp_err_t result = esp_ota_get_state_partition(
        running_partition,
        &ota_state
    );

    if (result != ESP_OK)
    {
        return 0xFFFFFFFEUL;
    }

    return (uint32_t)ota_state;
}

static void fwup_confirm_app_if_pending()
{
    const esp_partition_t *running_partition = esp_ota_get_running_partition();

    if (running_partition == nullptr)
    {
        return;
    }

    esp_ota_img_states_t ota_state;

    esp_err_t result = esp_ota_get_state_partition(
        running_partition,
        &ota_state
    );

    if (result != ESP_OK)
    {
        return;
    }

    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
    {
        /*
            Buraya ileride gerçek self-test koyacağız.

            Örneğin:
            - USB update task başladı mı?
            - BLE update service başladı mı?
            - NVS okunabiliyor mu?
            - Ana uygulama taskları crash olmadan başladı mı?

            Şimdilik setup() çalıştıysa ve Serial başladıysa app'i valid kabul ediyoruz.
        */

        esp_ota_mark_app_valid_cancel_rollback();
    }
}

// ====================================================
// Packet Handlers
// ====================================================

static void fwup_handle_hello(const FwupPacketHeader *header)
{
    FwupHelloResponse response;
    memset(&response, 0, sizeof(response));

    response.protocol_version = FWUP_PROTOCOL_VERSION;
    response.max_payload_size = FWUP_MAX_PAYLOAD_SIZE;
    response.running_partition_subtype = fwup_get_running_partition_subtype();
    response.rollback_state = fwup_get_rollback_state();

    strncpy(response.app_version, APP_VERSION, sizeof(response.app_version) - 1);

    fwup_send_packet(
        FWUP_PACKET_HELLO_RESPONSE,
        header->sequence,
        &response,
        sizeof(response)
    );
}

static void fwup_handle_start(const FwupPacketHeader *header, const uint8_t *payload)
{
    if (header->payload_length != sizeof(FwupStartRequest))
    {
        fwup_send_error(header->sequence, -1, header->payload_length);
        return;
    }

    if (ota_session_active)
    {
        Update.abort();
        ota_session_active = false;
    }

    FwupStartRequest request;
    memcpy(&request, payload, sizeof(request));

    expected_image_size = request.image_size;
    expected_image_crc32 = request.image_crc32;
    received_image_size = 0;
    calculated_image_crc32 = 0;

    if (expected_image_size == 0)
    {
        fwup_send_error(header->sequence, -2, 0);
        return;
    }

    bool update_started = Update.begin(expected_image_size, U_FLASH);

    if (!update_started)
    {
        fwup_send_error(header->sequence, -3, Update.getError());
        return;
    }

    ota_session_active = true;

    fwup_send_ack(header->sequence, 0, 0);
}

static void fwup_handle_data(const FwupPacketHeader *header, const uint8_t *payload)
{
    if (!ota_session_active)
    {
        fwup_send_error(header->sequence, -10, 0);
        return;
    }

    if (header->payload_length == 0)
    {
        fwup_send_error(header->sequence, -11, 0);
        return;
    }

    if ((received_image_size + header->payload_length) > expected_image_size)
    {
        fwup_send_error(header->sequence, -12, received_image_size);
        return;
    }

    size_t written = Update.write(
        (uint8_t *)payload,
        header->payload_length
    );

    if (written != header->payload_length)
    {
        fwup_send_error(header->sequence, -13, Update.getError());
        return;
    }

    calculated_image_crc32 = fwup_crc32_update(
        calculated_image_crc32,
        payload,
        header->payload_length
    );

    received_image_size += header->payload_length;

    fwup_send_ack(header->sequence, 0, received_image_size);
}

static void fwup_handle_end(const FwupPacketHeader *header)
{
    if (!ota_session_active)
    {
        fwup_send_error(header->sequence, -20, 0);
        return;
    }

    if (received_image_size != expected_image_size)
    {
        Update.abort();
        ota_session_active = false;

        fwup_send_error(header->sequence, -21, received_image_size);
        return;
    }

    if (calculated_image_crc32 != expected_image_crc32)
    {
        Update.abort();
        ota_session_active = false;

        fwup_send_error(header->sequence, -22, calculated_image_crc32);
        return;
    }

    bool update_finished = Update.end(true);

    if (!update_finished)
    {
        ota_session_active = false;

        fwup_send_error(header->sequence, -23, Update.getError());
        return;
    }

    ota_session_active = false;

    fwup_send_ack(header->sequence, 0, received_image_size);

    delay(500);
    ESP.restart();
}

static void fwup_handle_abort(const FwupPacketHeader *header)
{
    if (ota_session_active)
    {
        Update.abort();
        ota_session_active = false;
    }

    fwup_send_ack(header->sequence, 0, 0);
}

static void fwup_handle_reboot(const FwupPacketHeader *header)
{
    fwup_send_ack(header->sequence, 0, 0);

    delay(300);
    ESP.restart();
}

static void fwup_handle_rollback(const FwupPacketHeader *header)
{
    esp_err_t result = esp_ota_mark_app_invalid_rollback_and_reboot();

    if (result != ESP_OK)
    {
        fwup_send_error(header->sequence, result, 0);
        return;
    }

    // Başarılı olursa zaten reboot eder.
}

// ====================================================
// Main Firmware Update Loop
// ====================================================

static void fwup_process()
{
    FwupPacketHeader header;

    bool packet_available = fwup_read_packet(&header, payload_buffer);

    if (!packet_available)
    {
        return;
    }

    switch (header.packet_type)
    {
        case FWUP_PACKET_HELLO:
            fwup_handle_hello(&header);
            break;

        case FWUP_PACKET_START:
            fwup_handle_start(&header, payload_buffer);
            break;

        case FWUP_PACKET_DATA:
            fwup_handle_data(&header, payload_buffer);
            break;

        case FWUP_PACKET_END:
            fwup_handle_end(&header);
            break;

        case FWUP_PACKET_ABORT:
            fwup_handle_abort(&header);
            break;

        case FWUP_PACKET_REBOOT:
            fwup_handle_reboot(&header);
            break;

        case FWUP_PACKET_ROLLBACK:
            fwup_handle_rollback(&header);
            break;

        default:
            fwup_send_error(header.sequence, -99, header.packet_type);
            break;
    }
}

// ====================================================
// Arduino Setup / Loop
// ====================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    fwup_confirm_app_if_pending();

    Serial.println();
    Serial.println("XIAO ESP32S3 USB OTA ready");
    Serial.print("App version: ");
    Serial.println(APP_VERSION);
    Serial.print("Running partition subtype: 0x");
    Serial.println(fwup_get_running_partition_subtype(), HEX);
    Serial.print("Rollback state: ");
    Serial.println(fwup_get_rollback_state());
}

void loop()
{
    fwup_process();

    /*
        Normal uygulama kodun burada çalışmaya devam edebilir.

        Dikkat:
        Firmware update sırasında aynı Serial portuna sürekli text basmak
        protokolü kirletebilir. Parser SOF aradığı için dayanıklı ama yine de
        production'da Serial debug'u kapatmak daha iyi.
    */

    delay(1);
}