
#include "interface_usb.h"
#include "ascii_protocol.hpp"

#include <MotorControl/utils.h>

#include <fibre/protocol.hpp>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <usbd_def.h>
#include <usb_device.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

#include <odrive_main.h>

static uint8_t active_endpoint_pair;

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
static thread_local uint32_t deadline_ms = 0;

static const uint32_t usb_check_timeout = 1; // ms

struct USBData_t {
    uint8_t* buffer;
    uint32_t len;
    bool data_is_pending;
} odriveData, cdcData;

osThreadId usb_thread;

USBStats_t usb_stats_ = {0};

class USBSender : public PacketSink {
public:
    int process_packet(const uint8_t* buffer, size_t length) {
        // cannot send partial packets
        if (length > USB_TX_DATA_SIZE)
            return -1;
        // wait for USB interface to become ready
        if (osSemaphoreWait(sem_usb_tx, deadline_to_timeout(deadline_ms)) != osOK) {
            // If the host resets the device it might be that the TX-complete handler is never called
            // and the sem_usb_tx semaphore is never released. To handle this we just override the
            // TX buffer if this wait times out. The implication is that the channel is no longer lossless.
            // TODO: handle endpoint reset properly
            usb_stats_.tx_overrun_cnt++;
        }
        // transmit packet
        uint8_t status = CDC_Transmit_FS(
                const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                well... it's not actually. Stupid STM. */, length, active_endpoint_pair);
        if (status != USBD_OK) {
            osSemaphoreRelease(sem_usb_tx);
            return -1;
        }
        usb_stats_.tx_cnt++;
        return 0;
    }
} usb_packet_output;

class TreatPacketSinkAsStreamSink : public StreamSink {
public:
    TreatPacketSinkAsStreamSink(PacketSink& output) : output_(output) {}
    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            if (output_.process_packet(buffer, length) != 0)
                return -1;
            buffer += chunk;
            length -= chunk;
            if (processed_bytes)
                *processed_bytes += chunk;
        }
        return 0;
    }
    size_t get_free_space() { return SIZE_MAX; }
private:
    PacketSink& output_;
} usb_stream_output(usb_packet_output);
StreamSink* usb_stream_output_ptr = &usb_stream_output;

#if defined(USB_PROTOCOL_NATIVE)
BidirectionalPacketBasedChannel usb_channel(usb_packet_output);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
StreamBasedPacketSink usb_packetized_output(usb_stream_output);
BidirectionalPacketBasedChannel usb_channel(usb_packetized_output);
StreamToPacketSegmenter usb_native_stream_input(usb_channel);
#endif

static void usb_server_thread(void * ctx) {
    (void) ctx;
    
    for (;;) {
        osStatus sem_stat = osSemaphoreWait(sem_usb_rx, usb_check_timeout);
        if (sem_stat == osOK) {
            usb_stats_.rx_cnt++;
            deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
            if (odriveData.data_is_pending) {
                odriveData.data_is_pending = false;
                // Ewwww. Doing this for process_packet to know what endpoint to write to - for now.
                active_endpoint_pair = ODRIVE_OUT_EP;
                printf("Processing packet on endpoint %u and readying for next receipt..\r\n", ODRIVE_OUT_EP);
#if defined(USB_PROTOCOL_NATIVE)
                usb_channel.process_packet(odriveData.buffer, odriveData.len);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
                usb_native_stream_input.process_bytes(odriveData.buffer, odriveData.len, nullptr);
#endif
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, ODRIVE_OUT_EP);  // Allow next packet
            }
            if (cdcData.data_is_pending && board_config.enable_ascii_protocol_on_usb) {
                cdcData.data_is_pending = false;
                // Ewwww. Doing this for process_packet to know what endpoint to write to - for now.
                active_endpoint_pair = CDC_OUT_EP;
                printf("Processing packet on endpoint %u and readying for next receipt..\r\n", CDC_OUT_EP);
                ASCII_protocol_parse_stream(cdcData.buffer, cdcData.len, usb_stream_output);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, CDC_OUT_EP);  // Allow next packet
            }
        }
    }
}

// Called from CDC_Receive_FS callback function, this allows the communication
// thread to handle the incoming data
void usb_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair) {
    if (endpoint_pair == ODRIVE_OUT_EP) {
        printf("Received packet on endpoint %u\r\n", endpoint_pair);
        odriveData.buffer = buf;
        odriveData.len = len;
        odriveData.data_is_pending = true;
    } else {
        printf("Received packet on endpoint %u\r\n", endpoint_pair);
        cdcData.buffer = buf;
        cdcData.len = len;
        cdcData.data_is_pending = true;
        
    }
    osSemaphoreRelease(sem_usb_rx);
}

void start_usb_server() {
    // Start USB communication thread
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0, 512);
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), NULL);
}
