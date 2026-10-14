/*
  virtual serial port the GCS attaches to, fed by the LTE driver thread
 */

#include "AP_LTE.h"

#if AP_LTE_ENABLED

#include <AP_Math/AP_Math.h>

bool AP_LTE_Port::init_buffers(const uint32_t size_rx, const uint32_t size_tx)
{
    WITH_SEMAPHORE(sem);
    if (readbuffer == nullptr) {
        readbuffer = NEW_NOTHROW ByteBuffer(size_rx);
    }
    if (writebuffer == nullptr) {
        writebuffer = NEW_NOTHROW ByteBuffer(size_tx);
    }
    return readbuffer != nullptr && writebuffer != nullptr;
}

// ---- device side: the driver thread ----------------------------------------

// downlink, modem -> vehicle. Returns what the buffer could take.
size_t AP_LTE_Port::device_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->write(buffer, size) : 0;
}

// uplink, vehicle -> modem
ssize_t AP_LTE_Port::device_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->read(buffer, count) : 0;
}

uint32_t AP_LTE_Port::device_available()
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->available() : 0;
}

void AP_LTE_Port::device_clear()
{
    WITH_SEMAPHORE(sem);
    if (readbuffer != nullptr) {
        readbuffer->clear();
    }
    if (writebuffer != nullptr) {
        writebuffer->clear();
    }
}

// ---- UARTDriver side: the GCS backend --------------------------------------

uint32_t AP_LTE_Port::txspace()
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->space() : 0;
}

void AP_LTE_Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    init_buffers(MAX(rxS, AP_LTE_VPORT_RXBUF), MAX(txS, AP_LTE_VPORT_TXBUF));
}

size_t AP_LTE_Port::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(sem);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

ssize_t AP_LTE_Port::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t AP_LTE_Port::_available()
{
    WITH_SEMAPHORE(sem);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}

bool AP_LTE_Port::_discard_input()
{
    WITH_SEMAPHORE(sem);
    if (readbuffer != nullptr) {
        readbuffer->clear();
    }
    return true;
}

#endif  // AP_LTE_ENABLED
