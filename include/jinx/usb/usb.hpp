/*
Copyright (C) 2022  pom@vro.life

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef __jinx_usb_hpp__
#define __jinx_usb_hpp__

#include <poll.h>

#include <new>
#include <stdexcept>
#include <chrono>
#include <system_error>
#include <unordered_map>

#include <libusb-1.0/libusb.h>

#include <jinx/async.hpp>
#include <jinx/macros.hpp>
#include <jinx/raii.hpp>

namespace jinx {
namespace usb {

JINX_ERROR_DEFINE(usb, enum libusb_error);
JINX_ERROR_DEFINE(transfer, enum libusb_transfer_status);

JINX_RAII_SIMPLE_OBJECT(USBDeviceHandle, struct libusb_device_handle, libusb_close);

template<typename AsyncImpl, typename EventEngine=typename AsyncImpl::EventEngineType>
class AsyncUSBAgent
{
    EventEngine &_eve;

    libusb_context* _usb_ctx{nullptr};

    std::unordered_map<int, typename EventEngine::EventHandleIO*> _fds;

    bool _should_handle_timeouts{false};

    typename EventEngine::EventHandleTimer _timeout_handle;

public:
    explicit AsyncUSBAgent(EventEngine& eve) :_eve(eve) {
        int ret = libusb_init(&_usb_ctx);
        if (ret < 0) {
            throw std::runtime_error("failed to initialize libusb context");
        }
        _should_handle_timeouts = libusb_pollfds_handle_timeouts(_usb_ctx) == 0;

        auto* fds = libusb_get_pollfds(_usb_ctx);
        while (*fds != nullptr) {
            fd_added_cb((*fds)->fd, (*fds)->events, this);
            fds += 1;
        }

        libusb_set_pollfd_notifiers(_usb_ctx, fd_added_cb, fd_removed_cb, this);
    }

    JINX_NO_COPY_NO_MOVE(AsyncUSBAgent);

    ~AsyncUSBAgent()
    {
        libusb_exit(_usb_ctx);
        jinx_assert(_fds.empty());
    }

    operator libusb_context*() { // NOLINT
        return _usb_ctx;
    }

private:
    static void fd_added_cb(int usb_fd, short events, void* data)
    {
        auto* self = reinterpret_cast<AsyncUSBAgent*>(data);
        auto iter = self->_fds.find(usb_fd);
        if (iter != self->_fds.end()) {
            return;
        }

        auto pair = self->_fds.emplace(
            usb_fd, 
            new typename EventEngine::EventHandleIO{});

        unsigned int flags = EventEngine::IOFlagPersist;
        if ((events & POLLIN) != 0) {
            flags |= EventEngine::IOFlagRead;
        }
        if ((events & POLLOUT) != 0) {
            flags |= EventEngine::IOFlagWrite;
        }
        self->_eve.add_io(flags, *pair.first->second, usb_fd, handle_io_event, self) >> JINX_IGNORE_RESULT;
    }

    static void fd_removed_cb(int usb_fd, void* data)
    {
        auto* self = reinterpret_cast<AsyncUSBAgent*>(data);
        auto iter = self->_fds.find(usb_fd);
        if (iter == self->_fds.end()) {
            return;
        }
        self->_eve.remove_io(*iter->second) >> JINX_IGNORE_RESULT;
        delete iter->second;
        self->_fds.erase(iter);
    }

    void update_timeout()
    {
        if (not _should_handle_timeouts) {
            return;
        }

        struct timeval timeval{};
        int ret = libusb_get_next_timeout(_usb_ctx, &timeval);
        if (ret > 0) {
            _eve.add_timer(_timeout_handle, &timeval, handle_timer_event, this) >> JINX_IGNORE_RESULT;
        } else {
            _eve.remove_timer(_timeout_handle) >> JINX_IGNORE_RESULT;
        }
    }

    static void handle_io_event(unsigned int flags, const error::Error& error, void* data)
    {
        auto* self = reinterpret_cast<AsyncUSBAgent*>(data);
        struct timeval timeval = { 0, 0 };
        libusb_handle_events_timeout(self->_usb_ctx, &timeval);
        self->update_timeout();
    }

    static void handle_timer_event(const error::Error& error, void* data)
    {
        auto* self = reinterpret_cast<AsyncUSBAgent*>(data);
        struct timeval timeval = { 0, 0 };
        libusb_handle_events_timeout(self->_usb_ctx, &timeval);
        self->update_timeout();
    }
};

class USBControlTransfer : public jinx::AsyncFunction<int> {
    libusb_transfer* _transfer{nullptr};

public:
    USBControlTransfer() = default;
    ~USBControlTransfer() override {
        if (_transfer != nullptr) {
            libusb_free_transfer(_transfer);
        }
    }
    JINX_NO_COPY_NO_MOVE(USBControlTransfer);

    template<typename Rep, typename Period>
    USBControlTransfer& operator ()(
        libusb_device_handle* handle, 
        unsigned char* buffer, 
        const std::chrono::duration<Rep, Period>& timeout) 
    {
        if (_transfer == nullptr) {
            _transfer = libusb_alloc_transfer(0);
        }
        if (_transfer == nullptr) {
            this->async_throw(make_error(LIBUSB_ERROR_NO_MEM));
            return *this;
        }
        unsigned int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        libusb_fill_control_transfer(
            _transfer, 
            handle, 
            buffer, 
            resume, this, timeout_ms);
        async_start(&USBControlTransfer::submit);
        return *this;
    }

protected:
    jinx::Async submit() {
        int ret = libusb_submit_transfer(_transfer);
        if (ret < 0) {
            return this->async_throw(make_error(static_cast<libusb_error>(ret)));
        }
        async_start(&USBControlTransfer::completed);
        return async_suspend();
    }

    jinx::Async completed() {
        if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            return async_throw(make_error(_transfer->status));
        }
        emplace_result(_transfer->actual_length);
        return async_return();
    }

    static void resume(libusb_transfer* transfer)
    {
        auto* self = reinterpret_cast<USBControlTransfer*>(transfer->user_data);
        self->async_resume() >> JINX_IGNORE_RESULT;
    }
};

class USBBulkTransfer : public jinx::AsyncFunction<int> {
    libusb_transfer* _transfer{nullptr};

public:
    USBBulkTransfer() = default;
    ~USBBulkTransfer() override {
        if (_transfer != nullptr) {
            libusb_free_transfer(_transfer);
        }
    }

    JINX_NO_COPY_NO_MOVE(USBBulkTransfer);

    template<typename Rep, typename Period>
    USBBulkTransfer& operator ()(
        libusb_device_handle* handle, 
        unsigned char endpoint,
        jinx::SliceRead buffer,
        const std::chrono::duration<Rep, Period>& timeout) 
    {
        if (_transfer == nullptr) {
            _transfer = libusb_alloc_transfer(0);
        }
        if (_transfer == nullptr) {
            async_throw(make_error(LIBUSB_ERROR_NO_MEM));
            return *this;
        }
        unsigned int timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        libusb_fill_bulk_transfer(
            _transfer, 
            handle, 
            endpoint,
            reinterpret_cast<unsigned char*>(buffer.data()),
            buffer.size(), 
            resume, this, timeout_ms);
        async_start(&USBBulkTransfer::submit);
        return *this;
    }

protected:
    jinx::Async submit() {

        int ret = libusb_submit_transfer(_transfer);
        if (ret < 0) {
            return async_throw(make_error(static_cast<libusb_error>(ret)));
        }
        async_start(&USBBulkTransfer::completed);
        return async_suspend();
    }

    jinx::Async completed() {
        if (_transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            return async_throw(make_error(_transfer->status));
        }
        emplace_result(_transfer->actual_length);
        return async_return();
    }

    static void resume(libusb_transfer* transfer)
    {
        auto* self = reinterpret_cast<USBBulkTransfer*>(transfer->user_data);
        self->async_resume() >> JINX_IGNORE_RESULT;
    }
};

} // namespace usb
} // namespace jinx

#endif
