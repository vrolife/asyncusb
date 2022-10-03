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
#include "jinx/usb/usb.hpp"
#include "jinx/error.hpp"
#include <libusb.h>

namespace jinx {
namespace usb {

JINX_ERROR_IMPLEMENT(usb, {
    return libusb_error_name(code.value());
});

JINX_ERROR_IMPLEMENT(transfer, {
    switch(code.as<libusb_transfer_status>()) {
        case LIBUSB_TRANSFER_COMPLETED:
            return "transfer completed";
        case LIBUSB_TRANSFER_ERROR:
            return "transfer error";
        case LIBUSB_TRANSFER_TIMED_OUT:
            return "transfer timeout";
        case LIBUSB_TRANSFER_CANCELLED:
            return "transfer cancelled";
        case LIBUSB_TRANSFER_STALL:
            return "transfer stall";
        case LIBUSB_TRANSFER_NO_DEVICE:
            return "transfer no device";
        case LIBUSB_TRANSFER_OVERFLOW:
            return "transfer overflow";
    }
    return "unknown transfer status";
});

} // namespace usb
} // namespace jinx
