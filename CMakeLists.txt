
pkg_check_modules(USB REQUIRED IMPORTED_TARGET libusb-1.0>1.0)

add_library(jinx_usb STATIC usb.cpp)
add_library(jinx::usb ALIAS jinx_usb)
target_include_directories(jinx_usb PUBLIC include)
target_link_libraries(jinx_usb PUBLIC jinx PkgConfig::USB)
