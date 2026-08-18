#include <sdkconfig.h>
#include <esp_idf_version.h>

#if defined(CONFIG_TINYUSB_CDC_ENABLED) && ESP_IDF_VERSION_MAJOR >= 5
#    include "USBCDCChannel_IDF.h"
#else
#    include "USBCDCChannel.h"
#endif

USBCDCChannel CDCChannel(true);

#include "UartChannel.h"

#include "Settings.h"

// This derived class overrides init() to setup the primary UART
class UartConsole : public UartChannel {
public:
    UartConsole() : UartChannel(0, true) {}
    void init() override {
        auto uart0 = new Uart(0);
        uart0->begin(BAUD_RATE, UartData::Bits8, UartStop::Bits1, UartParity::None);
        UartChannel::init(uart0);

#if defined(CONFIG_TINYUSB_CDC_ENABLED) && ESP_IDF_VERSION_MAJOR >= 5
        auto cdc_enable = new EnumSetting("USB CDC Enable", WEBSET, WG, NULL, "USBCDC/Enable", false, &onoffOptions);
        if (cdc_enable->get()) {
            CDCChannel.init();
        }
#endif
    }
};

#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG

// The console is the chip's native USB port, which means UART0's pins are not wired to anything a
// host can reach, so talk to USB-Serial-JTAG instead.  UartConsole is not instantiated at all here:
// bringing UART0 up would claim its pins for a channel nobody can hear, and on boards built this way
// those pins are usually needed for something else.
//
// TinyUSB CDC is deliberately not offered in this configuration either.  Starting it would hand the
// shared USB PHY over to USB-OTG and disconnect this console along with the flashing path.
#    include "USBSerialJTAGChannel.h"

USBSerialJTAGChannel UsbSerialJtag(true);
Channel&             Console = UsbSerialJtag;

#else

UartConsole Uart0;
Channel&    Console = Uart0;

#endif
