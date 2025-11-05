// Copyright (c) 2025 Stefan de Bruijn. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Machine/MachineConfig.h"
#include "Channel.h"
#include "Module.h"
#include "Main.h"
#include "WebUIServer.h"
#include "TelnetServer.h"
#include "Driver/spi.h"

#include <esp_eth.h>
#include <esp_eth_mac.h>
#include <esp_eth_phy.h>
#include <esp_eth_driver.h>
#include <esp_eth_mac_spi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define HSPI_HOST SPI2_HOST
#endif

namespace WebUI {
    extern bool needsNetworkServices;

    class EthernetConfig : public ConfigurableModule {
    private:
        Pin     _cs_pin;
        Pin     _int_pin;
        Pin     _rst_pin;
        bool    _dhcp         = true;
        int32_t _static_ip    = 0;
        int32_t _gateway      = 0;
        int32_t _netmask      = 0;
        int32_t _spi_freq_mhz = 20;

        esp_eth_handle_t            _eth_handle = nullptr;
        esp_netif_t*                _eth_netif  = nullptr;
        esp_eth_netif_glue_handle_t _eth_glue   = nullptr;

        bool    _ethernet_active = false;
        uint8_t _mac_address[6]  = { 0 };

        static EthernetConfig* _instance;

        static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
            if (_instance == nullptr)
                return;

            switch (event_id) {
                case ETHERNET_EVENT_CONNECTED:
                    log_info("Ethernet Link Up");
                    break;
                case ETHERNET_EVENT_DISCONNECTED:
                    log_info("Ethernet Link Down");
                    break;
                case ETHERNET_EVENT_START:
                    log_info("Ethernet Started");
                    break;
                case ETHERNET_EVENT_STOP:
                    log_info("Ethernet Stopped");
                    break;
                default:
                    break;
            }
        }

        static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
            if (_instance == nullptr)
                return;

            if (event_id == IP_EVENT_ETH_GOT_IP) {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
                log_info("Ethernet Got IP: " << IP_string(IPAddress(event->ip_info.ip.addr)));
            }
        }

        static const char* mac2str(uint8_t mac[6]) {
            static char macstr[18];
            if (0 > sprintf(macstr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5])) {
                strcpy(macstr, "00:00:00:00:00:00");
            }
            return macstr;
        }

        void print_mac(Channel& out, const char* prefix, const char* mac) { log_stream(out, prefix << " (" << mac << ")"); }

        bool StartEthernet() {
            if (_cs_pin.undefined()) {
                log_info("Ethernet CS pin not configured");
                return false;
            }

            esp_err_t ret;

            // Initialize network infrastructure (in case WiFi is disabled)
            // These are safe to call multiple times
            ret = esp_netif_init();
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                log_error("esp_netif_init failed: " << esp_err_to_name(ret));
                return false;
            }

            ret = esp_event_loop_create_default();
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                log_error("esp_event_loop_create_default failed: " << esp_err_to_name(ret));
                return false;
            }

            // Generate MAC address based on ESP32 chip ID
            uint64_t chipid = ESP.getEfuseMac();
            _mac_address[0] = 0x02;  // Locally administered MAC
            _mac_address[1] = 0x00;
            _mac_address[2] = (chipid >> 32) & 0xFF;
            _mac_address[3] = (chipid >> 16) & 0xFF;
            _mac_address[4] = (chipid >> 8) & 0xFF;
            _mac_address[5] = chipid & 0xFF;

            log_info("Starting Ethernet with CS pin " << _cs_pin.name());
            log_info("MAC: " << mac2str(_mac_address));

            // Install GPIO ISR service if using interrupt pin
            if (!_int_pin.undefined()) {
                ret = gpio_install_isr_service(0);
                if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
                    log_error("GPIO ISR service install failed: " << esp_err_to_name(ret));
                }
            }

            // Configure SPI device for DM9051 (let the driver manage it)
            spi_device_interface_config_t spi_devcfg = {};
            spi_devcfg.command_bits                  = 0;
            spi_devcfg.address_bits                  = 0;
            spi_devcfg.mode                          = 0;
            spi_devcfg.clock_speed_hz                = _spi_freq_mhz * 1000000;
            spi_devcfg.spics_io_num                  = _cs_pin.getNative(Pin::Capabilities::Output);
            spi_devcfg.queue_size                    = 20;

            eth_dm9051_config_t dm9051_config = ETH_DM9051_DEFAULT_CONFIG(HSPI_HOST, &spi_devcfg);
            dm9051_config.int_gpio_num        = _int_pin.undefined() ? -1 : _int_pin.getNative(Pin::Capabilities::Input);
            dm9051_config.poll_period_ms      = _int_pin.undefined() ? 10 : 0;

            // Create MAC
            eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
            esp_eth_mac_t*   mac        = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
            if (mac == nullptr) {
                log_error("MAC creation failed");
                return false;
            }

            // Create PHY
            eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
            phy_config.phy_addr         = -1;  // Auto-detect
            phy_config.reset_gpio_num   = _rst_pin.undefined() ? -1 : _rst_pin.getNative(Pin::Capabilities::Output);
            esp_eth_phy_t* phy          = esp_eth_phy_new_dm9051(&phy_config);
            if (phy == nullptr) {
                log_error("PHY creation failed");
                mac->del(mac);
                return false;
            }

            // Install Ethernet driver
            esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
            ret                         = esp_eth_driver_install(&eth_config, &_eth_handle);
            if (ret != ESP_OK) {
                log_error("Ethernet driver install failed: " << esp_err_to_name(ret));
                phy->del(phy);
                mac->del(mac);
                return false;
            }

            // Set MAC address
            ret = esp_eth_ioctl(_eth_handle, ETH_CMD_S_MAC_ADDR, _mac_address);
            if (ret != ESP_OK) {
                log_error("Set MAC address failed: " << esp_err_to_name(ret));
                return false;
            }

            // Create network interface
            esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
            _eth_netif                      = esp_netif_new(&netif_config);
            if (_eth_netif == nullptr) {
                log_error("Network interface creation failed");
                return false;
            }

            // Attach Ethernet driver to TCP/IP stack
            _eth_glue = esp_eth_new_netif_glue(_eth_handle);
            if (_eth_glue == nullptr) {
                log_error("Netif glue creation failed");
                return false;
            }

            ret = esp_netif_attach(_eth_netif, _eth_glue);
            if (ret != ESP_OK) {
                log_error("Netif attach failed: " << esp_err_to_name(ret));
                return false;
            }

            // Register event handlers
            ret = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, nullptr);
            if (ret != ESP_OK) {
                log_error("Event handler register failed: " << esp_err_to_name(ret));
                return false;
            }

            ret = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, nullptr);
            if (ret != ESP_OK) {
                log_error("IP event handler register failed: " << esp_err_to_name(ret));
                return false;
            }

            // Configure static IP if needed
            if (!_dhcp && _static_ip != 0) {
                esp_netif_dhcpc_stop(_eth_netif);

                esp_netif_ip_info_t ip_info;
                ip_info.ip.addr      = _static_ip;
                ip_info.gw.addr      = _gateway != 0 ? _gateway : _static_ip;
                ip_info.netmask.addr = _netmask != 0 ? _netmask : 0xFFFFFF00;  // 255.255.255.0

                ret = esp_netif_set_ip_info(_eth_netif, &ip_info);
                if (ret != ESP_OK) {
                    log_error("Set static IP failed: " << esp_err_to_name(ret));
                }
            }

            // Start Ethernet driver
            ret = esp_eth_start(_eth_handle);
            if (ret != ESP_OK) {
                log_error("Ethernet start failed: " << esp_err_to_name(ret));
                return false;
            }

            _ethernet_active = true;
            return true;
        }

        void StopEthernet() {
            if (_ethernet_active) {
                if (_eth_handle != nullptr) {
                    esp_eth_stop(_eth_handle);
                    esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler);
                    esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler);

                    if (_eth_glue != nullptr) {
                        esp_eth_del_netif_glue(_eth_glue);
                        _eth_glue = nullptr;
                    }

                    esp_eth_driver_uninstall(_eth_handle);
                    _eth_handle = nullptr;
                }

                if (_eth_netif != nullptr) {
                    esp_netif_destroy(_eth_netif);
                    _eth_netif = nullptr;
                }

                _ethernet_active = false;
                log_info("Ethernet Off");
            }
        }

        std::string ethernet_info() {
            std::string result;
            if (_ethernet_active && _eth_netif != nullptr) {
                result += "Mode=Ethernet:Status=";

                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(_eth_netif, &ip_info) == ESP_OK) {
                    result += (ip_info.ip.addr != 0) ? "Connected" : "Disconnected";
                    result += ":IP=";
                    result += IP_string(IPAddress(ip_info.ip.addr));
                } else {
                    result += "Unknown";
                }

                result += ":MAC=";
                std::string mac_str = mac2str(_mac_address);
                std::replace(mac_str.begin(), mac_str.end(), ':', '-');
                result += mac_str;
            }
            return result;
        }

    public:
        EthernetConfig(const char* name) : ConfigurableModule(name) { _instance = this; }

        void group(Configuration::HandlerBase& handler) override {
            handler.item("cs_pin", _cs_pin);
            handler.item("int_pin", _int_pin);
            handler.item("rst_pin", _rst_pin);
            handler.item("dhcp", _dhcp);
            handler.item("ip", _static_ip);
            handler.item("gateway", _gateway);
            handler.item("netmask", _netmask);
            handler.item("spi_freq_mhz", _spi_freq_mhz, 1, 40);
        }

        void validate() override {
            if (!_cs_pin.undefined()) {
                if (!_dhcp && _static_ip == 0) {
                    log_warn("Ethernet configured for static IP but no IP address specified");
                }
            }
        }

        void afterParse() override {}

        void init() override {
            if (!_cs_pin.undefined()) {
                needsNetworkServices = true;
                
                _cs_pin.setAttr(Pin::Attr::Output);
                if (!_int_pin.undefined()) {
                    _int_pin.setAttr(Pin::Attr::Input | Pin::Attr::ISR);
                }
                if (!_rst_pin.undefined()) {
                    _rst_pin.setAttr(Pin::Attr::Output);
                }

                if (StartEthernet()) {
                    log_info("Ethernet initialized");
                } else {
                    log_error("Ethernet initialization failed");
                }
            } else {
                log_info("Ethernet disabled (CS pin not configured)");
            }
        }

        int  init_priority() override { return 0x5000; };

        void deinit() override { StopEthernet(); }

        void build_info(Channel& channel) {
            std::string eth_info_str = ethernet_info();
            if (eth_info_str.length()) {
                log_msg_to(channel, eth_info_str);
            } else {
                log_msg_to(channel, "No Ethernet");
            }
        }

        void poll() {
            // Ethernet maintenance is handled by ESP-IDF
        }

        void status_report(Channel& out) {
            if (_ethernet_active && _eth_netif != nullptr) {
                log_stream(out, "Ethernet: Active");
                log_stream(out, "Web port: " << WebUI_Server::port());
                print_mac(out, "MAC", mac2str(_mac_address));
                log_stream(out, "IP Mode: " << (_dhcp ? "DHCP" : "Static"));

                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(_eth_netif, &ip_info) == ESP_OK) {
                    log_stream(out, "IP: " << IP_string(IPAddress(ip_info.ip.addr)));
                    log_stream(out, "Gateway: " << IP_string(IPAddress(ip_info.gw.addr)));
                    log_stream(out, "Mask: " << IP_string(IPAddress(ip_info.netmask.addr)));
                }
            } else {
                log_stream(out, "Ethernet: Off");
            }
        }

        void wifi_stats(JSONencoder& j) {
            if (_ethernet_active && _eth_netif != nullptr) {
                j.id_value_object("Ethernet Mode", "Active");
                j.id_value_object("Web port", WebUI_Server::port());
                j.id_value_object("Data port", TelnetServer::port());
                j.id_value_object("MAC", mac2str(_mac_address));
                j.id_value_object("IP Mode", _dhcp ? "DHCP" : "Static");

                esp_netif_ip_info_t ip_info;
                if (esp_netif_get_ip_info(_eth_netif, &ip_info) == ESP_OK) {
                    j.id_value_object("IP", IP_string(IPAddress(ip_info.ip.addr)));
                    j.id_value_object("Gateway", IP_string(IPAddress(ip_info.gw.addr)));
                    j.id_value_object("Mask", IP_string(IPAddress(ip_info.netmask.addr)));
                }
            } else {
                j.id_value_object("Ethernet Mode", "Off");
            }
        }

        ~EthernetConfig() {
            deinit();
            if (_instance == this) {
                _instance = nullptr;
            }
        }
    };

    EthernetConfig* EthernetConfig::_instance = nullptr;

    ConfigurableModuleFactory::InstanceBuilder<EthernetConfig> __attribute__((init_priority(106))) ethernet_module("ethernet");
}
