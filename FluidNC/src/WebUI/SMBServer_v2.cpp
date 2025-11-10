// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "SMBServer.h"
#include "Machine/MachineConfig.h"
#include "Report.h"
#include "FluidPath.h"
#include "Settings.h"
#include "Mdns.h"

#include <AsyncTCP.h>
#include <cstring>
#include <cstdio>
#include <sys/stat.h>
#include <dirent.h>
#include <sstream>
#include <iomanip>

// Include libsmb2 headers
extern "C" {
#include "smb2/libsmb2.h"
#include "smb2/smb2.h"
#include "libsmb2-private.h"
}

namespace WebUI {

    // ============================================================================
    // Helper Functions
    // ============================================================================

    static void dumpHex(const char* label, const uint8_t* data, size_t len) {
        const size_t maxDump = 256;
        size_t dumpLen = len > maxDump ? maxDump : len;
        
        log_debug("SMB: " << label << " (" << len << " bytes):");
        
        for (size_t i = 0; i < dumpLen; i += 16) {
            std::stringstream line;
            line << "  " << std::hex << std::setfill('0') << std::setw(4) << i << ": ";
            
            for (size_t j = 0; j < 16 && (i + j) < dumpLen; j++) {
                line << std::setw(2) << (int)data[i + j] << " ";
            }
            
            for (size_t j = dumpLen - i; j < 16; j++) {
                line << "   ";
            }
            
            line << " | ";
            for (size_t j = 0; j < 16 && (i + j) < dumpLen; j++) {
                uint8_t c = data[i + j];
                line << (char)(c >= 32 && c < 127 ? c : '.');
            }
            
            log_debug(line.str());
        }
        
        if (len > maxDump) {
            log_debug("  ... (" << (len - maxDump) << " more bytes truncated)");
        }
    }

    // ============================================================================
    // SMBClient - wraps libsmb2 context and AsyncTCP client
    // ============================================================================

    class SMBClient {
    public:
        SMBClient(SMBServer* server, AsyncClient* tcp);
        ~SMBClient();

        bool isDisconnected() const { return _disconnected; }
        
        // Process incoming data from AsyncTCP
        void processIncomingData(const uint8_t* data, size_t len);
        
        // Send any pending outgoing data via AsyncTCP
        void flushOutgoingData();

    private:
        SMBServer*        _server;
        AsyncClient*      _tcp;
        smb2_context*     _smb2_ctx;
        bool              _disconnected;

        // AsyncTCP callbacks
        void onTcpDisconnect();
        void onTcpError(int8_t error);
        
        // File handle tracking (fileId -> FILE*)
        std::map<uint64_t, FILE*> _openFiles;
        uint64_t _nextFileId = 1000;

        friend class SMBServer;
    };

    // ============================================================================
    // libsmb2 Handler Callbacks
    // ============================================================================

    static int smb_authorize_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                     const char *user, const char *domain, const char *workstation) {
        // Allow anonymous access
        return 0;
    }

    static int smb_session_handler(struct smb2_server *srvr, struct smb2_context *smb2) {
        log_debug("SMB: Session established, dialect " << std::hex << smb2_get_dialect(smb2));
        return 0;
    }

    static int smb_logoff_handler(struct smb2_server *srvr, struct smb2_context *smb2) {
        log_debug("SMB: Logoff");
        return 0;
    }

    static int smb_tree_connect_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                        struct smb2_tree_connect_request *req,
                                        struct smb2_tree_connect_reply *rep) {
        log_debug("SMB: Tree connect");
        rep->share_type = SMB2_SHARE_TYPE_DISK;
        rep->maximal_access = 0x101f01ff;  // Full access
        rep->share_flags = 0;
        rep->capabilities = 0;
        return 0;
    }

    static int smb_tree_disconnect_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                          const uint32_t tree_id) {
        log_debug("SMB: Tree disconnect");
        return 0;
    }

    static int smb_create_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                  struct smb2_create_request *req,
                                  struct smb2_create_reply *rep) {
        log_debug("SMB: CREATE - placeholder");
        rep->file_attributes = SMB2_FILE_ATTRIBUTE_NORMAL;
        return 0;
    }

    static int smb_close_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                struct smb2_close_request *req,
                                struct smb2_close_reply *rep) {
        log_debug("SMB: CLOSE - placeholder");
        memset(rep, 0, sizeof(*rep));
        return 0;
    }

    static int smb_read_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                               struct smb2_read_request *req,
                               struct smb2_read_reply *rep) {
        log_debug("SMB: READ - placeholder");
        rep->data = nullptr;
        rep->data_length = 0;
        rep->data_remaining = 0;
        return 0;
    }

    static int smb_write_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                struct smb2_write_request *req,
                                struct smb2_write_reply *rep) {
        log_debug("SMB: WRITE - placeholder");
        rep->count = req->length;
        rep->remaining = 0;
        return 0;
    }

    static int smb_query_info_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                      struct smb2_query_info_request *req,
                                      struct smb2_query_info_reply *rep) {
        log_debug("SMB: QUERY_INFO - placeholder");
        rep->output_buffer = nullptr;
        rep->output_buffer_length = 0;
        return 0;
    }

    static int smb_query_directory_handler(struct smb2_server *srvr, struct smb2_context *smb2,
                                          struct smb2_query_directory_request *req,
                                          struct smb2_query_directory_reply *rep) {
        log_debug("SMB: QUERY_DIRECTORY - placeholder");
        rep->output_buffer = nullptr;
        rep->output_buffer_length = 0;
        return 0;
    }

    static int smb_echo_handler(struct smb2_server *srvr, struct smb2_context *smb2) {
        log_debug("SMB: ECHO");
        return 0;
    }

    // ============================================================================
    // SMBClient Implementation
    // ============================================================================

    SMBClient::SMBClient(SMBServer* server, AsyncClient* tcp)
        : _server(server), _tcp(tcp), _disconnected(false) {
        
        log_info("SMB: New client from " << _tcp->remoteIP().toString().c_str());

        // Create libsmb2 context in server mode
        _smb2_ctx = smb2_init_context();
        if (!_smb2_ctx) {
            log_error("SMB: Failed to create libsmb2 context");
            _disconnected = true;
            return;
        }

        // Mark as server mode
        _smb2_ctx->owning_server = _server->_smb2_server;

        // Configure AsyncTCP
        _tcp->setNoDelay(true);
        _tcp->setRxTimeout(30);
        _tcp->setAckTimeout(5000);

        // Set up callbacks
        _tcp->onData([](void* arg, AsyncClient* c, void* data, size_t len) {
            static_cast<SMBClient*>(arg)->processIncomingData((const uint8_t*)data, len);
        }, this);

        _tcp->onDisconnect([](void* arg, AsyncClient* c) {
            static_cast<SMBClient*>(arg)->onTcpDisconnect();
        }, this);

        _tcp->onError([](void* arg, AsyncClient* c, int8_t error) {
            static_cast<SMBClient*>(arg)->onTcpError(error);
        }, this);
    }

    SMBClient::~SMBClient() {
        log_debug("SMB: Destroying client");

        // Close all open files
        for (auto& pair : _openFiles) {
            if (pair.second) {
                fclose(pair.second);
            }
        }
        _openFiles.clear();

        // Free libsmb2 context
        if (_smb2_ctx) {
            smb2_destroy_context(_smb2_ctx);
            _smb2_ctx = nullptr;
        }

        // Close AsyncTCP
        if (_tcp) {
            _tcp->close();
            delete _tcp;
            _tcp = nullptr;
        }
    }

    void SMBClient::processIncomingData(const uint8_t* data, size_t len) {
        if (_disconnected || !_smb2_ctx) {
            return;
        }

        log_debug("SMB: RX " << len << " bytes");
        dumpHex("RX", data, len);

        // ACK immediately to keep TCP window open
        _tcp->ack(len);

        // Copy data to libsmb2's encrypted buffer (reusing the mechanism for our async data)
        _smb2_ctx->enc = (uint8_t*)malloc(len);
        if (!_smb2_ctx->enc) {
            log_error("SMB: Out of memory for RX buffer");
            _disconnected = true;
            return;
        }

        memcpy(_smb2_ctx->enc, data, len);
        _smb2_ctx->enc_len = (int)len;
        _smb2_ctx->enc_pos = 0;

        // Let libsmb2 process the data
        if (smb2_read_from_buf(_smb2_ctx) < 0) {
            log_error("SMB: Failed to process data: " << smb2_get_error(_smb2_ctx));
            _disconnected = true;
        }

        // Send any pending responses
        flushOutgoingData();
    }

    void SMBClient::flushOutgoingData() {
        if (!_tcp || !_smb2_ctx || !_tcp->connected()) {
            return;
        }

        // Process all PDUs in the outqueue
        while (_smb2_ctx->outqueue) {
            struct smb2_pdu* pdu = _smb2_ctx->outqueue;
            
            // Calculate total size from io vectors
            size_t total_size = 0;
            for (int i = 0; i < pdu->out.niov; i++) {
                total_size += pdu->out.iov[i].len;
            }

            // Allocate buffer for the complete PDU
            std::vector<uint8_t> buffer(total_size);
            size_t offset = 0;

            // Copy all iovecs into the buffer
            for (int i = 0; i < pdu->out.niov; i++) {
                memcpy(buffer.data() + offset, pdu->out.iov[i].buf, pdu->out.iov[i].len);
                offset += pdu->out.iov[i].len;
            }

            log_debug("SMB: TX " << total_size << " bytes");
            dumpHex("TX", buffer.data(), total_size);

            // Send via AsyncTCP
            if (_tcp->canSend()) {
                size_t written = _tcp->write((const char*)buffer.data(), total_size);
                if (written != total_size) {
                    log_error("SMB: Partial write: " << written << "/" << total_size);
                    _disconnected = true;
                    break;
                }
            } else {
                log_error("SMB: Cannot send, buffer full");
                _disconnected = true;
                break;
            }

            // Remove PDU from queue
            SMB2_LIST_REMOVE(&_smb2_ctx->outqueue, pdu);
            smb2_free_pdu(_smb2_ctx, pdu);
        }
    }

    void SMBClient::onTcpDisconnect() {
        log_info("SMB: Client disconnected");
        _disconnected = true;
    }

    void SMBClient::onTcpError(int8_t error) {
        log_error("SMB: TCP error " << (int)error);
        _disconnected = true;
    }

    // ============================================================================
    // SMBServer Implementation
    // ============================================================================

    SMBServer::SMBServer(const char* name) : ConfigurableModule(name) {}

    SMBServer::~SMBServer() {
        deinit();
    }

    void SMBServer::group(Configuration::HandlerBase& handler) {
        handler.item("enabled", _enabled);
        handler.item("port", _port);
        handler.item("server_name", _server_name);
        handler.item("share_path", _share_path);
    }

    void SMBServer::init() {
        if (!_enabled) {
            log_debug("SMB Server disabled");
            return;
        }

        // Check for SD card
        if (!config->_sdCard || !config->_sdCard->config_ok) {
            log_error("SMB Server requires SD card configuration");
            return;
        }

        // Check for network
        extern bool needsNetworkServices;
        if (!needsNetworkServices) {
            log_warn("SMB Server: No network available");
            return;
        }

        log_info("Starting SMB Server on port " << _port);

        // Create mutex
        _clientsMutex = xSemaphoreCreateMutex();
        if (!_clientsMutex) {
            log_error("SMB: Failed to create mutex");
            return;
        }

        // Set up libsmb2 server structure
        _smb2_server = new smb2_server();
        memset(_smb2_server, 0, sizeof(*_smb2_server));

        _smb2_handlers = new smb2_server_request_handlers();
        memset(_smb2_handlers, 0, sizeof(*_smb2_handlers));

        // Configure handlers
        _smb2_handlers->authorize_user = smb_authorize_handler;
        _smb2_handlers->session_established = smb_session_handler;
        _smb2_handlers->logoff_cmd = smb_logoff_handler;
        _smb2_handlers->tree_connect_cmd = smb_tree_connect_handler;
        _smb2_handlers->tree_disconnect_cmd = smb_tree_disconnect_handler;
        _smb2_handlers->create_cmd = smb_create_handler;
        _smb2_handlers->close_cmd = smb_close_handler;
        _smb2_handlers->read_cmd = smb_read_handler;
        _smb2_handlers->write_cmd = smb_write_handler;
        _smb2_handlers->query_info_cmd = smb_query_info_handler;
        _smb2_handlers->query_directory_cmd = smb_query_directory_handler;
        _smb2_handlers->echo_cmd = smb_echo_handler;

        _smb2_server->handlers = _smb2_handlers;
        _smb2_server->signing_enabled = 0;  // Disable signing for now
        _smb2_server->allow_anonymous = 1;
        _smb2_server->port = _port;
        _smb2_server->max_transact_size = 0x100000;
        _smb2_server->max_read_size = 0x100000;
        _smb2_server->max_write_size = 0x100000;
        memcpy(_smb2_server->guid, "FluidNC-SMB-GUID", 16);
        strncpy(_smb2_server->hostname, _server_name.c_str(), sizeof(_smb2_server->hostname) - 1);
        strncpy(_smb2_server->domain, "WORKGROUP", sizeof(_smb2_server->domain) - 1);

        // Create AsyncServer
        _asyncServer = new AsyncServer(_port);
        _asyncServer->onClient([](void* arg, AsyncClient* client) {
            static_cast<SMBServer*>(arg)->handleNewClient(client);
        }, this);
        _asyncServer->begin();

        // Register mDNS
        Mdns::add("_smb", "_tcp", _port);

        log_info("SMB Server started successfully");
    }

    void SMBServer::deinit() {
        // Remove mDNS
        Mdns::remove("_smb", "_tcp");

        // Stop AsyncServer
        if (_asyncServer) {
            _asyncServer->end();
            delete _asyncServer;
            _asyncServer = nullptr;
        }

        // Clean up clients
        if (_clientsMutex) {
            xSemaphoreTake(_clientsMutex, portMAX_DELAY);
        }
        for (auto client : _clients) {
            delete client;
        }
        _clients.clear();
        if (_clientsMutex) {
            xSemaphoreGive(_clientsMutex);
            vSemaphoreDelete(_clientsMutex);
            _clientsMutex = nullptr;
        }

        // Clean up libsmb2 structures
        if (_smb2_handlers) {
            delete _smb2_handlers;
            _smb2_handlers = nullptr;
        }
        if (_smb2_server) {
            delete _smb2_server;
            _smb2_server = nullptr;
        }
    }

    void SMBServer::handleNewClient(AsyncClient* client) {
        cleanupDisconnectedClients();

        if (!_clientsMutex || xSemaphoreTake(_clientsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            log_error("SMB: Failed to acquire mutex");
            client->close();
            delete client;
            return;
        }

        if (_clients.size() >= MAX_CLIENTS) {
            log_warn("SMB: Maximum clients reached");
            xSemaphoreGive(_clientsMutex);
            client->close();
            delete client;
            return;
        }

        SMBClient* smbClient = new SMBClient(this, client);
        _clients.push_back(smbClient);

        log_info("SMB: Client connected, total: " << _clients.size());
        
        xSemaphoreGive(_clientsMutex);
    }

    void SMBServer::cleanupDisconnectedClients() {
        if (!_clientsMutex || xSemaphoreTake(_clientsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }

        auto it = _clients.begin();
        while (it != _clients.end()) {
            if ((*it)->isDisconnected()) {
                delete *it;
                it = _clients.erase(it);
            } else {
                ++it;
            }
        }

        xSemaphoreGive(_clientsMutex);
    }

    void SMBServer::fileIOWorkerTask(void* param) {
        // Placeholder - not needed yet since we're processing in handlers
        vTaskDelete(nullptr);
    }

    // Register module
    ConfigurableModuleFactory::InstanceBuilder<SMBServer> __attribute__((init_priority(110))) smb_server("smb_server");

}  // namespace WebUI

