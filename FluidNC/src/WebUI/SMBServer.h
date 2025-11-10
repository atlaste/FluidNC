// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Module.h"
#include <AsyncTCP.h>
#include <vector>
#include <string>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Forward declare libsmb2 types
struct smb2_context;
struct smb2_server;
struct smb2_server_request_handlers;

namespace WebUI {

    class SMBClient;

    // Forward declarations for file I/O structures
    struct FileIORequest;
    struct FileIOResult;

    class SMBServer : public ConfigurableModule {
    public:
        SMBServer(const char* name);
        ~SMBServer();

        // ConfigurableModule interface
        void init() override;
        void deinit() override;
        void poll();
        void group(Configuration::HandlerBase& handler) override;
        int  init_priority() override { return 0x5200; }

        // Accessors for clients
        const std::string& sharePath() const { return _share_path; }
        QueueHandle_t      fileIOQueue() const { return _fileIOQueue; }

    private:
        friend class SMBClient;
        
        // Configuration
        bool        _enabled     = false;
        int32_t     _port        = 445;
        std::string _server_name = "FluidNCS3";
        std::string _share_path  = "/";

        // Network
        AsyncServer*            _asyncServer = nullptr;
        std::vector<SMBClient*> _clients;
        SemaphoreHandle_t       _clientsMutex = nullptr;

        // File I/O worker
        QueueHandle_t _fileIOQueue = nullptr;
        TaskHandle_t  _fileIOTask  = nullptr;

        // libsmb2 server infrastructure
        smb2_server*                     _smb2_server   = nullptr;
        smb2_server_request_handlers*    _smb2_handlers = nullptr;

        // Internal methods
        void        handleNewClient(AsyncClient* client);
        static void fileIOWorkerTask(void* param);
        void        cleanupDisconnectedClients();

        // Maximum clients
        static const size_t MAX_CLIENTS = 2;
    };

}  // namespace WebUI

