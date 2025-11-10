// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "SMBServer.h"
#include "Machine/MachineConfig.h"
#include "Report.h"
#include "FluidPath.h"
#include "Settings.h"
#include "Mdns.h"

#include <AsyncTCP.h>
#include <algorithm>
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
    // FileIO Request/Result Structures
    // ============================================================================

    struct FileIORequest {
        enum Type { OPEN, CLOSE, READ, WRITE, STAT, READDIR };

        Type     type;
        uint64_t requestId;  // To match with response
        uint64_t fileId;
        uint64_t offset;
        uint32_t length;
        uint32_t desiredAccess;
        uint32_t createDisposition;
        std::string path;
        std::vector<uint8_t> data;  // For writes

        // Callback - must be thread-safe
        std::function<void(const FileIOResult&)> callback;
    };

    struct FileIOResult {
        enum Status { SUCCESS = 0, NOT_FOUND, ACCESS_DENIED, IO_ERROR, NO_SD_CARD };

        uint64_t requestId;
        Status   status;
        uint64_t fileId;  // For OPEN
        uint64_t fileSize;
        uint64_t allocationSize;
        uint32_t fileAttributes;
        uint64_t creationTime;
        uint64_t lastAccessTime;
        uint64_t lastWriteTime;
        uint64_t changeTime;
        std::vector<uint8_t>       data;      // For READ
        std::vector<std::string>   dirEntries;  // For READDIR
        std::vector<struct stat>   dirStats;    // File stats for directory entries
    };

    // ============================================================================
    // SMBClient Class - Per-connection state machine
    // ============================================================================

    class SMBClient {
    public:
        SMBClient(SMBServer* server, AsyncClient* client);
        ~SMBClient();

        void         onTcpData(void* arg, AsyncClient* client, void* data, size_t len);
        void         onTcpDisconnect(void* arg, AsyncClient* client);
        void         onTcpError(void* arg, AsyncClient* client, int8_t error);
        bool         isDisconnected() const { return _disconnected; }
        AsyncClient* tcpClient() { return _tcp; }

    private:
        SMBServer*   _server;
        AsyncClient* _tcp;
        bool         _disconnected;

        // RX buffer for accumulating SMB packets
        std::vector<uint8_t> _rxBuffer;

        // SMB protocol state
        uint64_t _sessionId;
        uint32_t _treeId;
        uint64_t _messageId;
        uint64_t _nextFileId;

        // Open file handles (fileId -> FILE*)
        std::map<uint64_t, FILE*> _openFiles;
        std::map<uint64_t, std::string> _openPaths;  // For directory handles

        // Protocol parsing and handling
        void parseSmbPackets();
        void handleSmbCommand(const uint8_t* packet, size_t len);

        // SMB command handlers (all non-blocking)
        void handleNegotiate(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleSessionSetup(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleLogoff(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleTreeConnect(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleTreeDisconnect(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleCreate(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleClose(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleRead(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleWrite(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleQueryInfo(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleQueryDirectory(struct smb2_header* hdr, const uint8_t* data, size_t len);
        void handleEcho(struct smb2_header* hdr, const uint8_t* data, size_t len);

        // Response building
        void sendResponse(struct smb2_header* hdr, uint32_t status, const uint8_t* data, size_t len);
        void sendErrorResponse(struct smb2_header* hdr, uint32_t status);

        // Helper methods
        uint64_t allocateFileId() { return ++_nextFileId; }
    };

    // ============================================================================
    // Helper Functions
    // ============================================================================

    static void dumpHex(const char* label, const uint8_t* data, size_t len) {
        const size_t maxDump = 256; // Limit dump to first 256 bytes
        size_t dumpLen = len > maxDump ? maxDump : len;
        
        log_debug("SMB: " << label << " (" << len << " bytes):");
        
        for (size_t i = 0; i < dumpLen; i += 16) {
            std::stringstream line;
            line << "  " << std::hex << std::setfill('0') << std::setw(4) << i << ": ";
            
            // Hex bytes
            for (size_t j = 0; j < 16 && (i + j) < dumpLen; j++) {
                line << std::setw(2) << (int)data[i + j] << " ";
            }
            
            // Padding for incomplete lines
            for (size_t j = dumpLen - i; j < 16; j++) {
                line << "   ";
            }
            
            // ASCII representation
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
    // SMBClient Implementation
    // ============================================================================

    SMBClient::SMBClient(SMBServer* server, AsyncClient* client)
        : _server(server), _tcp(client), _disconnected(false), _sessionId(0), _treeId(0), _messageId(0), _nextFileId(1000) {
        _rxBuffer.reserve(8192);  // Reserve initial buffer space

        log_info("SMB: Setting up new client from " << _tcp->remoteIP().toString().c_str());

        // Configure AsyncClient
        _tcp->setNoDelay(true);
        _tcp->setRxTimeout(30);      // 30 seconds RX timeout
        _tcp->setAckTimeout(5000);   // 5 seconds ACK timeout

        // Set up AsyncClient callbacks - MUST be set before any data can arrive
        _tcp->onData(
            [](void* arg, AsyncClient* c, void* data, size_t len) {
                static_cast<SMBClient*>(arg)->onTcpData(arg, c, data, len);
            },
            this);

        _tcp->onDisconnect(
            [](void* arg, AsyncClient* c) {
                static_cast<SMBClient*>(arg)->onTcpDisconnect(arg, c);
            },
            this);

        _tcp->onError(
            [](void* arg, AsyncClient* c, int8_t error) {
                static_cast<SMBClient*>(arg)->onTcpError(arg, c, error);
            },
            this);

        // Set timeout callback
        _tcp->onTimeout(
            [](void* arg, AsyncClient* c, uint32_t time) {
                log_warn("SMB: Client timeout after " << time << "ms");
                static_cast<SMBClient*>(arg)->_disconnected = true;
            },
            this);

        log_info("SMB: Client setup complete, state=" << (int)_tcp->state() 
                 << " connected=" << _tcp->connected() 
                 << " canSend=" << _tcp->canSend()
                 << " space=" << _tcp->space());
    }

    SMBClient::~SMBClient() {
        log_debug("SMB: Destroying SMBClient");
        
        // Close all open files
        for (auto& entry : _openFiles) {
            if (entry.second) {
                fclose(entry.second);
            }
        }
        _openFiles.clear();
        _openPaths.clear();

        if (_tcp) {
            _tcp->close();
            delete _tcp;
            _tcp = nullptr;
        }
    }

    void SMBClient::onTcpData(void* arg, AsyncClient* client, void* data, size_t len) {
        log_debug("SMB: Received " << len << " bytes");
        
        // Dump received data
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        dumpHex("RX", bytes, len);
        
        // Append to receive buffer
        _rxBuffer.insert(_rxBuffer.end(), bytes, bytes + len);

        // Acknowledge receipt immediately to keep TCP window moving
        // This MUST be done before any processing that might take time
        client->ack(len);

        // Try to parse complete SMB packets
        try {
            parseSmbPackets();
        } catch (...) {
            log_error("SMB: Exception in parseSmbPackets");
            _disconnected = true;
        }
    }

    void SMBClient::onTcpDisconnect(void* arg, AsyncClient* client) {
        log_info("SMB: Client disconnected");
        _disconnected = true;
    }

    void SMBClient::onTcpError(void* arg, AsyncClient* client, int8_t error) {
        log_error("SMB: TCP error " << (int)error);
        _disconnected = true;
    }

    void SMBClient::parseSmbPackets() {
        // SMB2 packet structure: NetBIOS Session Message (4 bytes) + SMB2 header (64 bytes) + data
        // For direct TCP (port 445), the NetBIOS header is still present but simplified

        while (_rxBuffer.size() >= 4) {
            // Check for NetBIOS Session Message header (first byte should be 0x00)
            if (_rxBuffer[0] != 0x00) {
                log_error("SMB: Invalid NetBIOS header: 0x" << String(_rxBuffer[0], HEX).c_str());
                _disconnected = true;
                return;
            }

            // Get packet length (24-bit big-endian in bytes 1-3)
            uint32_t packetLen = (_rxBuffer[1] << 16) | (_rxBuffer[2] << 8) | _rxBuffer[3];

            log_debug("SMB: Packet length: " << packetLen << ", buffer size: " << _rxBuffer.size());

            // Need complete packet
            if (_rxBuffer.size() < 4 + packetLen) {
                log_debug("SMB: Waiting for more data");
                return;  // Wait for more data
            }

            // Extract packet (skip NetBIOS header)
            const uint8_t* packet = _rxBuffer.data() + 4;

            // Handle the SMB command
            try {
                handleSmbCommand(packet, packetLen);
            } catch (...) {
                log_error("SMB: Exception in handleSmbCommand");
                _disconnected = true;
                return;
            }

            // Remove processed packet from buffer
            _rxBuffer.erase(_rxBuffer.begin(), _rxBuffer.begin() + 4 + packetLen);
        }
    }

    void SMBClient::handleSmbCommand(const uint8_t* packet, size_t len) {
        if (len < 64) {  // SMB2 header is 64 bytes
            log_error("SMB: Packet too short");
            return;
        }

        // Parse SMB2 header manually (we'll use libsmb2 structures but parse ourselves for now)
        struct smb2_header hdr;
        memset(&hdr, 0, sizeof(hdr));

        // Check protocol ID
        if (memcmp(packet, "\xFE\x53\x4D\x42", 4) != 0 &&  // SMB2: 0xFE 'S' 'M' 'B'
            memcmp(packet, "\xFF\x53\x4D\x42", 4) != 0) {  // SMB1: 0xFF 'S' 'M' 'B'
            log_error("SMB: Invalid protocol ID");
            return;
        }

        // Parse header fields (little-endian)
        memcpy(hdr.protocol_id, packet, 4);
        hdr.struct_size         = *(uint16_t*)(packet + 4);
        hdr.credit_charge       = *(uint16_t*)(packet + 6);
        hdr.status              = *(uint32_t*)(packet + 8);
        hdr.command             = *(uint16_t*)(packet + 12);
        hdr.credit_request_response = *(uint16_t*)(packet + 14);
        hdr.flags               = *(uint32_t*)(packet + 16);
        hdr.next_command        = *(uint32_t*)(packet + 20);
        hdr.message_id          = *(uint64_t*)(packet + 24);
        hdr.sync.process_id     = *(uint32_t*)(packet + 32);
        hdr.sync.tree_id        = *(uint32_t*)(packet + 36);
        hdr.session_id          = *(uint64_t*)(packet + 40);
        memcpy(hdr.signature, packet + 48, 16);

        // Dispatch to appropriate handler based on command
        const uint8_t* cmdData = packet + 64;  // Data starts after header
        size_t         cmdLen  = len - 64;

        switch (hdr.command) {
        case SMB2_NEGOTIATE:
            handleNegotiate(&hdr, cmdData, cmdLen);
            break;
        case SMB2_SESSION_SETUP:
            handleSessionSetup(&hdr, cmdData, cmdLen);
            break;
        case SMB2_LOGOFF:
            handleLogoff(&hdr, cmdData, cmdLen);
            break;
        case SMB2_TREE_CONNECT:
            handleTreeConnect(&hdr, cmdData, cmdLen);
            break;
        case SMB2_TREE_DISCONNECT:
            handleTreeDisconnect(&hdr, cmdData, cmdLen);
            break;
        case SMB2_CREATE:
            handleCreate(&hdr, cmdData, cmdLen);
            break;
        case SMB2_CLOSE:
            handleClose(&hdr, cmdData, cmdLen);
            break;
        case SMB2_READ:
            handleRead(&hdr, cmdData, cmdLen);
            break;
        case SMB2_WRITE:
            handleWrite(&hdr, cmdData, cmdLen);
            break;
        case SMB2_QUERY_INFO:
            handleQueryInfo(&hdr, cmdData, cmdLen);
            break;
        case SMB2_QUERY_DIRECTORY:
            handleQueryDirectory(&hdr, cmdData, cmdLen);
            break;
        case SMB2_ECHO:
            handleEcho(&hdr, cmdData, cmdLen);
            break;
        default:
            log_warn("SMB: Unsupported command " << hdr.command);
            sendErrorResponse(&hdr, 0xC00000BB);  // STATUS_NOT_SUPPORTED
            break;
        }
    }

    void SMBClient::handleNegotiate(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: NEGOTIATE");

        // Build negotiate response
        // Response structure: StructureSize(2) + SecurityMode(2) + DialectRevision(2) + NegotiateContextCount(2) +
        //                     ServerGuid(16) + Capabilities(4) + MaxTransactSize(4) + MaxReadSize(4) + MaxWriteSize(4) +
        //                     SystemTime(8) + ServerStartTime(8) + SecurityBufferOffset(2) + SecurityBufferLength(2)
        std::vector<uint8_t> response(65);
        uint8_t*             resp = response.data();

        *(uint16_t*)(resp + 0) = 65;      // Structure size
        *(uint16_t*)(resp + 2) = 0x01;    // Security mode: SMB2_NEGOTIATE_SIGNING_ENABLED
        *(uint16_t*)(resp + 4) = 0x0311;  // Dialect: SMB 3.1.1
        *(uint16_t*)(resp + 6) = 0;       // NegotiateContextCount

        // Server GUID (random, could be made persistent)
        memset(resp + 8, 0xAA, 16);

        *(uint32_t*)(resp + 24) = 0x0000001F;  // Capabilities: DFS
        *(uint32_t*)(resp + 28) = 0x00100000;  // MaxTransactSize: 1MB
        *(uint32_t*)(resp + 32) = 0x00100000;  // MaxReadSize: 1MB
        *(uint32_t*)(resp + 36) = 0x00100000;  // MaxWriteSize: 1MB

        // SystemTime and ServerStartTime (Windows FILETIME format)
        uint64_t currentTime = 0;  // TODO: Convert current time to FILETIME
        *(uint64_t*)(resp + 40) = currentTime;
        *(uint64_t*)(resp + 48) = currentTime;

        *(uint16_t*)(resp + 56) = 0;  // SecurityBufferOffset
        *(uint16_t*)(resp + 58) = 0;  // SecurityBufferLength

        // Reserved
        memset(resp + 60, 0, 4);

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleSessionSetup(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: SESSION_SETUP");

        // For anonymous/guest access, we'll accept any session setup
        // Assign a session ID if we don't have one
        if (_sessionId == 0) {
            _sessionId = 0x0000100000000001ULL;  // Fixed session ID
        }

        // Build session setup response
        // Response structure: StructureSize(2) + SessionFlags(2) + SecurityBufferOffset(2) + SecurityBufferLength(2)
        std::vector<uint8_t> response(9);
        uint8_t*             resp = response.data();

        *(uint16_t*)(resp + 0) = 9;   // Structure size
        *(uint16_t*)(resp + 2) = 0;   // SessionFlags: none
        *(uint16_t*)(resp + 4) = 72;  // SecurityBufferOffset (header + this struct)
        *(uint16_t*)(resp + 6) = 0;   // SecurityBufferLength (no security blob for anonymous)
        resp[8]                = 0;   // Reserved

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleLogoff(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: LOGOFF");

        // Close session
        _sessionId = 0;

        // Build logoff response (structure size only)
        uint8_t response[4];
        *(uint16_t*)(response + 0) = 4;  // Structure size
        *(uint16_t*)(response + 2) = 0;  // Reserved

        sendResponse(hdr, 0, response, sizeof(response));
    }

    void SMBClient::handleTreeConnect(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: TREE_CONNECT");

        // Parse request to get share path (we'll accept any share name)
        // Request structure: StructureSize(2) + Reserved(2) + PathOffset(2) + PathLength(2)
        if (len < 8) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        // Assign tree ID
        _treeId = 0x00000001;

        // Build tree connect response
        // Response structure: StructureSize(2) + ShareType(1) + Reserved(1) + ShareFlags(4) + Capabilities(4) + MaximalAccess(4)
        std::vector<uint8_t> response(16);
        uint8_t*             resp = response.data();

        *(uint16_t*)(resp + 0) = 16;         // Structure size
        resp[2]                = 0x01;       // ShareType: DISK
        resp[3]                = 0;          // Reserved
        *(uint32_t*)(resp + 4) = 0;          // ShareFlags
        *(uint32_t*)(resp + 8) = 0;          // Capabilities
        *(uint32_t*)(resp + 12) = 0x001F01FF;  // MaximalAccess: full access

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleTreeDisconnect(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: TREE_DISCONNECT");

        // Clear tree ID
        _treeId = 0;

        // Close all open files for this tree
        for (auto& entry : _openFiles) {
            if (entry.second) {
                fclose(entry.second);
            }
        }
        _openFiles.clear();
        _openPaths.clear();

        // Build tree disconnect response
        uint8_t response[4];
        *(uint16_t*)(response + 0) = 4;  // Structure size
        *(uint16_t*)(response + 2) = 0;  // Reserved

        sendResponse(hdr, 0, response, sizeof(response));
    }

    void SMBClient::handleCreate(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: CREATE");

        // Parse CREATE request
        // Structure: StructureSize(2) + SecurityFlags(1) + RequestedOplockLevel(1) + ImpersonationLevel(4) +
        //           SmbCreateFlags(8) + Reserved(8) + DesiredAccess(4) + FileAttributes(4) + ShareAccess(4) +
        //           CreateDisposition(4) + CreateOptions(4) + NameOffset(2) + NameLength(2) + CreateContextsOffset(4) + CreateContextsLength(4)
        if (len < 56) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint16_t nameOffset = *(uint16_t*)(data + 44);
        uint16_t nameLength = *(uint16_t*)(data + 46);
        uint32_t createDisposition = *(uint32_t*)(data + 28);
        uint32_t createOptions = *(uint32_t*)(data + 32);

        // Extract filename (UTF-16LE, convert to UTF-8)
        if (nameOffset < 56 || nameOffset + nameLength > len) {
            sendErrorResponse(hdr, 0xC000000D);
            return;
        }

        std::string filename;
        const uint16_t* utf16Name = (const uint16_t*)(data + nameOffset);
        size_t utf16Len = nameLength / 2;
        for (size_t i = 0; i < utf16Len; i++) {
            uint16_t ch = utf16Name[i];
            if (ch == 0) break;
            if (ch < 128) {
                filename += (char)ch;
            }
            // TODO: Handle full UTF-16 to UTF-8 conversion
        }

        // Replace backslashes with forward slashes
        for (char& c : filename) {
            if (c == '\\') c = '/';
        }

        // Build full path
        std::string fullPath = _server->sharePath() + filename;
        log_debug("SMB: CREATE file: " << fullPath);

        // Check if this is a directory operation
        bool isDirectory = (createOptions & 0x00000001) != 0;  // FILE_DIRECTORY_FILE

        try {
            // Try to get file info
            FluidPath path(fullPath.c_str(), "sd");
            struct stat st;
            bool exists = (stat(path.c_str(), &st) == 0);

            uint64_t fileId = allocateFileId();
            FILE* file = nullptr;

            if (isDirectory) {
                // Directory access
                if (!exists) {
                    sendErrorResponse(hdr, 0xC0000034);  // STATUS_OBJECT_NAME_NOT_FOUND
                    return;
                }
                if (!S_ISDIR(st.st_mode)) {
                    sendErrorResponse(hdr, 0xC0000103);  // STATUS_NOT_A_DIRECTORY
                    return;
                }
                // Store path for directory queries
                _openPaths[fileId] = fullPath;
            } else {
                // File access
                const char* mode = "r+b";  // Read/write by default
                if (!exists) {
                    if (createDisposition == 2 || createDisposition == 4) {  // CREATE or OPEN_IF
                        mode = "w+b";
                    } else {
                        sendErrorResponse(hdr, 0xC0000034);  // STATUS_OBJECT_NAME_NOT_FOUND
                        return;
                    }
                }

                file = fopen(path.c_str(), mode);
                if (!file) {
                    sendErrorResponse(hdr, 0xC0000022);  // STATUS_ACCESS_DENIED
                    return;
                }

                _openFiles[fileId] = file;
                fstat(fileno(file), &st);
            }

            // Build CREATE response
            // Response structure: StructureSize(2) + OplockLevel(1) + Flags(1) + CreateAction(4) + CreationTime(8) +
            //                    LastAccessTime(8) + LastWriteTime(8) + ChangeTime(8) + AllocationSize(8) +
            //                    EndOfFile(8) + FileAttributes(4) + Reserved2(4) + FileId(16) + CreateContextsOffset(4) + CreateContextsLength(4)
            std::vector<uint8_t> response(89);
            uint8_t* resp = response.data();

            *(uint16_t*)(resp + 0) = 89;  // Structure size
            resp[2] = 0;  // OplockLevel: NONE
            resp[3] = 0;  // Flags
            *(uint32_t*)(resp + 4) = exists ? 1 : 2;  // CreateAction: FILE_OPENED or FILE_CREATED

            // Timestamps (TODO: Convert Unix time to Windows FILETIME)
            *(uint64_t*)(resp + 8) = 0;   // CreationTime
            *(uint64_t*)(resp + 16) = 0;  // LastAccessTime
            *(uint64_t*)(resp + 24) = 0;  // LastWriteTime
            *(uint64_t*)(resp + 32) = 0;  // ChangeTime

            *(uint64_t*)(resp + 40) = st.st_size;  // AllocationSize
            *(uint64_t*)(resp + 48) = st.st_size;  // EndOfFile

            uint32_t attrs = 0x80;  // FILE_ATTRIBUTE_NORMAL
            if (S_ISDIR(st.st_mode)) {
                attrs = 0x10;  // FILE_ATTRIBUTE_DIRECTORY
            }
            *(uint32_t*)(resp + 56) = attrs;  // FileAttributes

            *(uint32_t*)(resp + 60) = 0;  // Reserved2

            // FileId (persistent and volatile)
            *(uint64_t*)(resp + 64) = fileId;
            *(uint64_t*)(resp + 72) = fileId;

            *(uint32_t*)(resp + 80) = 0;  // CreateContextsOffset
            *(uint32_t*)(resp + 84) = 0;  // CreateContextsLength

            sendResponse(hdr, 0, response.data(), response.size());

        } catch (...) {
            sendErrorResponse(hdr, 0xC0000225);  // STATUS_DEVICE_NOT_READY (SD card issue)
        }
    }

    void SMBClient::handleClose(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: CLOSE");

        // Parse CLOSE request
        // Structure: StructureSize(2) + Flags(2) + Reserved(4) + FileId(16)
        if (len < 24) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint64_t fileId = *(uint64_t*)(data + 8);

        // Close file if open
        auto fileIt = _openFiles.find(fileId);
        if (fileIt != _openFiles.end() && fileIt->second) {
            fclose(fileIt->second);
            _openFiles.erase(fileIt);
        }

        // Remove directory path if stored
        _openPaths.erase(fileId);

        // Build CLOSE response
        // Response structure: StructureSize(2) + Flags(2) + Reserved(4) + CreationTime(8) + LastAccessTime(8) +
        //                    LastWriteTime(8) + ChangeTime(8) + AllocationSize(8) + EndOfFile(8) + FileAttributes(4)
        std::vector<uint8_t> response(60);
        memset(response.data(), 0, response.size());
        *(uint16_t*)(response.data() + 0) = 60;  // Structure size

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleRead(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: READ");

        // Parse READ request
        // Structure: StructureSize(2) + Padding(1) + Flags(1) + Length(4) + Offset(8) + FileId(16) + MinimumCount(4) +
        //           Channel(4) + RemainingBytes(4) + ReadChannelInfoOffset(2) + ReadChannelInfoLength(2)
        if (len < 48) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint32_t length = *(uint32_t*)(data + 4);
        uint64_t offset = *(uint64_t*)(data + 8);
        uint64_t fileId = *(uint64_t*)(data + 16);

        // Limit read size
        if (length > 0x00100000) {  // 1MB max
            length = 0x00100000;
        }

        // Find open file
        auto fileIt = _openFiles.find(fileId);
        if (fileIt == _openFiles.end() || !fileIt->second) {
            sendErrorResponse(hdr, 0xC0000008);  // STATUS_INVALID_HANDLE
            return;
        }

        FILE* file = fileIt->second;

        // Read data
        std::vector<uint8_t> fileData(length);
        fseek(file, offset, SEEK_SET);
        size_t bytesRead = fread(fileData.data(), 1, length, file);
        fileData.resize(bytesRead);

        // Build READ response with data
        // Response structure: StructureSize(2) + DataOffset(1) + Reserved(1) + DataLength(4) + DataRemaining(4) + Reserved2(4) + Buffer(variable)
        std::vector<uint8_t> response(16 + bytesRead);
        uint8_t* resp = response.data();

        *(uint16_t*)(resp + 0) = 17;  // Structure size (not including padding)
        resp[2] = 80;  // DataOffset (header 64 + response struct 16)
        resp[3] = 0;  // Reserved
        *(uint32_t*)(resp + 4) = bytesRead;  // DataLength
        *(uint32_t*)(resp + 8) = 0;  // DataRemaining
        *(uint32_t*)(resp + 12) = 0;  // Reserved2

        // Copy file data
        if (bytesRead > 0) {
            memcpy(resp + 16, fileData.data(), bytesRead);
        }

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleWrite(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: WRITE");

        // Parse WRITE request
        // Structure: StructureSize(2) + DataOffset(2) + Length(4) + Offset(8) + FileId(16) + Channel(4) +
        //           RemainingBytes(4) + WriteChannelInfoOffset(2) + WriteChannelInfoLength(2) + Flags(4)
        if (len < 48) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint16_t dataOffset = *(uint16_t*)(data + 2);
        uint32_t length = *(uint32_t*)(data + 4);
        uint64_t offset = *(uint64_t*)(data + 8);
        uint64_t fileId = *(uint64_t*)(data + 16);

        // Validate data offset and length
        if (dataOffset < 48 || dataOffset + length > len) {
            sendErrorResponse(hdr, 0xC000000D);
            return;
        }

        // Find open file
        auto fileIt = _openFiles.find(fileId);
        if (fileIt == _openFiles.end() || !fileIt->second) {
            sendErrorResponse(hdr, 0xC0000008);  // STATUS_INVALID_HANDLE
            return;
        }

        FILE* file = fileIt->second;

        // Write data
        fseek(file, offset, SEEK_SET);
        size_t bytesWritten = fwrite(data + dataOffset, 1, length, file);
        fflush(file);

        // Build WRITE response
        // Response structure: StructureSize(2) + Reserved(2) + Count(4) + Remaining(4) + WriteChannelInfoOffset(2) + WriteChannelInfoLength(2)
        std::vector<uint8_t> response(17);
        uint8_t* resp = response.data();

        *(uint16_t*)(resp + 0) = 17;  // Structure size
        *(uint16_t*)(resp + 2) = 0;  // Reserved
        *(uint32_t*)(resp + 4) = bytesWritten;  // Count
        *(uint32_t*)(resp + 8) = 0;  // Remaining
        *(uint16_t*)(resp + 12) = 0;  // WriteChannelInfoOffset
        *(uint16_t*)(resp + 14) = 0;  // WriteChannelInfoLength
        resp[16] = 0;  // Padding

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleQueryInfo(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: QUERY_INFO");

        // Parse QUERY_INFO request
        // Structure: StructureSize(2) + InfoType(1) + FileInfoClass(1) + OutputBufferLength(4) + InputBufferOffset(2) +
        //           Reserved(2) + InputBufferLength(4) + AdditionalInformation(4) + Flags(4) + FileId(16)
        if (len < 40) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint8_t  infoType      = data[2];
        uint8_t  fileInfoClass = data[3];
        uint64_t fileId        = *(uint64_t*)(data + 24);

        log_debug("SMB: QUERY_INFO type=" << (int)infoType << " class=" << (int)fileInfoClass);

        // Get file stats
        struct stat st;
        bool        valid = false;

        // Check if it's a regular file
        auto fileIt = _openFiles.find(fileId);
        if (fileIt != _openFiles.end() && fileIt->second) {
            fstat(fileno(fileIt->second), &st);
            valid = true;
        }
        // Or a directory
        else {
            auto pathIt = _openPaths.find(fileId);
            if (pathIt != _openPaths.end()) {
                try {
                    FluidPath path(pathIt->second.c_str(), "sd");
                    if (stat(path.c_str(), &st) == 0) {
                        valid = true;
                    }
                } catch (...) {
                }
            }
        }

        if (!valid) {
            sendErrorResponse(hdr, 0xC0000008);  // STATUS_INVALID_HANDLE
            return;
        }

        // Build response based on info class
        std::vector<uint8_t> infoData;

        if (infoType == 0x01) {  // SMB2_0_INFO_FILE
            if (fileInfoClass == 0x04) {  // FileBasicInformation
                infoData.resize(40);
                memset(infoData.data(), 0, 40);
                // CreationTime, LastAccessTime, LastWriteTime, ChangeTime (8 bytes each) + FileAttributes (4 bytes) + Reserved (4 bytes)
                uint32_t attrs = S_ISDIR(st.st_mode) ? 0x10 : 0x80;  // DIRECTORY or NORMAL
                *(uint32_t*)(infoData.data() + 32) = attrs;
            } else if (fileInfoClass == 0x05) {  // FileStandardInformation
                infoData.resize(24);
                uint8_t* info                 = infoData.data();
                *(uint64_t*)(info + 0)        = st.st_size;  // AllocationSize
                *(uint64_t*)(info + 8)        = st.st_size;  // EndOfFile
                *(uint32_t*)(info + 16)       = 1;           // NumberOfLinks
                info[20]                      = 0;           // DeletePending
                info[21]                      = S_ISDIR(st.st_mode) ? 1 : 0;  // Directory
                *(uint16_t*)(info + 22) = 0;  // Reserved
            } else if (fileInfoClass == 0x12) {  // FileAllInformation
                // This is complex, return basic info
                infoData.resize(104);
                memset(infoData.data(), 0, 104);
                uint8_t* info = infoData.data();
                // Basic info (40 bytes)
                uint32_t attrs = S_ISDIR(st.st_mode) ? 0x10 : 0x80;
                *(uint32_t*)(info + 32) = attrs;
                // Standard info (24 bytes at offset 40)
                *(uint64_t*)(info + 40) = st.st_size;  // AllocationSize
                *(uint64_t*)(info + 48) = st.st_size;  // EndOfFile
                *(uint32_t*)(info + 56) = 1;           // NumberOfLinks
                info[60]                = 0;           // DeletePending
                info[61]                = S_ISDIR(st.st_mode) ? 1 : 0;
            } else {
                // Unsupported info class
                sendErrorResponse(hdr, 0xC00000BB);  // STATUS_NOT_SUPPORTED
                return;
            }
        } else {
            // Unsupported info type
            sendErrorResponse(hdr, 0xC00000BB);
            return;
        }

        // Build QUERY_INFO response
        // Response structure: StructureSize(2) + OutputBufferOffset(2) + OutputBufferLength(4) + Buffer(variable)
        std::vector<uint8_t> response(8 + infoData.size());
        uint8_t*             resp = response.data();

        *(uint16_t*)(resp + 0) = 9;  // Structure size
        *(uint16_t*)(resp + 2) = 72;  // OutputBufferOffset (header + struct)
        *(uint32_t*)(resp + 4) = infoData.size();  // OutputBufferLength

        if (!infoData.empty()) {
            memcpy(resp + 8, infoData.data(), infoData.size());
        }

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleQueryDirectory(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: QUERY_DIRECTORY");

        // Parse QUERY_DIRECTORY request
        // Structure: StructureSize(2) + FileInformationClass(1) + Flags(1) + FileIndex(4) + FileId(16) +
        //           FileNameOffset(2) + FileNameLength(2) + OutputBufferLength(4)
        if (len < 32) {
            sendErrorResponse(hdr, 0xC000000D);  // STATUS_INVALID_PARAMETER
            return;
        }

        uint8_t  fileInfoClass = data[2];
        uint8_t  flags         = data[3];
        uint64_t fileId        = *(uint64_t*)(data + 8);

        // Find directory path
        auto pathIt = _openPaths.find(fileId);
        if (pathIt == _openPaths.end()) {
            sendErrorResponse(hdr, 0xC0000008);  // STATUS_INVALID_HANDLE
            return;
        }

        std::string dirPath = pathIt->second;
        log_debug("SMB: QUERY_DIRECTORY path=" << dirPath);

        // Read directory entries
        std::vector<uint8_t> dirData;

        try {
            FluidPath path(dirPath.c_str(), "sd");
            DIR*      dir = opendir(path.c_str());
            if (!dir) {
                sendErrorResponse(hdr, 0xC0000034);  // STATUS_OBJECT_NAME_NOT_FOUND
                return;
            }

            struct dirent* entry;
            bool           hasEntries = false;

            // Read up to a reasonable number of entries
            int entryCount = 0;
            while ((entry = readdir(dir)) != nullptr && entryCount < 100) {
                // Skip "." and ".."
                if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                    continue;
                }

                // Get file stats
                std::string fullPath = dirPath + "/" + entry->d_name;
                FluidPath   entryPath(fullPath.c_str(), "sd");
                struct stat st;
                if (stat(entryPath.c_str(), &st) != 0) {
                    continue;
                }

                // Build directory entry based on file information class
                // FileIdBothDirectoryInformation (class 0x25) is most common
                size_t entryStart = dirData.size();

                // Convert filename to UTF-16LE
                std::vector<uint16_t> utf16Name;
                for (char c : std::string(entry->d_name)) {
                    utf16Name.push_back((uint16_t)(unsigned char)c);
                }
                size_t nameBytes = utf16Name.size() * 2;

                // Entry structure: NextEntryOffset(4) + FileIndex(4) + CreationTime(8) + LastAccessTime(8) +
                //                 LastWriteTime(8) + ChangeTime(8) + EndOfFile(8) + AllocationSize(8) +
                //                 FileAttributes(4) + FileNameLength(4) + EaSize(4) + ShortNameLength(1) +
                //                 Reserved(1) + ShortName(24) + Reserved2(2) + FileId(8) + FileName(variable)
                size_t entrySize = 104 + nameBytes;
                // Align to 8 bytes
                size_t alignedSize = (entrySize + 7) & ~7;

                dirData.resize(entryStart + alignedSize);
                uint8_t* entryData = dirData.data() + entryStart;
                memset(entryData, 0, alignedSize);

                // Will fill NextEntryOffset later
                *(uint32_t*)(entryData + 0) = 0;  // NextEntryOffset (update later)
                *(uint32_t*)(entryData + 4) = entryCount;  // FileIndex

                // Timestamps (TODO: Convert to Windows FILETIME)
                *(uint64_t*)(entryData + 8)  = 0;  // CreationTime
                *(uint64_t*)(entryData + 16) = 0;  // LastAccessTime
                *(uint64_t*)(entryData + 24) = 0;  // LastWriteTime
                *(uint64_t*)(entryData + 32) = 0;  // ChangeTime

                *(uint64_t*)(entryData + 40) = st.st_size;  // EndOfFile
                *(uint64_t*)(entryData + 48) = st.st_size;  // AllocationSize

                uint32_t attrs = S_ISDIR(st.st_mode) ? 0x10 : 0x80;  // DIRECTORY or NORMAL
                *(uint32_t*)(entryData + 56) = attrs;  // FileAttributes

                *(uint32_t*)(entryData + 60) = nameBytes;  // FileNameLength
                *(uint32_t*)(entryData + 64) = 0;  // EaSize
                entryData[68]                = 0;  // ShortNameLength
                entryData[69]                = 0;  // Reserved
                // ShortName (24 bytes at offset 70)
                // Reserved2 (2 bytes at offset 94)
                *(uint64_t*)(entryData + 96) = fileId + entryCount;  // FileId

                // FileName (UTF-16LE at offset 104)
                memcpy(entryData + 104, utf16Name.data(), nameBytes);

                // Update NextEntryOffset for previous entry
                if (entryCount > 0) {
                    // Find previous entry start
                    size_t prevStart = 0;
                    for (int i = 0; i < entryCount; i++) {
                        uint32_t nextOffset = *(uint32_t*)(dirData.data() + prevStart);
                        if (nextOffset == 0)
                            break;
                        prevStart += nextOffset;
                    }
                    if (prevStart < entryStart) {
                        *(uint32_t*)(dirData.data() + prevStart) = entryStart - prevStart;
                    }
                }

                hasEntries = true;
                entryCount++;
            }

            closedir(dir);

            if (!hasEntries) {
                sendErrorResponse(hdr, 0x80000006);  // STATUS_NO_MORE_FILES
                return;
            }

        } catch (...) {
            sendErrorResponse(hdr, 0xC0000225);  // STATUS_DEVICE_NOT_READY
            return;
        }

        // Build QUERY_DIRECTORY response
        // Response structure: StructureSize(2) + OutputBufferOffset(2) + OutputBufferLength(4) + Buffer(variable)
        std::vector<uint8_t> response(8 + dirData.size());
        uint8_t*             resp = response.data();

        *(uint16_t*)(resp + 0) = 9;  // Structure size
        *(uint16_t*)(resp + 2) = 72;  // OutputBufferOffset
        *(uint32_t*)(resp + 4) = dirData.size();  // OutputBufferLength

        if (!dirData.empty()) {
            memcpy(resp + 8, dirData.data(), dirData.size());
        }

        sendResponse(hdr, 0, response.data(), response.size());
    }

    void SMBClient::handleEcho(struct smb2_header* hdr, const uint8_t* data, size_t len) {
        log_debug("SMB: ECHO");
        // Echo just returns success
        sendResponse(hdr, 0, nullptr, 0);
    }

    void SMBClient::sendResponse(struct smb2_header* hdr, uint32_t status, const uint8_t* data, size_t dataLen) {
        // Build SMB2 response packet
        std::vector<uint8_t> response(4 + 64 + dataLen);  // NetBIOS + header + data

        // NetBIOS Session Message header
        uint32_t smbLen = 64 + dataLen;
        response[0]     = 0x00;
        response[1]     = (smbLen >> 16) & 0xFF;
        response[2]     = (smbLen >> 8) & 0xFF;
        response[3]     = smbLen & 0xFF;

        // SMB2 header
        uint8_t* hdrBytes = response.data() + 4;
        memcpy(hdrBytes, "\xFE\x53\x4D\x42", 4);  // Protocol ID
        *(uint16_t*)(hdrBytes + 4)  = 64;          // Structure size
        *(uint16_t*)(hdrBytes + 6)  = 0;           // Credit charge
        *(uint32_t*)(hdrBytes + 8)  = status;      // Status
        *(uint16_t*)(hdrBytes + 12) = hdr->command;
        *(uint16_t*)(hdrBytes + 14) = 1;  // Credit response
        *(uint32_t*)(hdrBytes + 16) = hdr->flags | SMB2_FLAGS_SERVER_TO_REDIR;
        *(uint32_t*)(hdrBytes + 20) = 0;  // Next command
        *(uint64_t*)(hdrBytes + 24) = hdr->message_id;
        *(uint32_t*)(hdrBytes + 32) = 0;  // Reserved
        *(uint32_t*)(hdrBytes + 36) = hdr->sync.tree_id;
        *(uint64_t*)(hdrBytes + 40) = _sessionId;
        memset(hdrBytes + 48, 0, 16);  // Signature

        // Copy data
        if (data && dataLen > 0) {
            memcpy(response.data() + 4 + 64, data, dataLen);
        }

        // Dump sent data
        dumpHex("TX", response.data(), response.size());

        // Send via AsyncTCP
        if (_tcp && _tcp->connected() && _tcp->canSend()) {
            size_t written = _tcp->write((const char*)response.data(), response.size());
            if (written != response.size()) {
                log_error("SMB: Only wrote " << written << " of " << response.size() << " bytes");
            }
        } else {
            log_error("SMB: Cannot send response - not connected or buffer full");
        }
    }

    void SMBClient::sendErrorResponse(struct smb2_header* hdr, uint32_t status) {
        // Error response has a 9-byte structure size
        uint8_t errorData[9];
        *(uint16_t*)(errorData + 0) = 9;  // Structure size
        errorData[2]                = 0;  // Error context count
        memset(errorData + 3, 0, 6);      // Reserved + byte count

        sendResponse(hdr, status, errorData, sizeof(errorData));
    }

    // ============================================================================
    // SMBServer Implementation
    // ============================================================================

    SMBServer::SMBServer(const char* name) : ConfigurableModule(name) {}

    SMBServer::~SMBServer() {
        deinit();
    }

    void SMBServer::group(Configuration::HandlerBase& handler) {
        handler.item("enable", _enabled);
        handler.item("port", _port, 1, 65535);
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

        // Check for network availability
        extern bool needsNetworkServices;
        if (!needsNetworkServices) {
            log_warn("SMB Server: No network available");
            return;
        }

        log_info("Starting SMB Server on port " << _port);

        // Create mutex for client list protection
        _clientsMutex = xSemaphoreCreateMutex();
        if (!_clientsMutex) {
            log_error("SMB: Failed to create clients mutex");
            return;
        }

        // Create file I/O queue
        _fileIOQueue = xQueueCreate(8, sizeof(FileIORequest*));
        if (!_fileIOQueue) {
            log_error("SMB: Failed to create file I/O queue");
            vSemaphoreDelete(_clientsMutex);
            _clientsMutex = nullptr;
            return;
        }

        // Create file I/O worker task
        if (xTaskCreatePinnedToCore(fileIOWorkerTask, "smb_fileio", 8192, this, 5, &_fileIOTask, 1) != pdPASS) {
            log_error("SMB: Failed to create file I/O task");
            vQueueDelete(_fileIOQueue);
            _fileIOQueue = nullptr;
            vSemaphoreDelete(_clientsMutex);
            _clientsMutex = nullptr;
            return;
        }

        // Create and start AsyncServer
        _asyncServer = new AsyncServer(_port);
        _asyncServer->onClient(
            [](void* arg, AsyncClient* client) {
                static_cast<SMBServer*>(arg)->handleNewClient(client);
            },
            this);
        _asyncServer->begin();

        // Register with mDNS for network discovery
        Mdns::add("_smb", "_tcp", _port);

        log_info("SMB Server started successfully");
    }

    void SMBServer::deinit() {
        // Remove mDNS registration
        Mdns::remove("_smb", "_tcp");

        // Stop accepting new connections
        if (_asyncServer) {
            _asyncServer->end();
            delete _asyncServer;
            _asyncServer = nullptr;
        }

        // Disconnect all clients (with mutex protection)
        if (_clientsMutex) {
            xSemaphoreTake(_clientsMutex, portMAX_DELAY);
        }
        for (auto client : _clients) {
            delete client;
        }
        _clients.clear();
        if (_clientsMutex) {
            xSemaphoreGive(_clientsMutex);
        }

        // Stop file I/O task
        if (_fileIOTask) {
            vTaskDelete(_fileIOTask);
            _fileIOTask = nullptr;
        }

        // Delete queue
        if (_fileIOQueue) {
            vQueueDelete(_fileIOQueue);
            _fileIOQueue = nullptr;
        }

        // Delete mutex
        if (_clientsMutex) {
            vSemaphoreDelete(_clientsMutex);
            _clientsMutex = nullptr;
        }

        log_info("SMB Server stopped");
    }


    void SMBServer::handleNewClient(AsyncClient* client) {
        // Clean up any disconnected clients before checking the limit
        cleanupDisconnectedClients();

        // Lock mutex for client list access
        if (!_clientsMutex || xSemaphoreTake(_clientsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            log_error("SMB: Failed to acquire clients mutex");
            client->close();
            delete client;
            return;
        }

        // Check client limit
        if (_clients.size() >= MAX_CLIENTS) {
            log_warn("SMB: Maximum clients reached, rejecting connection");
            xSemaphoreGive(_clientsMutex);
            client->close();
            delete client;
            return;
        }

        // Create SMBClient wrapper
        SMBClient* smbClient = new SMBClient(this, client);
        _clients.push_back(smbClient);

        log_info("SMB: Client connected, total clients: " << _clients.size());
        
        xSemaphoreGive(_clientsMutex);
    }

    void SMBServer::fileIOWorkerTask(void* param) {
        SMBServer*     server = static_cast<SMBServer*>(param);
        FileIORequest* req    = nullptr;

        log_debug("SMB: File I/O worker task started");

        TickType_t lastCleanup = xTaskGetTickCount();

        while (true) {
            // Try to receive a file I/O request (with timeout)
            if (xQueueReceive(server->_fileIOQueue, &req, pdMS_TO_TICKS(100)) == pdTRUE && req) {
                // Process file I/O request
                FileIOResult result;
                result.requestId = req->requestId;
                result.status    = FileIOResult::SUCCESS;

                // TODO: Implement file operations based on req->type

                // Invoke callback with result
                if (req->callback) {
                    req->callback(result);
                }

                // Clean up request
                delete req;
            }

            // Periodically clean up disconnected clients (every 500ms)
            TickType_t now = xTaskGetTickCount();
            if (now - lastCleanup >= pdMS_TO_TICKS(500)) {
                server->cleanupDisconnectedClients();
                lastCleanup = now;
            }
        }
    }

    void SMBServer::cleanupDisconnectedClients() {
        if (!_clientsMutex || xSemaphoreTake(_clientsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;  // Skip cleanup if we can't get the mutex
        }

        // Clean up disconnected clients
        _clients.erase(std::remove_if(_clients.begin(),
                                       _clients.end(),
                                       [](SMBClient* c) {
                                           if (c->isDisconnected()) {
                                               log_debug("SMB: Cleaning up disconnected client");
                                               delete c;
                                               return true;
                                           }
                                           return false;
                                       }),
                       _clients.end());

        xSemaphoreGive(_clientsMutex);
    }

    // Register the module
    ConfigurableModuleFactory::InstanceBuilder<SMBServer> __attribute__((init_priority(110))) smb_server("smb_server");

}  // namespace WebUI

