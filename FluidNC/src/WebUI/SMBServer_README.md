# SMBServer Module - Implementation Summary

## Overview
The SMBServer module provides read-write file sharing over SMB2/3 protocol for FluidNC, backed by SD card storage. It uses a fully callback-driven, non-blocking architecture to avoid delays in motion control operations.

## Configuration
Add to your FluidNC YAML configuration:

```yaml
smb_server:
  enable: true              # Enable/disable SMB server
  port: 445                 # TCP port (default: 445)
  server_name: FluidNC      # Server name for network discovery
  share_path: /             # Root path on SD card to share (default: /)
```

## Features

### SMB Protocol Support
- **SMB2/SMB3 Negotiation** - Protocol version negotiation
- **Session Management** - Anonymous/guest access (no authentication required)
- **Tree Connect** - Share connection management
- **File Operations**:
  - CREATE - Open/create files and directories
  - CLOSE - Close files
  - READ - Read file data
  - WRITE - Write file data
- **Metadata Operations**:
  - QUERY_INFO - Get file/directory information
  - QUERY_DIRECTORY - List directory contents
- **Misc Operations**:
  - ECHO - Keep-alive
  - LOGOFF - Session termination

### Non-Blocking by design
- All protocol handlers are non-blocking
- AsyncTCP callbacks for network I/O
- File I/O isolated to dedicated worker task
- No delays in main motion control loop

## Usage

### Connecting from Windows
1. Open File Explorer
2. Type in address bar: `\\<fluidnc-ip>\share`
   - Or use the mDNS hostname: `\\<hostname>.local\share`
3. Browse and manage files

### Connecting from Linux
```bash
# Mount the share
sudo mount -t cifs //<fluidnc-ip>/share /mnt/fluidnc -o guest,vers=3.0

# Or use smbclient
smbclient //<fluidnc-ip>/share -N

# With mDNS hostname
smbclient //<hostname>.local/share -N
```

### Connecting from macOS
1. Open Finder
2. Look in Network section (server should appear automatically via mDNS)
3. Or press Cmd+K and enter: `smb://<fluidnc-ip>/share`
   - Or with mDNS: `smb://<hostname>.local/share`
   
## Limitations
- **No authentication** - Anonymous access only
- **No encryption** - Data transmitted in clear text
- **Local network use recommended** - Not suitable for internet exposure
- **File system access** - Full read/write access to shared path
- **No SMB signing or encryption**
- **No multi-channel support**
- **No oplocks/leases**
- **ASCII only**
- **No Windows FILETIME conversion (timestamps always zero)**
- **Single share only**
- **No directory creation/deletion via SMB (use SD card directly)**

## Future Enhancements
- [ ] Implement proper UTF-16 to UTF-8 conversion
- [ ] Add Windows FILETIME timestamp conversion
- [ ] Directory creation/deletion operations
- [ ] Locking of files being streamed to g-code parser

## Credits

Implementation based on SMB2/3 protocol specifications and inspired by libsmb2 library structure.

The SMBServer implementation is done by Stefan de Bruijn for FluidNC.

