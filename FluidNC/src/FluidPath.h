#pragma once

#include <filesystem>
#include <string>
#include "Driver/localfs.h"

namespace stdfs = std::filesystem;

class FluidPath : public stdfs::path {
public:
    FluidPath(const char* name, const char* fs, std::error_code& ec) noexcept : FluidPath(name, fs, &ec) {}
    FluidPath(const std::string& name, const char* fs, std::error_code& ec) noexcept : FluidPath(name.c_str(), fs, &ec) {}
    FluidPath(const char* name, const char* fs) : FluidPath(name, fs, nullptr) {}
    FluidPath(const std::string& name, const char* fs) : FluidPath(name.c_str(), fs) {}

    ~FluidPath();

    FluidPath() = default;

    FluidPath(const FluidPath& o);             // copy
    FluidPath(FluidPath&& o);                  // move
    FluidPath& operator=(const FluidPath& o);  // copy assignment
    FluidPath& operator=(FluidPath&& o);       // move assignment

    // true if there is something after the mount name.
    // /localfs/foo -> true,  /localfs -> false
    bool hasTail() { return ++(++begin()) != end(); }

private:
    FluidPath(const char* name, const char* fs, std::error_code*);

    static uint32_t _refcnt;
    bool            _isSD = false;
};

#include <Print.h>
inline Print& operator<<(Print& lhs, FluidPath path) {
    lhs.print(path.string().c_str());
    return lhs;
}

// TODO FIXME: We need to move std-ops::space here and rename it. 

#if ESP_IDF_VERSION_MAJOR >= 5

// 'space' is still not fixed in IDF v5. The rest is.

#include "../stdfs/fluidnc_vfs_ops.h"

namespace std::filesystem {
    inline space_info fnc_space(const path& p, error_code& ec) noexcept {
        space_info info = { static_cast<uintmax_t>(-1), static_cast<uintmax_t>(-1), static_cast<uintmax_t>(-1) };
        uint64_t          total, used;
        auto              mount = *(++p.begin());
        if (fluidnc_vfs_stats(mount.c_str(), total, used)) {
            info = space_info { static_cast<uintmax_t>(total), static_cast<uintmax_t>(total - used), static_cast<uintmax_t>(total - used) };
            ec.clear();
            return info;
        }
        ec.assign(errno, std::generic_category());
        return info;
    }
}
#else
namespace std::filesystem {
    inline space_info fnc_space(const path& p, error_code& ec) noexcept {
        return space(p, ec);
    }
}
#endif
