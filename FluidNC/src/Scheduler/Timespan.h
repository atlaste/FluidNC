#pragma once

#include "Platform.h"

#include <cstdint>
#include <type_traits>

namespace Scheduler {
    template <int64_t value, int64_t factor>
    struct CompileTimeTimespan {};

    class Timespan {
        int64_t delta_ = 0;

    public:
        constexpr Timespan() noexcept = default;
        constexpr Timespan(int64_t delta) noexcept : delta_(delta) {}

        template <int64_t value, int64_t factor>
        constexpr Timespan(CompileTimeTimespan<value, factor> /*time*/) : delta_(value * factor) {}

        constexpr Timespan(const Timespan&) noexcept            = default;
        constexpr Timespan(Timespan&&) noexcept                 = default;
        constexpr Timespan& operator=(const Timespan&) noexcept = default;
        constexpr Timespan& operator=(Timespan&&) noexcept      = default;

        constexpr int64_t operator()() const noexcept { return delta_; }

        // ... rest of your methods with constexpr where possible ...

        static constexpr Timespan FromHours(const unsigned long long int v) noexcept { return Timespan(int64_t(v) * 3600000000ul); }
        static constexpr Timespan FromMinutes(const unsigned long long int v) noexcept { return Timespan(int64_t(v) * 60000000ul); }
        static constexpr Timespan FromSeconds(const unsigned long long int v) noexcept { return Timespan(int64_t(v) * 1000000ul); }
        static constexpr Timespan FromMilliseconds(const unsigned long long int v) noexcept { return Timespan(int64_t(v) * 1000ul); }
        static constexpr Timespan FromMicroseconds(const unsigned long long int v) noexcept { return Timespan(int64_t(v) * 1ul); }

        ~Timespan() noexcept = default;
    };

    static_assert(std::is_trivially_copyable_v<Timespan>, "Timespan must be trivially copyable");
    static_assert(std::is_trivially_destructible_v<Timespan>, "Timespan must be trivially destructible");
}

constexpr ::Scheduler::Timespan operator"" _hours(const unsigned long long int v) noexcept {
    return ::Scheduler::Timespan::FromHours(v);
}

constexpr ::Scheduler::Timespan operator"" _minutes(const unsigned long long int v) noexcept {
    return ::Scheduler::Timespan::FromMinutes(v);
}

constexpr ::Scheduler::Timespan operator"" _sec(const unsigned long long int v) noexcept {
    return ::Scheduler::Timespan::FromSeconds(v);
}

constexpr ::Scheduler::Timespan operator"" _msec(const unsigned long long int v) noexcept {
    return ::Scheduler::Timespan::FromMilliseconds(v);
}

constexpr ::Scheduler::Timespan operator"" _usec(const unsigned long long int v) noexcept {
    return ::Scheduler::Timespan::FromMicroseconds(v);
}

// I tried very, very, very hard to optimize this away. But the reality is that whatever you try, co_yield 100_msec; will create a temporary
// and that temporary won't be ellided. If we use a template with no fields, the compiler just doesn't have any choice. For a single yield,
// this little macro optimizes 20 bytes in the frame. Stupid, but true...

#define co_delay_sec(x)                                                                                                                    \
    { co_yield ::Scheduler::CompileTimeTimespan<x, 1'000'000>(); }

#define co_delay_msec(x)                                                                                                                   \
    { co_yield ::Scheduler::CompileTimeTimespan<x, 1'000>(); }

#define co_delay_usec(x)                                                                                                                   \
    { co_yield ::Scheduler::CompileTimeTimespan<x, 1>(); }

// end of defines.
