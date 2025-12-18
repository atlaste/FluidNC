#include "Timer.h"

#include "Platform.h"

#if IS_PLATFORM(HW_Win32 | HW_Win64)

#    ifndef _WINDOWS_
#        define WIN32_LEAN_AND_MEAN

#        include <Windows.h>
#    endif

#endif

namespace Scheduler {
    int64_t Timer::delta_ = 0;

    void Timer::delayMicros(int64_t micros) {
#if IS_PLATFORM(HW_RPI)
        if (micros > 0) {
            bcm2835_delayMicroseconds(uint64_t(micros));
        }
#else
        for (auto dl = Timer::currentTime() + Timer::TicksPerMicroSecond * micros; dl - Timer::currentTime() > 0;) {
            nop();
        }
#endif
    }

    void Timer::delayMillis(int64_t millis) {
#if IS_PLATFORM(HW_RPI)
        if (millis > 0) {
            bcm2835_delay(uint32_t(millis));
        }
#elif IS_PLATFORM(HW_Win32 | HW_Win64)
        if (millis > 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(millis));
        } else {
            delayMicros(millis * 1000LL);
        }
#else
        for (auto dl = Timer::currentTime() + Timer::TicksPerMilliSecond * millis; dl - Timer::currentTime() > 0;) {
            nop();
        }
#endif
    }
}
#if IS_PLATFORM(HW_Win32 | HW_Win64)

namespace
{
    static LONGLONG Frequency()
    {
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        auto f = frequency.QuadPart;
        return f / 1000000;
    }
}

namespace Scheduler {
    int64_t Timer::currentTime() {
        static LONGLONG frequency = Frequency();
        LARGE_INTEGER   StartingTime;
        QueryPerformanceCounter(&StartingTime);
        return StartingTime.QuadPart / frequency;
    }
}

#endif
