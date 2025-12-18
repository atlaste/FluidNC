#pragma once

#include "Event.h"
#include "FixedMemory.h"
#include "Logging.h"
#include "SlowEventScheduler.h"
#include "Platform.h"

#include <coroutine>
#include <cstdint>
#include <exception>
#include <queue>
#include <utility>

namespace Scheduler {
    template <typename T>
    class Schedulable;

    struct PromiseBase;

    // QueueMe for AsyncLock
    template <typename Queue, typename... Args>
    class QueueMe {
        Queue*              queue_;
        std::tuple<Args...> args_;

        template <typename Tuple, size_t... Indices>
        inline void call_detail(Event* evt, Tuple& tuple, std::index_sequence<Indices...>) {
            return queue_->push(evt, std::get<Indices>(tuple)...);
        }

    public:
        QueueMe(Queue& queue, Args... args) : queue_(&queue), args_(args...) {}

        inline void push(Event* evt) { call_detail(evt, args_, std::make_index_sequence<std::tuple_size<std::tuple<Args...>>::value>()); }
    };

    struct CoroutineEventState {
        using ResumeFunction = void (*)(CoroutineEventState*, void*);

        enum Action {
            Continue,
            Delay,
            Call,
            Return,
            Throw,
            Suspend,
        };

        Action             action_     = Continue;
        Timespan           delay_      = 0_msec;
        void*              resumeArg_  = nullptr;
        ResumeFunction     resumeFunc_ = nullptr;
        Event*             event_      = nullptr;
        std::exception_ptr exception_  = nullptr;
    };

    // Base promise type for all coroutines
    struct PromiseBase {
        // PromiseBase() {}

        void*                               callerObj_     = nullptr;  // Who's waiting for us
        CoroutineEventState::ResumeFunction callerInvoker_ = nullptr;
        CoroutineEventState*                stateObj_      = nullptr;  // Set when running

        INLINE std::suspend_always initial_suspend() noexcept { return {}; }
        INLINE std::suspend_always final_suspend() noexcept { return {}; }
        void                       unhandled_exception() noexcept {
            stateObj_->action_    = CoroutineEventState::Throw;
            stateObj_->exception_ = std::current_exception();
        }

        std::suspend_always yield_value(Timespan from) noexcept {
            stateObj_->action_ = CoroutineEventState::Delay;
            stateObj_->delay_  = from;
            return {};
        }

        template <int64_t value, int64_t factor>
        INLINE std::suspend_always yield_value(const CompileTimeTimespan<value, factor>& from) noexcept {
            stateObj_->action_ = CoroutineEventState::Delay;
            stateObj_->delay_  = std::move(from);
            return {};
        }

        struct QueueAwaiter {
            INLINE bool await_ready() const noexcept { return false; }
            INLINE void await_suspend(std::coroutine_handle<>) noexcept {}
            INLINE void await_resume() noexcept {}
        };

        template <typename Queue, typename... Args>
        QueueAwaiter yield_value(QueueMe<Queue, Args...> marker) noexcept {
            stateObj_->action_ = CoroutineEventState::Suspend;
            marker.push(stateObj_->event_);
            return QueueAwaiter {};
        }

        // await_transform for Schedulable lvalue - sets up caller chain
        template <typename U>
        auto& await_transform(Schedulable<U>& tmp) noexcept {
            auto& prom             = tmp.h_.promise();
            prom.callerObj_        = stateObj_->resumeArg_;
            prom.callerInvoker_    = stateObj_->resumeFunc_;
            stateObj_->resumeArg_  = &tmp;
            stateObj_->resumeFunc_ = Schedulable<U>::invoker;
            stateObj_->action_     = CoroutineEventState::Call;

            return tmp;  // Return Schedulable itself as awaiter
        }

        // await_transform for Schedulable rvalue
        template <typename U>
        auto&& await_transform(Schedulable<U>&& tmp) noexcept {
            // Move the temporary into a local variable that will be stored in the coroutine frame
            auto& prom             = tmp.h_.promise();
            prom.callerObj_        = stateObj_->resumeArg_;
            prom.callerInvoker_    = stateObj_->resumeFunc_;
            stateObj_->resumeArg_  = nullptr;  // Will be set in await_suspend.
            stateObj_->resumeFunc_ = Schedulable<U>::invoker;
            stateObj_->action_     = CoroutineEventState::Call;

            return tmp;  // Return Schedulable itself as awaiter
        }

        // scheduler memory pool
        INLINE void* operator new(std::size_t n) noexcept { return FixedMemory::instance().allocate(n); }
        INLINE void  operator delete(void* ptr) noexcept { FixedMemory::instance().deallocate(ptr); }
    };

    template <typename T>
    class Schedulable {
    public:
        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        handle_type h_;

        struct promise_type : public PromiseBase {
            T retValue_;

            INLINE Schedulable get_return_object() noexcept { return Schedulable(handle_type::from_promise(*this)); }

            INLINE static Schedulable get_return_object_on_allocation_failure() { throw std::bad_alloc(); }

            void return_value(T&& from) {
                stateObj_->action_           = CoroutineEventState::Return;
                retValue_                    = std::forward<T>(from);
                this->stateObj_->resumeArg_  = this->callerObj_;
                this->stateObj_->resumeFunc_ = this->callerInvoker_;
            }
            void return_value(const T& from) {
                stateObj_->action_           = CoroutineEventState::Return;
                retValue_                    = from;
                this->stateObj_->resumeArg_  = this->callerObj_;
                this->stateObj_->resumeFunc_ = this->callerInvoker_;
            }
        };

        Schedulable() : h_() {}
        Schedulable(handle_type&& h) : h_(h) {}

        Schedulable(Schedulable&& o) noexcept : h_(o.h_) {
            o.h_ = nullptr;

            PromiseBase& p = h_.promise();
            if (p.stateObj_ != nullptr && p.stateObj_->resumeArg_ == &o) {
                p.stateObj_->resumeArg_ = this;
            }
        }

        Schedulable& operator=(Schedulable&& o) noexcept {
            if (this != &o) {
                if (h_) {
                    h_.destroy();
                }
                h_   = o.h_;
                o.h_ = nullptr;

                PromiseBase& p = h_.promise();
                if (p.stateObj_ != nullptr && p.stateObj_->resumeArg_ == &o) {
                    p.stateObj_->resumeArg_ = this;
                }
            }
            return *this;
        }

        Schedulable(const Schedulable& o)            = delete;
        Schedulable& operator=(const Schedulable& o) = delete;

        static void invoker(CoroutineEventState* state, void* ptr) {
            // Run the iteration:
            auto  sched    = static_cast<Schedulable<T>*>(ptr);
            auto& prom     = sched->h_.promise();
            prom.stateObj_ = state;
            (sched->h_)();
        }

        // Awaiter interface
        INLINE bool await_ready() const noexcept { return false; }
        INLINE void await_suspend(std::coroutine_handle<> caller) noexcept {
            // NOW we know 'this' is in its final location in the coroutine frame:
            auto& callerPromise                 = std::coroutine_handle<PromiseBase>::from_address(caller.address()).promise();
            callerPromise.stateObj_->resumeArg_ = this;
        }
        // INLINE T    await_resume() noexcept { return std::move(h_.promise().retValue_); }
        INLINE T&& await_resume() noexcept { return std::move(h_.promise().retValue_); }

        INLINE T get() const { return std::move(h_.promise().retValue_); }

        ~Schedulable() {
            if (h_) {
                h_.destroy();
            }
        }
    };

    // Specialization for void
    template <>
    class Schedulable<void> {
    public:
        struct promise_type;
        using handle_type = std::coroutine_handle<promise_type>;

        handle_type h_;

        struct promise_type : public PromiseBase {
            INLINE Schedulable get_return_object() { return Schedulable(handle_type::from_promise(*this)); }
            INLINE void        return_void() {
                stateObj_->action_           = CoroutineEventState::Return;
                this->stateObj_->resumeArg_  = this->callerObj_;
                this->stateObj_->resumeFunc_ = this->callerInvoker_;
            }

            INLINE static Schedulable get_return_object_on_allocation_failure() { throw std::bad_alloc(); }
        };

        Schedulable(handle_type&& h) : h_(h) {}

        Schedulable(Schedulable&& o) noexcept : h_(o.h_) {
            o.h_ = nullptr;

            PromiseBase& p = h_.promise();
            if (p.stateObj_ != nullptr && p.stateObj_->resumeArg_ == &o) {
                p.stateObj_->resumeArg_ = this;
            }
        }
        Schedulable& operator=(Schedulable&& o) noexcept {
            if (this != &o) {
                if (h_) {
                    h_.destroy();
                }
                h_   = o.h_;
                o.h_ = nullptr;

                PromiseBase& p = h_.promise();
                if (p.stateObj_ != nullptr && p.stateObj_->resumeArg_ == &o) {
                    p.stateObj_->resumeArg_ = this;
                }
            }
            return *this;
        }

        Schedulable(const Schedulable& o)            = delete;
        Schedulable& operator=(const Schedulable& o) = delete;

        // Awaiter interface
        INLINE bool await_ready() const noexcept { return false; }
        INLINE void await_suspend(std::coroutine_handle<> caller) noexcept {
            // NOW we know 'this' is in its final location in the coroutine frame:
            auto& callerPromise                 = std::coroutine_handle<PromiseBase>::from_address(caller.address()).promise();
            callerPromise.stateObj_->resumeArg_ = this;
        }
        INLINE void await_resume() noexcept {}

        static void invoker(CoroutineEventState* state, void* ptr) {
            // Run the iteration:
            auto  sched    = static_cast<Schedulable<void>*>(ptr);
            auto& prom     = sched->h_.promise();
            prom.stateObj_ = state;

            (sched->h_)();
        }

        ~Schedulable() {
            if (h_) {
                h_.destroy();
            }
        }
    };

    namespace {
        void handle_exception(const std::exception_ptr& ptr) {
            std::string errorMsg = "Unhandled exception.";
            try {
                std::rethrow_exception(ptr);
            } catch (const std::system_error& ex) {
                errorMsg = "Unhandled system error: ";
                errorMsg += ex.what();
            } catch (const std::runtime_error& ex) {
                errorMsg = "Unhandled runtime error: ";
                errorMsg += ex.what();
            } catch (const std::exception& ex) {
                errorMsg = "Unhandled exception: ";
                errorMsg += ex.what();
            } catch (...) {
                // unknown exception type
            }

            // TODO FIXME: Get stack trace.
            log_error(errorMsg);
        }
    }

    // Specialization for void
    class CoroutineEvent : public Event {
        using handle_type = Schedulable<void>::handle_type;

        Schedulable<void>   topLevel_;
        CoroutineEventState state_;

        void resume() { (*state_.resumeFunc_)(&state_, state_.resumeArg_); }

    public:
        CoroutineEvent(Schedulable<void>&& coro) : topLevel_(std::move(coro)) {
            state_.resumeArg_  = &topLevel_;
            state_.resumeFunc_ = &Schedulable<void>::invoker;
            state_.event_      = this;
            state_.action_     = CoroutineEventState::Continue;
        }

        void invoke() override {
            resume();
            switch (state_.action_) {
                case CoroutineEventState::Return:
                    if (state_.resumeArg_ != nullptr) {
                        slowScheduler->schedule(this, Timepoint::now());
                    } else {
                        delete this;
                    }
                    // else: Done.
                    break;
                case CoroutineEventState::Call:
                case CoroutineEventState::Continue:
                    slowScheduler->schedule(this, Timepoint::now());
                    break;
                case CoroutineEventState::Delay:
                    slowScheduler->schedule(this, Timepoint::now() + state_.delay_);
                    break;
                case CoroutineEventState::Throw:
                    handle_exception(state_.exception_);
                    break;
                case CoroutineEventState::Suspend:
                    // no-op
                    break;
            }
        }

        ~CoroutineEvent() = default;
    };

    template <typename T>
    class CoroutineEventWithVoidResult : public Event {
    public:
        using ResultHandler = void (*)(void*);

    private:
        using handle_type = Schedulable<T>::handle_type;

        Schedulable<T>      topLevel_;
        CoroutineEventState state_;
        ResultHandler       handler_;
        void*               userData_;

        void resume() { (*state_.resumeFunc_)(&state_, state_.resumeArg_); }

    public:
        CoroutineEventWithVoidResult(Schedulable<T>&& coro, ResultHandler handler, void* userData) :
            topLevel_(std::move(coro)), handler_(handler), userData_(userData) {
            state_.resumeArg_  = &topLevel_;
            state_.resumeFunc_ = &Schedulable<T>::invoker;
            state_.event_      = this;
            state_.action_     = CoroutineEventState::Continue;
        }

        void invoke() override {
#if IS_PLATFORM(HW_ESP32 | HW_ESP32_S2 | HW_ESP32_S3)
            // log_info("Invoking async action step");
#endif
            resume();
            switch (state_.action_) {
                case CoroutineEventState::Return:
                    if (state_.resumeArg_ != nullptr) {
                        slowScheduler->schedule(this, Timepoint::now());
                    } else {
                        (*handler_)(userData_);
                        delete this;
                    }
                    break;
                case CoroutineEventState::Call:
                case CoroutineEventState::Continue:
                    slowScheduler->schedule(this, Timepoint::now());
                    break;
                case CoroutineEventState::Delay:
                    slowScheduler->schedule(this, Timepoint::now() + state_.delay_);
                    break;
                case CoroutineEventState::Throw:
                    log_info("Handling exception in async action");
                    handle_exception(state_.exception_);
                    break;
                case CoroutineEventState::Suspend:
                    // no-op
                    break;
            }
        }

        ~CoroutineEventWithVoidResult() = default;
    };

    template <typename T>
    class CoroutineEventWithResult : public Event {
    public:
        using ResultHandler = void (*)(T&&, void*);

    private:
        using handle_type = Schedulable<T>::handle_type;

        Schedulable<T>      topLevel_;
        CoroutineEventState state_;
        ResultHandler       handler_;
        void*               userData_;

        void resume() { (*state_.resumeFunc_)(&state_, state_.resumeArg_); }

    public:
        CoroutineEventWithResult(Schedulable<T>&& coro, ResultHandler handler, void* userData) :
            topLevel_(std::move(coro)), handler_(handler), userData_(userData) {
            state_.resumeArg_  = &topLevel_;
            state_.resumeFunc_ = &Schedulable<T>::invoker;
            state_.event_      = this;
            state_.action_     = CoroutineEventState::Continue;
        }

        void invoke() override {
            resume();
            switch (state_.action_) {
                case CoroutineEventState::Return:
                    if (state_.resumeArg_ != nullptr) {
                        slowScheduler->schedule(this, Timepoint::now());
                    } else {
                        (*handler_)(topLevel_.get(), userData_);
                        delete this;
                    }
                    break;
                case CoroutineEventState::Call:
                case CoroutineEventState::Continue:
                    slowScheduler->schedule(this, Timepoint::now());
                    break;
                case CoroutineEventState::Delay:
                    slowScheduler->schedule(this, Timepoint::now() + state_.delay_);
                    break;
                case CoroutineEventState::Throw:
                    handle_exception(state_.exception_);
                    break;
                case CoroutineEventState::Suspend:
                    // no-op
                    break;
            }
        }

        ~CoroutineEventWithResult() = default;
    };

    // schedule() creates CoroutineEvent wrapper
    template <typename T>
    INLINE Event* schedule(Schedulable<T>&& coroutine, void (*handler)(T&&, void*), void* userData) {
        auto evt = new CoroutineEventWithResult(std::move(coroutine), handler, userData);
        slowScheduler->schedule(evt);
        return evt;
    }

    // schedule() creates CoroutineEvent wrapper
    INLINE Event* schedule(Schedulable<void>&& coroutine) {
        auto evt = new CoroutineEvent(std::move(coroutine));
        slowScheduler->schedule(evt);
        return evt;
    }

    // schedule() creates CoroutineEvent wrapper
    INLINE Event* schedule(Schedulable<void>&& coroutine, void (*handler)(void*), void* userData) {
        auto evt = new CoroutineEventWithVoidResult(std::move(coroutine), handler, userData);
        slowScheduler->schedule(evt);
        return evt;
    }

    // AsyncLock for shared resources
    class AsyncLock {
    private:
        bool               locked_ = false;
        std::queue<Event*> waiters_;

    public:
        class AsyncLockGuard {
            AsyncLock* lock_;

        public:
            AsyncLockGuard() : lock_(nullptr) {}
            AsyncLockGuard(AsyncLock* lock) : lock_(lock) {}

            AsyncLockGuard(AsyncLockGuard&& other) noexcept : lock_(other.lock_) { other.lock_ = nullptr; }

            AsyncLockGuard& operator=(AsyncLockGuard&& other) noexcept {
                if (this != &other) {
                    if (lock_) {
                        lock_->unlock();
                    }
                    lock_       = other.lock_;
                    other.lock_ = nullptr;
                }
                return *this;
            }

            AsyncLockGuard(const AsyncLockGuard&)            = delete;
            AsyncLockGuard& operator=(const AsyncLockGuard&) = delete;

            ~AsyncLockGuard() {
                if (lock_) {
                    lock_->unlock();
                }
            }
        };

        Schedulable<AsyncLockGuard> lock() {
            if (locked_) {
                co_yield QueueMe(waiters_);
            }
            locked_ = true;
            co_return AsyncLockGuard(this);
        }

    private:
        void unlock() {
            locked_ = false;

            if (!waiters_.empty()) {
                auto* next_waiter = waiters_.front();
                waiters_.pop();
                locked_ = true;
                slowScheduler->schedule(next_waiter, Timepoint::now());
            }
        }
    };
}
