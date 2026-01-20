#pragma once

#include <pthread.h>
#include <sched.h>
#include <cstdint>
#include <vector>


namespace Lib {
class Thread {
public:
    using CallbackPtr = void* (*)(void*);

    Thread() = default;
    ~Thread();

    int start(CallbackPtr callback,
              void* userData = nullptr,
              int cpuCore = -1,
              bool realtime = false,
              int realtimePriority = -1,
              int schedulingPolicy = SCHED_FIFO);
    int join();
    static int joinAll();
    bool isRunning() const { return m_IsRunning_; }

    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    int setCurrentThreadAffinity(int cpuCore);
    int setCurrentThreadRealtime(int priority = -1, int schedulingPolicy = SCHED_FIFO);

private:
    int setThreadAffinity(pthread_t threadHandle, int cpuCore);
    static int setThreadScheduling(pthread_t threadHandle, int schedulingPolicy, int priority);
    static void* threadEntry(void* arg);

    pthread_t m_ThreadHandle_{};
    int m_CpuCore_ = -1;
    CallbackPtr m_Callback_ = nullptr;
    void* m_UserData_ = nullptr;
    bool m_IsRunning_ = false;

    static std::vector<Thread*> runningThreads_;
};
}

#pragma endregion