#include "Thread.hpp"

#include <algorithm>
#include <errno.h>
#include <sched.h>

std::vector<Lib::Thread*> Lib::Thread::runningThreads_ = {};

namespace Lib {

namespace {
int resolveSchedulingPolicy(int requestedPolicy) {
    return (requestedPolicy <= 0) ? SCHED_FIFO : requestedPolicy;
}

int resolveRealtimePriority(int policy, int requestedPriority, int& resolvedPriority) {
    const int minPriority = sched_get_priority_min(policy);
    if (minPriority == -1) {
        return (errno != 0) ? errno : EINVAL;
    }
    const int maxPriority = sched_get_priority_max(policy);
    if (maxPriority == -1) {
        return (errno != 0) ? errno : EINVAL;
    }
    const int defaultPriority = (requestedPriority < 0) ? maxPriority : requestedPriority;
    resolvedPriority = std::clamp(defaultPriority, minPriority, maxPriority);
    return 0;
}
}

Thread::~Thread() {
    if (m_IsRunning_) {
        join();
    }
}

int Thread::start(CallbackPtr callback,
                  void* userData,
                  int cpuCore,
                  bool realtime,
                  int realtimePriority,
                  int schedulingPolicy) {
    if (!callback) {
        return -1;
    }

    if (m_IsRunning_) {
        return EBUSY;
    }

    m_CpuCore_ = cpuCore;
    m_Callback_ = callback;
    m_UserData_ = userData;

    pthread_attr_t attr;
    pthread_attr_t* attrPtr = nullptr;
    int resolvedPolicy = resolveSchedulingPolicy(schedulingPolicy);
    sched_param schedParam{};
    if (realtime) {
        attrPtr = &attr;
        const int attrInit = pthread_attr_init(attrPtr);
        if (attrInit != 0) {
            m_Callback_ = nullptr;
            m_UserData_ = nullptr;
            return attrInit;
        }

        const int inheritResult = pthread_attr_setinheritsched(attrPtr, PTHREAD_EXPLICIT_SCHED);
        if (inheritResult != 0) {
            pthread_attr_destroy(attrPtr);
            m_Callback_ = nullptr;
            m_UserData_ = nullptr;
            return inheritResult;
        }

        const int policyResult = pthread_attr_setschedpolicy(attrPtr, resolvedPolicy);
        if (policyResult != 0) {
            pthread_attr_destroy(attrPtr);
            m_Callback_ = nullptr;
            m_UserData_ = nullptr;
            return policyResult;
        }

        int resolvedPriority = 0;
        const int priorityResult = resolveRealtimePriority(resolvedPolicy, realtimePriority, resolvedPriority);
        if (priorityResult != 0) {
            pthread_attr_destroy(attrPtr);
            m_Callback_ = nullptr;
            m_UserData_ = nullptr;
            return priorityResult;
        }
        schedParam.sched_priority = resolvedPriority;
        const int paramResult = pthread_attr_setschedparam(attrPtr, &schedParam);
        if (paramResult != 0) {
            pthread_attr_destroy(attrPtr);
            m_Callback_ = nullptr;
            m_UserData_ = nullptr;
            return paramResult;
        }
    }

    const int createResult = pthread_create(&m_ThreadHandle_, attrPtr, &Thread::threadEntry, this);
    if (attrPtr) {
        pthread_attr_destroy(attrPtr);
    }
    if (createResult != 0) {
        m_Callback_ = nullptr;
        m_UserData_ = nullptr;
        return createResult;
    }

    if (m_CpuCore_ >= 0) {
        setThreadAffinity(m_ThreadHandle_, m_CpuCore_);
    }
    m_IsRunning_ = true;
    runningThreads_.push_back(this);
    return 0;
}

int Thread::join() {
    if (!m_IsRunning_) {
        return 0;
    }

    const int result = pthread_join(m_ThreadHandle_, nullptr);
    if (result == 0) {
        m_IsRunning_ = false;
        m_Callback_ = nullptr;
        m_UserData_ = nullptr;
        m_CpuCore_ = -1;
        runningThreads_.erase(std::remove(runningThreads_.begin(), runningThreads_.end(), this), runningThreads_.end());
    }
    return result;
}

int Thread::joinAll() {
    int lastError = 0;
    for (Thread* thread : runningThreads_) {
        if (!thread) {
            continue;
        }
        const int result = thread->join();
        if (result != 0) {
            lastError = result;
        }
    }
    runningThreads_.clear();
    return lastError;
}


int Thread::setCurrentThreadAffinity(int cpuCore) {
    return setThreadAffinity(pthread_self(), cpuCore);
}

int Thread::setCurrentThreadRealtime(int priority, int schedulingPolicy) {
    return setThreadScheduling(pthread_self(), schedulingPolicy, priority);
}


int Thread::setThreadAffinity(pthread_t threadHandle, int cpuCore) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpuCore, &cpuset);
    return pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset);
}

int Thread::setThreadScheduling(pthread_t threadHandle, int schedulingPolicy, int priority) {
    const int resolvedPolicy = resolveSchedulingPolicy(schedulingPolicy);
    int resolvedPriority = 0;
    const int priorityResult = resolveRealtimePriority(resolvedPolicy, priority, resolvedPriority);
    if (priorityResult != 0) {
        return priorityResult;
    }
    sched_param schedParam{};
    schedParam.sched_priority = resolvedPriority;
    return pthread_setschedparam(threadHandle, resolvedPolicy, &schedParam);
}


void* Thread::threadEntry(void* arg) {
    auto* self = static_cast<Thread*>(arg);
    if (!self || !self->m_Callback_) {
        return nullptr;
    }
    void* result = self->m_Callback_(self->m_UserData_);
    self->m_IsRunning_ = false;
    return result;
}
}