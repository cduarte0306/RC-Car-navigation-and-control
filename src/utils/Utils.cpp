#include "Utils.hpp"

#include <sys/resource.h>
#include <stdexcept>
#include <execinfo.h>
#include <csignal>
#include <cstring>
#include <unistd.h>


namespace Utils {

    static void crash_handler(int sig, siginfo_t* info, void* /*ucontext*/) {
        // async-signal-safe write
        const char* msg = "\n*** CRASH - Stack trace:\n";
        write(STDERR_FILENO, msg, strlen(msg));

        // Capture up to 32 frames
        void* frames[32];
        int count = backtrace(frames, 32);

        // Write directly to stderr — async-signal-safe
        backtrace_symbols_fd(frames, count, STDERR_FILENO);

        // Re-raise so kernel still writes the core dump
        signal(sig, SIG_DFL);
        raise(sig);
    }

    void install_crash_handler() {
        struct sigaction sa{};
        sa.sa_sigaction = crash_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESETHAND | SA_SIGINFO;

        sigaction(SIGSEGV, &sa, nullptr);
        sigaction(SIGABRT, &sa, nullptr);
        sigaction(SIGFPE,  &sa, nullptr);
        sigaction(SIGBUS,  &sa, nullptr);
        sigaction(SIGTRAP, &sa, nullptr);  // catch your specific signal
    }

    int ConfigCores() {
        struct rlimit rl{};
        if (getrlimit(RLIMIT_CORE, &rl) != 0)
            throw std::runtime_error("getrlimit failed");

        rl.rlim_cur = rl.rlim_max;          // raise soft → hard limit

        if (setrlimit(RLIMIT_CORE, &rl) != 0)
            throw std::runtime_error("setrlimit failed");

        return 0;
    }
}