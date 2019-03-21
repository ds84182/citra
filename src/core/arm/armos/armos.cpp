#include "armos.h"

#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>

#include "common/logging/log.h"
#include "core/arm/armos/trampoline_page.h"

class Armos::GuestContext {
private:
    int struct_fd = -1;
    pid_t pid = -1;

    TrampolinePage* ts = nullptr;

public:
    GuestContext() = default;

    void Init() {
        struct_fd = syscall(__NR_memfd_create, "citra-guest-struct", 0);

        // if (struct_fd < 0) {
        //     // Doesn't support memfd, use shm instead
        //     struct_fd = shm_open("citra-guest-struct", O_CREAT | O_RDWR | O_TRUNC, 0700);
        // }

        // Needs to be a page in size
        ftruncate(struct_fd, kTrampolinePageSize);
        ts = reinterpret_cast<TrampolinePage*>(mmap(nullptr, kTrampolinePageSize, PROT_READ | PROT_WRITE, MAP_SHARED, struct_fd, 0));

        LOG_ERROR(Core_ARM11, "Mapped Guest Struct FD {} at {}", struct_fd, (void*)ts);

        pid = fork();

        if (pid == 0) {
            // Child process
            dup2(struct_fd, kTrampolineSHM);

            char *const argv[] = {
                "libarmos_trampoline",
                nullptr
            };

            execv("libarmos_trampoline.so", argv);
        }

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Armos {} {}", ts->trampoline_addr, ts->trampoline_stack);

        // Should be tracing true, init false
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        Continue();
        WaitUntilStop();

        // Should be tracing true, init true
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        // Another continue should enter the syscall loop
    }

private:
    void Continue() {
        kill(pid, SIGCONT);
    }

    void WaitUntilStop() {
        int status = 0;
        while (true) {
            int res = waitpid(pid, &status, 0);

            if (WIFEXITED(status)) {
                LOG_CRITICAL(Core_ARM11, "Process exited: {}", WEXITSTATUS(status));
                break;
            }

            int sig = WSTOPSIG(status);

            if (sig == SIGSTOP) {
                break;
            } else if (sig == SIGTRAP || sig == SIGCONT) {
                // Ignore
            } else {
                LOG_CRITICAL(Core_ARM11, "Got wrong signal: {}", sig);
                break;
            }

            CaptureNextSyscall();
        }
    }

    void CaptureNextSyscall() {
        ptrace(PTRACE_SYSCALL, pid);
    }
};

void Armos::Init() {
    GuestContext ctx;

    ctx.Init();

    exit(1);
}
