#include "armos.h"

#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/wait.h>

#include <unistd.h>

#include "common/logging/log.h"
#include "core/arm/armos/trampoline_page.h"

void Armos::Init() {
    int guest_struct_fd = syscall(__NR_memfd_create, "citra-guest-struct", 0);
    // Needs to be a page in size
    ftruncate(guest_struct_fd, kTrampolinePageSize);
    TrampolinePage* ts = reinterpret_cast<TrampolinePage*>(mmap(nullptr, kTrampolinePageSize, PROT_READ | PROT_WRITE, MAP_SHARED, guest_struct_fd, 0));

    LOG_WARNING(HW_Memory, "Mapped Guest Struct FD {} at {}", guest_struct_fd, ts);

    pid_t pid = fork();

    if (pid == 0) {
        // Child process
        dup2(guest_struct_fd, kTrampolineSHM);

        char *const argv[] = {
            "libarmos_trampoline",
            nullptr
        };

        execv("libarmos_trampoline.so", argv);
    }

    // Wait until stop

    // wait_until_stop
    {
        int status = 0;
        while (true) {
            int res = waitpid(pid, &status, 0);

            if (WIFEXITED(status)) {
                LOG_ERROR(HW_Memory, "Process exited: {}", WEXITSTATUS(status));
                break;
            }

            int sig = WSTOPSIG(status);

            if (sig == SIGSTOP) {
                break;
            } else if (sig == SIGTRAP || sig == SIGCONT) {
                // Ignore
            } else {
                LOG_ERROR(HW_Memory, "Got wrong signal: {}", sig);
            }

            // capture_next_syscall
            ptrace(PTRACE_SYSCALL, pid);
        }
    }

    LOG_WARNING(HW_Memory, "Armos {} {}", ts->trampoline_addr, ts->trampoline_stack);
}
