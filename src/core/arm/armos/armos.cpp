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
#include "core/memory.h"

static int AllocateSharedMemory(size_t size) {
    int fd = syscall(__NR_memfd_create, "citra-guest-struct", 0);
    ftruncate(fd, size);
    return fd;
}

static int ram_fd;
static size_t ram_fd_size;
static void *ram_mapping;
static uintptr_t ram_bump_offset;

class Armos::GuestContext {
private:
    int struct_fd = -1;
    pid_t pid = -1;

    TrampolinePage* ts = nullptr;

public:
    GuestContext() = default;

    void Init() {
        struct_fd = AllocateSharedMemory(kTrampolinePageSize);

        ts = reinterpret_cast<TrampolinePage*>(mmap(nullptr, kTrampolinePageSize, PROT_READ | PROT_WRITE, MAP_SHARED, struct_fd, 0));

        LOG_ERROR(Core_ARM11, "Mapped Guest Struct FD {} at {}", struct_fd, (void*)ts);

        pid = fork();

        if (pid == 0) {
            // Child process
            dup2(struct_fd, kTrampolineSHM);
            dup2(ram_fd, kMainMemSHM);

            close(struct_fd);
            close(ram_fd);

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
    ram_fd_size = Memory::FCRAM_N3DS_SIZE + Memory::VRAM_SIZE + Memory::N3DS_EXTRA_RAM_SIZE;
    ram_fd = AllocateSharedMemory(ram_fd_size);
    ram_bump_offset = 0;
    ram_mapping = mmap(nullptr, ram_fd_size, PROT_READ | PROT_WRITE, MAP_SHARED, ram_fd, 0);

    GuestContext ctx;

    ctx.Init();

    exit(1);
}

Armos::Region Armos::AllocateRegion(u32 size) {
    uintptr_t offset = ram_bump_offset;
    ram_bump_offset += size;
    return Region(reinterpret_cast<u8*>(ram_mapping) + offset, offset);
}
