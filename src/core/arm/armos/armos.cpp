#include "armos.h"

#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/user.h>
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
    int pipe_fd = -1;

    TrampolinePage* ts = nullptr;

public:
    GuestContext() = default;

    void Init() {
        struct_fd = AllocateSharedMemory(kTrampolinePageSize);

        ts = reinterpret_cast<TrampolinePage*>(mmap(nullptr, kTrampolinePageSize, PROT_READ | PROT_WRITE, MAP_SHARED, struct_fd, 0));

        LOG_ERROR(Core_ARM11, "Mapped Guest Struct FD {} at {}", struct_fd, (void*)ts);

        int pipes[2];
        pipe(pipes);

        pipe_fd = pipes[1];

        pid = fork();

        if (pid == 0) {
            // Child process
            dup2(struct_fd, kTrampolineSHM);
            dup2(ram_fd, kMainMemSHM);
            dup2(pipes[0], kTrampolineCommandPipe);

            close(struct_fd);
            close(ram_fd);
            close(pipes[0]);
            close(pipes[1]); // Close write end

            char *const argv[] = {
                "libarmos_trampoline",
                nullptr
            };

            execv("libarmos_trampoline.so", argv);
        }

        LOG_ERROR(Core_ARM11, "PID {}", pid);

        ts->latch.WaitHost();

        LOG_ERROR(Core_ARM11, "ATTACH result: {}", ptrace(PTRACE_ATTACH, pid));

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Trap done!");

        int res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        LOG_ERROR(Core_ARM11, "Armos {} {}", ts->trampoline_addr, ts->trampoline_stack);

        // Should be tracing true, init false
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        ts->latch.RaiseHost();
        ts->latch.WaitHost();
        // WaitUntilStop();

        res = ptrace(PTRACE_CONT, pid);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        // Continue();
        // WaitUntilStop();

        // Should be tracing true, init true
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        // Another continue should enter the syscall loop
    }

    void EnterTrampoline() {
        // kill(pid, SIGSTOP);

        // CaptureNextSyscall();
        // WaitUntilStop();

        // Wait for guest to enter the halt function
        ts->latch.WaitHost();

        // Continue guest into getpid loop
        ts->latch.RaiseHost();

        LOG_ERROR(Core_ARM11, "PTrace Interrupt");

        // ptrace(PTRACE_INTERRUPT, pid);
        // kill(pid, SIGSTOP);

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Got stop");

        int res = ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
        LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);

        WaitUntilTrap();

        user_regs regs = {0};

        ptrace(
            PTRACE_GETREGS,
            pid,
            0,
            &regs
        );

        regs.uregs[13] = ts->trampoline_stack + kTrampolineStackSize;
        regs.uregs[15] = ts->trampoline_addr;

        res = ptrace(
            PTRACE_SETREGS,
            pid,
            0,
            &regs
        );

        LOG_ERROR(Core_ARM11, "SETREGS result: {} {}", res, errno);

        res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        WaitUntilTrap();

        LOG_ERROR(Core_ARM11, "Got trap");

        // Coming out of last syscall, just continue

        res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        ts->latch.WaitHost();
        ts->latch.RaiseHost();

        // ts->latch.WaitHost();
        // ts->latch.RaiseHost();

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Got stop");

        res = ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
        LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);

        WaitUntilTrap();
    }

    void CommandMapMemory(u32 shm_offset, u32 virt_addr, u32 size) {
        SendCommand(Command::MapMemory {
            {},
            shm_offset,
            virt_addr,
            size
        });
    }

    void CommandUnmapMemory(u32 virt_addr, u32 size) {
        SendCommand(Command::UnmapMemory {
            {},
            virt_addr,
            size
        });
    }

private:
    void Continue() {
        kill(pid, SIGCONT);
        CaptureNextSyscall();
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
                ptrace(PTRACE_CONT, pid, 0, 0);
            } else {
                LOG_CRITICAL(Core_ARM11, "Got wrong signal: {}", sig);
                break;
            }
        }
    }

    void WaitUntilTrap() {
        int status = 0;
        while (true) {
            int res = waitpid(pid, &status, 0);

            LOG_ERROR(Core_ARM11, "Status: {}, res {}", status, res);

            if (WIFEXITED(status)) {
                LOG_CRITICAL(Core_ARM11, "Process exited: {}", WEXITSTATUS(status));
                abort();
            }

            int sig = WSTOPSIG(status);

            if (sig == SIGTRAP) {
                break;
            } else if (sig == SIGCONT) {
                // ignore
                int res = ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
                LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);
            } else {
                LOG_CRITICAL(Core_ARM11, "Got wrong signal: {}", sig);
                abort();
            }
        }
    }

    void CaptureNextSyscall() {
        ptrace(PTRACE_SYSCALL, pid);
    }

    void SendCommandRaw(void* data, size_t size) {
        write(pipe_fd, data, size);
    }

    template <typename T>
    void SendCommand(const T &data) {
        auto dat = T::IntoData(data);
        SendCommandRaw(reinterpret_cast<void*>(&dat), sizeof(decltype(dat)));
        __sync_fetch_and_add(&ts->atomic_command_pipe_count, 1);
    }
};

void Armos::Init() {
    ram_fd_size = Memory::FCRAM_N3DS_SIZE + Memory::VRAM_SIZE + Memory::N3DS_EXTRA_RAM_SIZE;
    ram_fd = AllocateSharedMemory(ram_fd_size);
    ram_bump_offset = 0;
    ram_mapping = mmap(nullptr, ram_fd_size, PROT_READ | PROT_WRITE, MAP_SHARED, ram_fd, 0);

    GuestContext ctx;

    ctx.Init();
    ctx.CommandMapMemory(0, 0x10000, 0x1000);
    ctx.EnterTrampoline();

    exit(1);
}

Armos::Region Armos::AllocateRegion(u32 size) {
    uintptr_t offset = ram_bump_offset;
    ram_bump_offset += size;
    return Region(reinterpret_cast<u8*>(ram_mapping) + offset, offset);
}
