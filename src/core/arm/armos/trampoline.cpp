#include <signal.h>

#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/types.h>

#include <unistd.h>

#include <cstdio>

#include "core/arm/armos/trampoline_page.h"

static Armos::TrampolinePage* ts = nullptr;
static u8 *ipc_scratch_ram = nullptr;

static void halt() {
    // Ready to go, stop again to allow the host to redirect execution

    // Go into an infinite loop with a syscall
    raise(SIGSTOP);
    constexpr char kInTrampolineMessage[] = "Waiting for syscall capture\n";
    write(0, kInTrampolineMessage, sizeof(kInTrampolineMessage));
    while (true) raise(SIGCHLD);
}

static void set_tls(u32 tls) {
    syscall(0x0f0005, tls); /* __ARM_NR_set_tls */
}

template <size_t N>
static void output(const char (&data)[N]) {
    write(0, data, N - 1);
}

struct CommandHandler {
    void operator()(Armos::Command::MapMemory *cmd) const {
        output("Command::MapMemory\n");

        // printf("%08X %08X %08X\n", cmd->virt_addr, cmd->size, cmd->shm_offset);

        void *res = mmap(reinterpret_cast<void*>(cmd->virt_addr), cmd->size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_SHARED, Armos::kMainMemSHM, cmd->shm_offset);

        if (res == MAP_FAILED) {
            output("Warning: MapMemory failed\n");
        }
    }

    void operator()(Armos::Command::UnmapMemory *cmd) const {
        output("Command::UnmapMemory\n");

        int res = munmap(reinterpret_cast<void*>(cmd->virt_addr), cmd->size);
        if (res < 0) {
            output("Warning: UnmapMemory failed\n");
        }
    }
};

static void trampoline() {
    constexpr char kInTrampolineMessage[] = "In trampoline\n";
    write(0, kInTrampolineMessage, sizeof(kInTrampolineMessage));

    // While in trampoline, reduce risk of clobbering guest's TLS
    set_tls(0);

    // Process command pipe
    while (true) {
        s32 prev = __sync_fetch_and_sub(&ts->atomic_command_pipe_count, 1);
        if (prev > 0) {
            // Process command
            output("Process command\n");

            read(Armos::kTrampolineCommandPipe, ipc_scratch_ram, 4);

            u8 command_id = ipc_scratch_ram[0];
            u32 command_size = ipc_scratch_ram[1] | (static_cast<u32>(ipc_scratch_ram[2]) << 8) | (static_cast<u32>(ipc_scratch_ram[3]) << 16);

            if (command_size > (4096 - 4)) {
                output("Command payload too large\n");
                raise(SIGABRT);
            }

            // Read command payload into IPC scratch ram
            read(Armos::kTrampolineCommandPipe, ipc_scratch_ram + 4, command_size);

            // Match command and run handler
            bool applied = Armos::Command::All::Apply(CommandHandler(), command_id, command_size, ipc_scratch_ram);

            if (!applied) {
                output("Recieved invalid command\n");
                raise(SIGABRT);
            }
        } else {
            // Atomic value is now -1, reset to 0
            __sync_fetch_and_add(&ts->atomic_command_pipe_count, 1);
            break;
        }
    }

    // Enter guest
    set_tls(ts->guest_tls_addr);
    halt();
}

extern "C" void _start(int argc, char *argv[]) {
    // Map in the trampoline struct
    ts = reinterpret_cast<Armos::TrampolinePage*>(mmap(nullptr, Armos::kTrampolinePageSize, PROT_READ | PROT_WRITE, MAP_SHARED, Armos::kTrampolineSHM, 0));

    // Initialize the trampoline struct
    ts->trampoline_addr = static_cast<u32>(reinterpret_cast<uintptr_t>(&trampoline));
    ts->trampoline_stack = static_cast<u32>(reinterpret_cast<uintptr_t>(mmap(nullptr, Armos::kTrampolineStackSize, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0)));

    ipc_scratch_ram = reinterpret_cast<u8*>(mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Init ptrace

    ts->tracing = true;

    constexpr char kInTrampolineMessage[] = "Tracing\n";
    write(0, kInTrampolineMessage, sizeof(kInTrampolineMessage));

    // Wait for attach
    raise(SIGSTOP);

    ts->init = true;

    constexpr char kInTrampolineMessage2[] = "Init\n";
    write(0, kInTrampolineMessage2, sizeof(kInTrampolineMessage2));

    raise(SIGSTOP);

    // Do other initialization here, if needed

    halt();
}
