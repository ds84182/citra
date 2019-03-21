#include <signal.h>

#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/types.h>

#include <unistd.h>

#include "core/arm/armos/trampoline_page.h"

static Armos::TrampolinePage* ts = nullptr;
static void *ipc_scratch_ram = nullptr;

static void halt() {
    // Ready to go, stop again to allow the host to redirect execution
    raise(SIGSTOP);

    // Go into an infinite loop with a syscall
    while (true) {
        getpid();
    }
}

static void set_tls(u32 tls) {
    u32 a1 = 0;

    u32 tls_addr __asm__ ("r0") = tls;
    u32 syscall_no __asm__ ("r7") = 0x0f0005;
    asm volatile(
        "swi 0x0"
        : "+r"(tls_addr)
        : "r"(syscall_no) /* __ARM_NR_set_tls */
        : "memory"
    );
}

static void trampoline() {
    // While in trampoline, reduce risk of clobbering guest's TLS
    set_tls(0);

    // Process command pipe
    while (true) {
        s32 prev = __sync_fetch_and_sub(&ts->atomic_command_pipe_count, 1);
        if (prev > 0) {
            // Process command
        } else {
            // Atomic value is now -1, reset to 0
            __sync_fetch_and_add(&ts->atomic_command_pipe_count, 1);
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

    ipc_scratch_ram = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    // Init ptrace

    ptrace(PTRACE_TRACEME);
    ts->tracing = true;

    // Wait for attach
    raise(SIGSTOP);
    ts->init = true;

    // Do other initialization here, if needed

    halt();
}
