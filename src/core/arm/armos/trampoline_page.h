#include "common/common_types.h"

namespace Armos {

constexpr size_t kTrampolinePageSize = 4096;

constexpr int kTrampolineSHM = 3;
constexpr int kTrampolineCommandPipe = 4;
constexpr int kMainMemSHM = 5;

constexpr size_t kTrampolineStackSize = 4096;

// Please keep the layout of this struct simple!
// The trampoline binary does not link with libc++
struct TrampolinePage {
    // Whether ptrace has been attached or not
    bool init;

    // Whether ptrace has been initialized or not
    bool tracing;

    // Whether the guest process is running guest code or the trampoline
    // Set by the host process
    bool in_guest;

    // Address to the trampoline function
    // This is jumped to by ptrace to process command pipe commands and to switch threads
    u32 trampoline_addr;

    // Base of the trampoline stack
    u32 trampoline_stack;

    // Address to the new TLS in guest memory
    u32 guest_tls_addr;

    // Number of commands waiting in the command pipe
    volatile s32 atomic_command_pipe_count;
};



}
