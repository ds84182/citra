#include "common/common_types.h"

#include <linux/futex.h>

#include <unistd.h>
#include <sys/syscall.h>

static long sys_futex(volatile void *addr1, int op, int val1, struct timespec *timeout, void *addr2, int val3) {
    return syscall(SYS_futex, addr1, op, val1, timeout, addr2, val3);
}

namespace Armos {

constexpr size_t kTrampolinePageSize = 4096;

constexpr int kTrampolineSHM = 3;
constexpr int kTrampolineCommandPipe = 4;
constexpr int kMainMemSHM = 5;

constexpr size_t kTrampolineStackSize = 4096;

namespace Command {
    template <typename T>
    struct Data {
        u8 header[4];
        T data;
    };

    template <u8 Ord, typename This>
    struct Base {
        static constexpr u8 Ordinal = Ord;

        static Data<This> IntoData(const This& data) {
            return Data<This> {
                {
                    Ordinal,
                    sizeof(This) & 0xFF,
                    (sizeof(This) >> 8) & 0xFF,
                    (sizeof(This) >> 16) & 0xFF,
                },
                data
            };
        }
    };

    struct MapMemory : Base<0, MapMemory> {
        u32 shm_offset;
        u32 virt_addr;
        u32 size;
    };

    struct UnmapMemory : Base<1, UnmapMemory> {
        u32 virt_addr;
        u32 size;
    };
}

struct Latch {
    volatile int value;

    void RaiseGuest() {
        WakeGeneric(1);
    }

    void WaitGuest() {
        WaitGeneric(2);
    }

    void RaiseHost() {
        WakeGeneric(2);
    }

    void WaitHost() {
        WaitGeneric(1);
    }

private:
    void WaitGeneric(int val) {
        while (value != val) {
            sys_futex(&value, FUTEX_WAIT, val, nullptr, nullptr, 0);
        }
    }

    void WakeGeneric(int val) {
        value = val;
        while (sys_futex(&value, FUTEX_WAKE, 1, nullptr, nullptr, 0) < 0);
    }
};

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

    Latch latch;
};



}
