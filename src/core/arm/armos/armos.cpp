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
#include "core/arm/arm_interface.h"
#include "core/arm/armos/trampoline_page.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/memory.h"
#include "core/hle/kernel/svc.h"

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
    bool tls_dirty = false;
    bool in_guest = false;
    bool downcount_alarm_raised = false;
    bool guest_fault = false;

    std::thread downcount_timer_thread;
    u64 next_downcount_ns = 0;
    std::condition_variable downcount_cond;
    std::mutex downcount_mutex;

    TrampolinePage* ts = nullptr;

public:
    GuestContext() = default;

    ~GuestContext() {
        if (pid != -1) {
            kill(pid, SIGKILL);
        }
    }

    void Init() {
        downcount_timer_thread = std::thread([=]() {
            this->DowncountTimerThread();
        });

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

        // ts->latch.WaitHost();

        LOG_ERROR(Core_ARM11, "ATTACH result: {}", ptrace(PTRACE_ATTACH, pid));

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Trap done!");

        int res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        WaitUntilStop();

        LOG_ERROR(Core_ARM11, "Armos {} {}", ts->trampoline_addr, ts->trampoline_stack);

        // Should be tracing true, init false
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        // ts->latch.RaiseHost();
        // ts->latch.WaitHost();
        // WaitUntilStop();

        res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        // Continue();
        WaitUntilStop();

        // Should be tracing true, init true
        LOG_ERROR(Core_ARM11, "Status: tracing {}, init {}", ts->tracing, ts->init);

        res = ptrace(PTRACE_CONT, pid, 0, 0);
        LOG_ERROR(Core_ARM11, "CONT result: {}", res);

        WaitUntilStop();

        // Enter syscall

        res = ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
        LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);

        WaitUntilTrap();

        // Exit syscall

        res = ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
        LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);

        WaitUntilTrap();
    }

    void EnterTrampoline() {
        tls_dirty = false;
        in_guest = false;

        DowncountThreadSignal();

        user_regs regs = {0};

        ptrace(
            PTRACE_GETREGS,
            pid,
            0,
            &regs
        );

        regs.uregs[13] = ts->trampoline_stack + kTrampolineStackSize;
        regs.uregs[15] = ts->trampoline_addr;

        constexpr u32 kThumbBit = 0x20;

        if (ts->trampoline_addr & 1) {
            // Thumb mode trampoline
            regs.uregs[16] |= kThumbBit;
        } else {
            // ARM mode trampoline
            regs.uregs[16] &= ~kThumbBit;
        }

        int res = ptrace(
            PTRACE_SETREGS,
            pid,
            0,
            &regs
        );

        // Coming out of last syscall, just continue
        auto init_status = __sync_fetch_and_add(&ts->trampoline_status, 0);
        int new_status = init_status;

        while (init_status == (new_status = __sync_fetch_and_add(&ts->trampoline_status, 0))) {
            res = ptrace(PTRACE_CONT, pid, 0, 0);

            WaitUntilStop();
        }
    }

    // Sometimes not declared in user.h
    struct user_vfp {
        unsigned long long fpregs[32];
        unsigned long fpscr;
    };

    void SetContext(const std::array<u32, 16>& reg, u32 cpsr, const std::array<u32, 64>& fp_regs, u32 fpscr, u32 fpexc) {
        user_regs regs = {0};

        for (int i=0; i<16; i++) {
            regs.uregs[i] = reg[i];
        }
        regs.uregs[17] = regs.uregs[0];

        regs.uregs[16] = cpsr;

        user_vfp vfp_regs = {0};

        memcpy(&vfp_regs.fpregs, &fp_regs, sizeof(u32) * 64);
        vfp_regs.fpscr = fpscr;

        int res = ptrace(PTRACE_SETREGS, pid, 0, &regs);

        if (res < 0) perror("Failed to PTRACE_SETREGS");

        res = ptrace(PTRACE_SETVFPREGS, pid, 0, &vfp_regs);

        if (res < 0) perror("Failed to PTRACE_SETVFPREGS");
    }

    void GetContext(std::array<u32, 16>& reg, u32& cpsr, std::array<u32, 64>& fp_regs, u32& fpscr, u32& fpexc, bool from_swi) {
        user_regs regs = {0};
        user_vfp vfp_regs = {0};

        int res = ptrace(PTRACE_GETREGS, pid, 0, &regs);

        if (res < 0) perror("Failed to PTRACE_GETREGS");

        res = ptrace(PTRACE_GETVFPREGS, pid, 0, &vfp_regs);

        if (res < 0) perror("Failed to PTRACE_GETVFPREGS");

        for (int i=1; i<16; i++) {
            reg[i] = regs.uregs[i];
        }
        reg[0] = regs.uregs[0];
        cpsr = regs.uregs[16];

        memcpy(&fp_regs, &vfp_regs.fpregs, sizeof(u32) * 64);
        fpscr = vfp_regs.fpscr;
    }

    void SetTLS(u32 tls) {
        if (ts->guest_tls_addr != tls) {
            ts->guest_tls_addr = tls;
            tls_dirty = true;
        }
    }

    void SetDowncount(u32 ns) {
        next_downcount_ns = ns;
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

    void CommandTrapMemory(u32 virt_addr, u32 size) {
        SendCommand(Command::TrapMemory {
            {},
            virt_addr,
            size
        });
    }

    enum class SyscallResult {
        Syscall,
        DowncountAlarm,
        GuestFault
    };

    SyscallResult EmulateSyscalls() {
        in_guest = true;
        downcount_alarm_raised = false;
        DowncountThreadSignal();

        CaptureNextSyscall();
        WaitUntilTrap();

        // Disable downcount alarm and other code that triggers while executing guest code.
        // We need fine tuned control of the code below.
        in_guest = false;
        DowncountThreadSignal();

        if (downcount_alarm_raised) {
            return SyscallResult::DowncountAlarm;
        }

        if (guest_fault) {
            return SyscallResult::GuestFault;
        }

        user_regs regs = {0};

        ptrace(
            PTRACE_GETREGS,
            pid,
            0,
            &regs
        );

        // Before Linux executes a garbage syscall, replace it with a different one (getpid)
        ptrace(PTRACE_SET_SYSCALL, pid, 0, -1);

        CaptureNextSyscall();
        WaitUntilTrap();

        if (downcount_alarm_raised) {
            LOG_CRITICAL(Core_ARM11, "Bad downcount alarm timing (inbetween syscall boundary)");
        }

        if (guest_fault) {
            LOG_CRITICAL(Core_ARM11, "Guest fault occurred during getpid (?)");
        }

        // Now after the call to getpid

        // Recover the registers from before the syscall

        ptrace(
            PTRACE_SETREGS,
            pid,
            0,
            &regs
        );

        return SyscallResult::Syscall;
    }

    bool HasPendingCommands() {
        return tls_dirty || ts->atomic_command_pipe_count;
    }

    void SignalDowncountAlarm() {
        kill(pid, SIGSTOP);
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
            } else if (sig == SIGTRAP || sig == SIGCONT || sig == SIGALRM) {
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

            if (WIFEXITED(status)) {
                LOG_CRITICAL(Core_ARM11, "Process exited: {}", WEXITSTATUS(status));
                abort();
            }

            int sig = WSTOPSIG(status);

            if (sig == SIGTRAP) {
                break;
            } else if (sig == SIGCONT) {
                // ignore
                int res = ptrace(PTRACE_SYSCALL, pid, 0, 0);
                // LOG_ERROR(Core_ARM11, "SYSCALL result: {}", res);
            } else if (sig == SIGSTOP) {
                if (in_guest) {
                    // Downcount alarm hit, return
                    downcount_alarm_raised = true;
                    break;
                } else {
                    // Ignore
                    ptrace(PTRACE_SYSCALL, pid, 0, 0);
                }
            } else if (sig == SIGSEGV && in_guest) {
                siginfo_t siginfo;
                ptrace(PTRACE_GETSIGINFO, pid, 0, &siginfo);

                guest_fault = true;
                break;
            } else {
                LOG_CRITICAL(Core_ARM11, "Got wrong signal: {}", sig);

                DumpRegisters();

                DumpFaultAddress();
                LOG_CRITICAL(Core_ARM11, "RIP");

                LOG_CRITICAL(Core_ARM11, "PID: {}", pid);

                kill(pid, SIGSTOP);
                ptrace(PTRACE_DETACH, pid, 0, SIGSTOP);

                auto pidstr = std::to_string(pid);

                char *const argv[] = {
                    "gdb",
                    "-p",
                    &pidstr[0],
                    nullptr
                };

                execv("/usr/bin/gdb", argv);
            }
        }
    }

    void DumpRegisters() {
        std::array<u32, 16> reg;
        u32 cpsr;
        std::array<u32, 64> fp_regs;
        u32 fpscr, fpexc;
        GetContext(reg, cpsr, fp_regs, fpscr, fpexc, true);

        LOG_CRITICAL(Core_ARM11, "R0: {}", reg[0]);
        LOG_CRITICAL(Core_ARM11, "R1: {}", reg[1]);
        LOG_CRITICAL(Core_ARM11, "R2: {}", reg[2]);
        LOG_CRITICAL(Core_ARM11, "R3: {}", reg[3]);
        LOG_CRITICAL(Core_ARM11, "R4: {}", reg[4]);
        LOG_CRITICAL(Core_ARM11, "R12: {}", reg[12]);

        LOG_CRITICAL(Core_ARM11, "PC: {}", reg[15]);
        LOG_CRITICAL(Core_ARM11, "LR: {}", reg[14]);
        LOG_CRITICAL(Core_ARM11, "SP: {}", reg[13]);
    }

    void DumpFaultAddress() {
        siginfo_t siginfo;
        ptrace(PTRACE_GETSIGINFO, pid, 0, &siginfo);

        LOG_CRITICAL(Core_ARM11, "Fault address: {}", siginfo.si_addr);
    }

    void CaptureNextSyscall() {
        // TODO: Only send SIGCONT during a signal stop?
        ptrace(PTRACE_SYSCALL, pid, 0, SIGCONT);
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

    void DowncountThreadSignal() {
        std::unique_lock<std::mutex> downcount_lock(downcount_mutex);
        downcount_cond.notify_one();
    }

    void DowncountTimerThread() {
        while (true) {
            // Wait for guest entry
            std::unique_lock<std::mutex> downcount_lock(downcount_mutex);

            downcount_cond.wait(downcount_lock, [=]() {
                return in_guest && !downcount_alarm_raised;
            });

            // Guest code is now running, wait for timeout or guest exit
            downcount_cond.wait_for(downcount_lock, std::chrono::nanoseconds(next_downcount_ns), [=]() {
                return !in_guest;
            });

            if (in_guest) {
                // If the guest is still running after timeout, raise the alarm
                SignalDowncountAlarm();

                // Wait for guest exit.
                downcount_cond.wait(downcount_lock, [=]() {
                    return !in_guest;
                });
            }
        }
    }
};

void Armos::Init() {
    ram_fd_size =
        Memory::FCRAM_N3DS_SIZE +
        Memory::VRAM_SIZE +
        Memory::N3DS_EXTRA_RAM_SIZE +
        0x1000 /* CFG Memory */ +
        0x1000 /* Shared Page */ +
        Memory::DSP_RAM_SIZE;
    ram_fd = AllocateSharedMemory(ram_fd_size);
    ram_bump_offset = 0;
    ram_mapping = mmap(nullptr, ram_fd_size, PROT_READ | PROT_WRITE, MAP_SHARED, ram_fd, 0);
}

Armos::Region Armos::AllocateRegion(u32 size) {
    uintptr_t offset = ram_bump_offset;
    ram_bump_offset += size;
    return Region(reinterpret_cast<u8*>(ram_mapping) + offset, offset);
}

uintptr_t Armos::RegionPtrToShm(void* ptr) {
    uintptr_t offset = reinterpret_cast<uintptr_t>(ptr) - reinterpret_cast<uintptr_t>(ram_mapping);
    if (offset >= ram_fd_size) {
        return static_cast<uintptr_t>(-1);
    }
    return offset;
}

Armos::GuestContext* Armos::Guest::SpawnGuest() {
    auto guest = new GuestContext();
    guest->Init();
    return guest;
}

void Armos::Guest::DeleteGuest(GuestContext* guest) {
    delete guest;
}

void Armos::Guest::RunGuest(GuestContext* guest, GuestCallbacks* callbacks, std::array<u32, 16>& reg, u32& cpsr, std::array<u32, 64>& fp_regs, u32& fpscr, u32& fpexc) {
    if (guest) {
        guest->SetDowncount(callbacks->GetDowncountNanos());
        while (callbacks->ShouldContinue()) {
            if (guest->HasPendingCommands()) {
                guest->EnterTrampoline();
            }

            guest->SetContext(reg, cpsr, fp_regs, fpscr, fpexc);
            auto res = guest->EmulateSyscalls();
            guest->GetContext(reg, cpsr, fp_regs, fpscr, fpexc, res == GuestContext::SyscallResult::Syscall);

            switch (res) {
            case GuestContext::SyscallResult::Syscall:
                callbacks->ClearDowncount();
                callbacks->OnSwi();
                break;
            case GuestContext::SyscallResult::DowncountAlarm:
                callbacks->ClearDowncount();
                break;
            case GuestContext::SyscallResult::GuestFault:
                break;
            default:
                UNREACHABLE();
            }
        }
    }
}

void Armos::Guest::MapMemory(GuestContext* guest, void* memory, u32 virt_addr, u32 size) {
    if (guest) {
        guest->CommandMapMemory(RegionPtrToShm(memory), virt_addr, size);
    }
}

void Armos::Guest::UnmapMemory(GuestContext* guest, u32 virt_addr, u32 size) {
    if (guest) {
        guest->CommandUnmapMemory(virt_addr, size);
    }
}

void Armos::Guest::TrapMemory(GuestContext* guest, u32 virt_addr, u32 size) {
    if (guest) {
        guest->CommandTrapMemory(virt_addr, size);
    }
}

class ArmosThreadContext final : public ARM_Interface::ThreadContext {
public:
    ArmosThreadContext() {
        Reset();
    }
    ~ArmosThreadContext() override = default;

    void Reset() override {
        cpu_registers = {};
        cpsr = 0;
        fpu_registers = {};
        fpscr = 0;
        fpexc = 0;
    }

    u32 GetCpuRegister(std::size_t index) const override {
        return cpu_registers[index];
    }
    void SetCpuRegister(std::size_t index, u32 value) override {
        cpu_registers[index] = value;
    }
    u32 GetCpsr() const override {
        return cpsr;
    }
    void SetCpsr(u32 value) override {
        cpsr = value;
    }
    u32 GetFpuRegister(std::size_t index) const override {
        return fpu_registers[index];
    }
    void SetFpuRegister(std::size_t index, u32 value) override {
        fpu_registers[index] = value;
    }
    u32 GetFpscr() const override {
        return fpscr;
    }
    void SetFpscr(u32 value) override {
        fpscr = value;
    }
    u32 GetFpexc() const override {
        return fpexc;
    }
    void SetFpexc(u32 value) override {
        fpexc = value;
    }

    std::array<u32, 16> cpu_registers;
    u32 cpsr;
    std::array<u32, 64> fpu_registers;
    u32 fpscr;
    u32 fpexc;
};

class ARM_Armos final : public ARM_Interface, public Armos::GuestCallbacks {
public:
    explicit ARM_Armos(Core::System* system) : system(*system) {}
    ~ARM_Armos() = default;

    /* ARM_Interface */

    void Run() override {
        rescheduled = false;
        Armos::Guest::RunGuest(GetGuest(), this, context.cpu_registers, context.cpsr, context.fpu_registers, context.fpscr, context.fpexc);
    }

    void Step() override {
        LOG_ERROR(Core_ARM11, "Single stepping not implemented");
    }

    void ClearInstructionCache() override {}
    void InvalidateCacheRange(u32 start_address, std::size_t length) override {}
    void PageTableChanged() override {}

    void SetPC(u32 pc) override {
        context.cpu_registers[15] = pc;
    }
    u32 GetPC() const override {
        return context.cpu_registers[15];
    }
    u32 GetReg(int index) const override {
        return context.cpu_registers[index];
    }
    void SetReg(int index, u32 value) override {
        context.cpu_registers[index] = value;
    }
    u32 GetVFPReg(int index) const override {
        return context.fpu_registers[index];
    }
    void SetVFPReg(int index, u32 value) override {
        context.fpu_registers[index] = value;
    }
    u32 GetVFPSystemReg(VFPSystemRegister reg) const override {
        if (reg == VFP_FPSCR) {
            return context.fpscr;
        } else if (reg == VFP_FPEXC) {
            return context.fpexc;
        }
        return 0;
    }
    void SetVFPSystemReg(VFPSystemRegister reg, u32 value) override {
        if (reg == VFP_FPSCR) {
            context.fpscr = value;
        } else if (reg == VFP_FPEXC) {
            context.fpexc = value;
        }
    }
    u32 GetCPSR() const override {
        return context.cpsr;
    }
    void SetCPSR(u32 cpsr) override {
        context.cpsr = cpsr;
    }
    u32 GetCP15Register(CP15Register reg) override {
        return 0;
    }
    void SetCP15Register(CP15Register reg, u32 value) override {
        if (reg == CP15_THREAD_URO) {
            GetGuest()->SetTLS(value);
        }
    }

    std::unique_ptr<ThreadContext> NewContext() const override {
        return std::make_unique<ArmosThreadContext>();
    }

    void SaveContext(const std::unique_ptr<ThreadContext>& arg) override {
        ArmosThreadContext* ctx = dynamic_cast<ArmosThreadContext*>(arg.get());
        ASSERT(ctx);

        *ctx = context;
    }

    void LoadContext(const std::unique_ptr<ThreadContext>& arg) override {
        ArmosThreadContext* ctx = dynamic_cast<ArmosThreadContext*>(arg.get());
        ASSERT(ctx);

        context = *ctx;
    }

    void PrepareReschedule() override {
        rescheduled = true;
    }

    /* Armos::GuestCallbacks */

    void OnSwi() override {
        u32 instr = system.Memory().Read32(GetPC() - 4);
        Kernel::SVCContext{system}.CallSVC(instr & 0xFFFF);
    }

    bool ShouldContinue() override {
        return (!rescheduled) && system.CoreTiming().GetDowncount() > 0;
    }

    u32 GetDowncountNanos() override {
        return cyclesToNs(system.CoreTiming().GetDowncount());
    }

    void ClearDowncount() override {
        system.CoreTiming().AddTicks(system.CoreTiming().GetDowncount());
    }

private:
    Core::System& system;
    ArmosThreadContext context;
    bool rescheduled = false;

    Armos::GuestContext* GetGuest() {
        return system.Memory().GetCurrentPageTable()->armos_guest;
    }
};

std::unique_ptr<ARM_Interface> Armos::MakeCPUInterface(Core::System* system) {
    return std::make_unique<ARM_Armos>(system);
}
