#include <memory>

#include "common/common_types.h"

class ARM_Interface;

namespace Core {
class System;
}

namespace Armos {
    class GuestContext;

    class GuestCallbacks {
    public:
        virtual void OnSwi() = 0;
        virtual bool ShouldContinue() = 0;
    };

    void Init();

    class Region {
    private:
        u8* data;
        uintptr_t shm_offset;

        Region(u8* data, uintptr_t shm_offset) : data(data), shm_offset(shm_offset) {}

        friend Armos::Region AllocateRegion(u32);
    public:
        u8 *get() {
            return data;
        }
    };

    Region AllocateRegion(u32 size);
    uintptr_t RegionPtrToShm(void* ptr);

    namespace Guest {
        GuestContext* SpawnGuest();
        void DeleteGuest(GuestContext* guest);
        void RunGuest(GuestContext* guest, GuestCallbacks* callbacks, std::array<u32, 16>& reg, u32& cpsr, std::array<u32, 64>& fp_regs, u32& fpscr, u32& fpexc);
        void MapMemory(GuestContext* guest, void* memory, u32 virt_addr, u32 size);
        void UnmapMemory(GuestContext* guest, u32 virt_addr, u32 size);
    }

    std::unique_ptr<ARM_Interface> MakeCPUInterface(Core::System* system);
}
