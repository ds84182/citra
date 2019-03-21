#include "common/common_types.h"

namespace Armos {
    class GuestContext;
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
}
