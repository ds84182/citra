// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <cstring>
#include <memory>

#include "common/assert.h"
#include "common/logging/log.h"

#include "core/hw/gpu.h"
#include "core/memory.h"

#include "player.h"

namespace CiTrace {

std::unique_ptr<Player> g_player;
bool g_playback = false;

void Player::Init() {
    g_playback = false;
}

void Player::Shutdown() {
    g_player.reset();
    g_playback = false;
}

template <typename T>
static T GetOffset(const std::vector<u8> &data, const u32 offset) {
    ASSERT_MSG(offset < data.size(), "Offset out of bounds: %08X", offset);

    return reinterpret_cast<T>(data.data()+offset);
}

void Player::Run(u32 tight_loop) {
    // Reset hardware registers to initial states
    auto initial = &header->initial_state_offsets;

    // Reset GPU Registers
    ASSERT_MSG(initial->gpu_registers_size == sizeof(GPU::g_regs)/sizeof(u32), "GPU Register Size Mismatch!");
    std::memcpy(&GPU::g_regs, GetOffset<const GPU::Regs*>(trace_data, initial->gpu_registers), sizeof(GPU::g_regs));

    auto stream = GetOffset<const CTStreamElement*>(trace_data, header->stream_offset);
    auto stream_end = GetOffset<const CTStreamElement*>(trace_data, header->stream_offset+(header->stream_size-1)*sizeof(CTStreamElement));
    //TODO: Out of bounds assert for header->stream_offset+header->stream_size

    while (stream <= stream_end) {
        switch (stream->type) {
            case CTStreamElementType::FrameMarker:
                LOG_INFO(Core, "Frame Marker!");
                stream = stream_end; //exit outer loop
                break;
            case CTStreamElementType::MemoryLoad: {
                LOG_INFO(Core, "Load Memory (addr %04X, size %04X)", stream->memory_load.physical_address, stream->memory_load.size);
                u8 *dest = Memory::GetPhysicalPointer(stream->memory_load.physical_address);
                auto source = GetOffset<const u8*>(trace_data, stream->memory_load.file_offset);
                //TODO: Range check
                std::memcpy(dest, source, stream->memory_load.size);
                break;
            }
            case CTStreamElementType::RegisterWrite: {
                auto register_write = &stream->register_write;
                u32 paddr = register_write->physical_address;
                // paddr - IO PBase + IO VBase
                u32 vaddr = paddr - 0x10100000 + 0x1EC00000;
                LOG_INFO(Core, "Write Register (addr %04X size %02X val %016lX)", vaddr, register_write->size, register_write->value);

                switch (register_write->size) {
                    case CTRegisterWrite::SIZE_8:
                        GPU::Write(vaddr, (u8) register_write->value);
                        break;
                    case CTRegisterWrite::SIZE_16:
                        GPU::Write(vaddr, (u16) register_write->value);
                        break;
                    case CTRegisterWrite::SIZE_32:
                        GPU::Write(vaddr, (u32) register_write->value);
                        break;
                    case CTRegisterWrite::SIZE_64:
                        GPU::Write(vaddr, register_write->value);
                        break;
                    default:
                        ASSERT_MSG(false, "Unknown Register Write Size: %02X", register_write->size);
                }
                //GPU::Write(vaddr, );
                break;
            }
            default:
                ASSERT_MSG(false, "Unknown Stream Element Type: %02X", stream->type);
        }

        stream++;
    }
}

}
