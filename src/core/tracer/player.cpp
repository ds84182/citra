// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <cstring>
#include <memory>

#include "common/assert.h"
#include "common/logging/log.h"

#include "core/hw/gpu.h"
#include "core/hw/lcd.h"
#include "core/memory.h"

#include "video_core/pica.h"

#include "player.h"

namespace CiTrace {

std::unique_ptr<Player> g_player;
bool g_playback = false;
static int playback_position = 0;

void Player::Init() {
    g_playback = false;
    playback_position = 0;
}

void Player::Shutdown() {
    g_player.reset();
    g_playback = false;
    playback_position = 0;
}

template <typename T>
static T GetOffset(const std::vector<u8> &data, const u32 offset) {
    ASSERT_MSG(offset < data.size(), "Offset out of bounds: %08X", offset);

    return reinterpret_cast<T>(data.data()+offset);
}

void Player::Run(u32 tight_loop) {
    if (playback_position >= header->stream_size)
        playback_position = 0;

    if (playback_position == 0) {
        // Reset hardware registers to initial states
        auto initial = &header->initial_state_offsets;

        // Reset GPU Registers
        ASSERT_MSG(initial->gpu_registers_size == sizeof(GPU::g_regs)/sizeof(u32), "GPU Register Size Mismatch!");
        std::memcpy(&GPU::g_regs, GetOffset<const GPU::Regs*>(trace_data, initial->gpu_registers), sizeof(GPU::g_regs));

        // Reset LCD Registers
        ASSERT_MSG(initial->lcd_registers_size == sizeof(LCD::g_regs)/sizeof(u32), "LCD Register Size Mismatch!");
        std::memcpy(&LCD::g_regs, GetOffset<const LCD::Regs*>(trace_data, initial->lcd_registers), sizeof(LCD::g_regs));

        // Reset Pica Registers
        ASSERT_MSG(initial->pica_registers_size == sizeof(Pica::g_state.regs)/sizeof(u32), "Pica Register Size Mismatch!");
        std::memcpy(&Pica::g_state.regs, GetOffset<const Pica::Regs*>(trace_data, initial->pica_registers), sizeof(Pica::g_state.regs));

        // Reset Default Attributes
        ASSERT_MSG(initial->default_attributes_size == 4 * 16, "Default Attributes Size Mismatch!");
        auto default_attributes = GetOffset<const u32*>(trace_data, initial->default_attributes);
        for (unsigned i = 0; i < 16; i++) {
            for (unsigned comp = 0; comp < 3; comp++) {
                Pica::g_state.vs.default_attributes[i][comp] = Pica::float24::FromRawFloat24(default_attributes[4 * i + comp]);
            }
        }

        // Reset Float Uniforms
        ASSERT_MSG(initial->vs_float_uniforms_size == 4 * 96, "VS Float Uniforms Size Mismatch!");
        auto float_uniforms = GetOffset<const u32*>(trace_data, initial->vs_float_uniforms);
        for (unsigned i = 0; i < 96; i++) {
            for (unsigned comp = 0; comp < 3; comp++) {
                Pica::g_state.vs.uniforms.f[i][comp] = Pica::float24::FromRawFloat24(float_uniforms[4 * i + comp]);
            }
        }

        // Reset Vertex Shader Program Binary
        ASSERT_MSG(initial->vs_program_binary_size == Pica::g_state.vs.program_code.size(), "VS Program Code Size Mismatch!");
        std::memcpy(&Pica::g_state.vs.program_code[0], GetOffset<const Pica::Regs*>(trace_data, initial->vs_program_binary), sizeof(u32)*Pica::g_state.vs.program_code.size());

        // Reset Vertex Shader Swizzle Data
        ASSERT_MSG(initial->vs_swizzle_data_size == Pica::g_state.vs.program_code.size(), "VS Swizzle Data Size Mismatch!");
        std::memcpy(&Pica::g_state.vs.swizzle_data[0], GetOffset<const Pica::Regs*>(trace_data, initial->vs_swizzle_data), Pica::g_state.vs.program_code.size());
    }

    auto stream = GetOffset<const CTStreamElement*>(trace_data, header->stream_offset);
    stream += playback_position;
    auto stream_end = GetOffset<const CTStreamElement*>(trace_data, header->stream_offset+(header->stream_size-1)*sizeof(CTStreamElement));
    //TODO: Out of bounds assert for header->stream_offset+header->stream_size
    bool frame_done = false;

    while ((!frame_done) && stream <= stream_end) {
        switch (stream->type) {
            case CTStreamElementType::FrameMarker:
                LOG_INFO(Core, "Frame Marker!");
                //stream = stream_end; //exit outer loop
                frame_done = true;
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
                break;
            }
            default:
                ASSERT_MSG(false, "Unknown Stream Element Type: %02X", stream->type);
        }

        stream++;
        playback_position++;
    }
}

}
