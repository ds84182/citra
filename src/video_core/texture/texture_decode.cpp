// Copyright 2017 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/assert.h"
#include "common/color.h"
#include "common/logging/log.h"
#include "common/math_util.h"
#include "common/swap.h"
#include "common/vector_math.h"
#include "video_core/regs_texturing.h"
#include "video_core/texture/etc1.h"
#include "video_core/texture/texture_decode.h"
#include "video_core/utils.h"

using TextureFormat = Pica::TexturingRegs::TextureFormat;

namespace Pica {
namespace Texture {

constexpr size_t TILE_SIZE = 8 * 8;
constexpr size_t ETC1_SUBTILES = 2 * 2;

size_t CalculateTileSize(TextureFormat format) {
    switch (format) {
    case TextureFormat::RGBA8:
        return 4 * TILE_SIZE;

    case TextureFormat::RGB8:
        return 3 * TILE_SIZE;

    case TextureFormat::RGB5A1:
    case TextureFormat::RGB565:
    case TextureFormat::RGBA4:
    case TextureFormat::IA8:
    case TextureFormat::RG8:
        return 2 * TILE_SIZE;

    case TextureFormat::I8:
    case TextureFormat::A8:
    case TextureFormat::IA4:
        return 1 * TILE_SIZE;

    case TextureFormat::I4:
    case TextureFormat::A4:
        return TILE_SIZE / 2;

    case TextureFormat::ETC1:
        return ETC1_SUBTILES * 8;

    case TextureFormat::ETC1A4:
        return ETC1_SUBTILES * 16;

    default: // placeholder for yet unknown formats
        UNIMPLEMENTED();
        return 0;
    }
}

Math::Vec4<u8> LookupTexture(const u8* source, unsigned int x, unsigned int y,
                             const TextureInfo& info, bool disable_alpha) {
    // Coordinate in tiles
    const unsigned int coarse_x = x / 8;
    const unsigned int coarse_y = y / 8;

    // Coordinate inside the tile
    const unsigned int fine_x = x % 8;
    const unsigned int fine_y = y % 8;

    const u8* line = source + coarse_y * info.stride;
    const u8* tile = line + coarse_x * CalculateTileSize(info.format);
    return LookupTexelInTile(tile, fine_x, fine_y, info, disable_alpha);
}

Math::Vec4<u8> LookupTexelInTile(const u8* source, unsigned int x, unsigned int y,
                                 const TextureInfo& info, bool disable_alpha) {
    DEBUG_ASSERT(x < 8);
    DEBUG_ASSERT(y < 8);

    using VideoCore::MortonInterleave;

    switch (info.format) {
    case TextureFormat::RGBA8: {
        auto res = Color::DecodeRGBA8(source + MortonInterleave(x, y) * 4);
        return {res.r(), res.g(), res.b(), static_cast<u8>(disable_alpha ? 255 : res.a())};
    }

    case TextureFormat::RGB8: {
        auto res = Color::DecodeRGB8(source + MortonInterleave(x, y) * 3);
        return {res.r(), res.g(), res.b(), 255};
    }

    case TextureFormat::RGB5A1: {
        auto res = Color::DecodeRGB5A1(source + MortonInterleave(x, y) * 2);
        return {res.r(), res.g(), res.b(), static_cast<u8>(disable_alpha ? 255 : res.a())};
    }

    case TextureFormat::RGB565: {
        auto res = Color::DecodeRGB565(source + MortonInterleave(x, y) * 2);
        return {res.r(), res.g(), res.b(), 255};
    }

    case TextureFormat::RGBA4: {
        auto res = Color::DecodeRGBA4(source + MortonInterleave(x, y) * 2);
        return {res.r(), res.g(), res.b(), static_cast<u8>(disable_alpha ? 255 : res.a())};
    }

    case TextureFormat::IA8: {
        const u8* source_ptr = source + MortonInterleave(x, y) * 2;

        if (disable_alpha) {
            // Show intensity as red, alpha as green
            return {source_ptr[1], source_ptr[0], 0, 255};
        } else {
            return {source_ptr[1], source_ptr[1], source_ptr[1], source_ptr[0]};
        }
    }

    case TextureFormat::RG8: {
        auto res = Color::DecodeRG8(source + MortonInterleave(x, y) * 2);
        return {res.r(), res.g(), 0, 255};
    }

    case TextureFormat::I8: {
        const u8* source_ptr = source + MortonInterleave(x, y);
        return {*source_ptr, *source_ptr, *source_ptr, 255};
    }

    case TextureFormat::A8: {
        const u8* source_ptr = source + MortonInterleave(x, y);

        if (disable_alpha) {
            return {*source_ptr, *source_ptr, *source_ptr, 255};
        } else {
            return {0, 0, 0, *source_ptr};
        }
    }

    case TextureFormat::IA4: {
        const u8* source_ptr = source + MortonInterleave(x, y);

        u8 i = Color::Convert4To8(((*source_ptr) & 0xF0) >> 4);
        u8 a = Color::Convert4To8((*source_ptr) & 0xF);

        if (disable_alpha) {
            // Show intensity as red, alpha as green
            return {i, a, 0, 255};
        } else {
            return {i, i, i, a};
        }
    }

    case TextureFormat::I4: {
        u32 morton_offset = MortonInterleave(x, y);
        const u8* source_ptr = source + morton_offset / 2;

        u8 i = (morton_offset % 2) ? ((*source_ptr & 0xF0) >> 4) : (*source_ptr & 0xF);
        i = Color::Convert4To8(i);

        return {i, i, i, 255};
    }

    case TextureFormat::A4: {
        u32 morton_offset = MortonInterleave(x, y);
        const u8* source_ptr = source + morton_offset / 2;

        u8 a = (morton_offset % 2) ? ((*source_ptr & 0xF0) >> 4) : (*source_ptr & 0xF);
        a = Color::Convert4To8(a);

        if (disable_alpha) {
            return {a, a, a, 255};
        } else {
            return {0, 0, 0, a};
        }
    }

    case TextureFormat::ETC1:
    case TextureFormat::ETC1A4: {
        bool has_alpha = (info.format == TextureFormat::ETC1A4);
        size_t subtile_size = has_alpha ? 16 : 8;

        // ETC1 further subdivides each 8x8 tile into four 4x4 subtiles
        constexpr unsigned int subtile_width = 4;
        constexpr unsigned int subtile_height = 4;

        unsigned int subtile_index = (x / subtile_width) + 2 * (y / subtile_height);
        x %= subtile_width;
        y %= subtile_height;

        const u8* subtile_ptr = source + subtile_index * subtile_size;

        u8 alpha = 255;
        if (has_alpha) {
            u64_le packed_alpha;
            memcpy(&packed_alpha, subtile_ptr, sizeof(u64));
            subtile_ptr += sizeof(u64);

            alpha = Color::Convert4To8((packed_alpha >> (4 * (x * subtile_width + y))) & 0xF);
        }

        u64_le subtile_data;
        memcpy(&subtile_data, subtile_ptr, sizeof(u64));

        return Math::MakeVec(SampleETC1Subtile(subtile_data, x, y),
                             disable_alpha ? (u8)255 : alpha);
    }

    default:
        LOG_ERROR(HW_GPU, "Unknown texture format: %x", (u32)info.format);
        DEBUG_ASSERT(false);
        return {};
    }
}

template <typename TileHandler>
void DecodeTiles(const u8* source, const TextureInfo& info, const TileHandler& tileHandler) {
    const u8* tile = source;

    const unsigned int tile_width = info.width / 8;
    const unsigned int tile_height = info.height / 8;

    for (unsigned int tile_y = 0; tile_y < tile_height; tile_y++) {
        for (unsigned int tile_x = 0; tile_x < tile_width; tile_x++) {
            tileHandler(tile_x, tile_y, tile);
            tile += CalculateTileSize(info.format);
        }
    }
}

template <typename ToRGBA8>
void DecodeToRGBA8(const u8* source, Math::Vec4<u8>* dest, const TextureInfo& info, const ToRGBA8& toRGBA8) {
    DecodeTiles(source, info, [&](unsigned int tile_x, unsigned int tile_y, const u8* tile) {
        for (unsigned int i = 0; i<8 * 8; i++) {
            auto interleveOffset = VideoCore::MortonLUT[i];

            auto x = (i % 8) + (tile_x * 8);
            auto y = info.height - 1 - ((i / 8) + (tile_y * 8));

            dest[y * info.width + x] = toRGBA8(tile, interleveOffset);
        }
    });
}

template <bool HasAlpha, typename OutputPixel>
void DecodeETCTiles(unsigned int tile_x, unsigned int tile_y, const u8* tile, const OutputPixel& outputPixel) {
    constexpr size_t subtile_size = HasAlpha ? 16 : 8;

    // ETC1 further subdivides each 8x8 tile into four 4x4 subtiles
    constexpr unsigned int subtile_width = 4;
    constexpr unsigned int subtile_height = 4;

    for (unsigned int subtile_index = 0; subtile_index < 4; subtile_index++) {
        unsigned int subtile_x_offset = ((subtile_index % 2) * subtile_width) + (tile_x * 8);
        unsigned int subtile_y_offset = ((subtile_index / 2) * subtile_height) + (tile_y * 8);

        const u8* subtile_ptr = tile + subtile_index * subtile_size;

        u64_le packed_alpha = 0xFFFFFFFFFFFFFFFFULL;
        if (HasAlpha) {
            memcpy(&packed_alpha, subtile_ptr, sizeof(u64));
            subtile_ptr += sizeof(u64);
        }

        u64_le subtile_data;
        memcpy(&subtile_data, subtile_ptr, sizeof(u64));

        ETC1Tile tile{subtile_data};

        for (unsigned y = 0; y < 4; y++) {
            for (unsigned x = 0; x < 4; x++) {
                outputPixel(subtile_x_offset + x, subtile_y_offset + y,
                    Math::MakeVec(tile.GetRGB(x, y),
                        static_cast<u8>(HasAlpha ? Color::Convert4To8((packed_alpha >> (4 * (x * subtile_width + y))) & 0xF) : 255)));
            }
        }
    }
}

void DecodeETCToRGBA8(const u8* source, Math::Vec4<u8>* dest, const TextureInfo& info) {
    DecodeTiles(source, info, [&](unsigned int tile_x, unsigned int tile_y, const u8* tile) {
        const auto outputPixel = [&](unsigned int x, unsigned int y, const Math::Vec4<u8> &color) {
            y = info.height - 1 - y;
            dest[y * info.width + x] = color;
        };
        if (info.format == TextureFormat::ETC1) {
            DecodeETCTiles<false, decltype(outputPixel)>(tile_x, tile_y, tile, outputPixel);
        } else {
            ASSERT(info.format == TextureFormat::ETC1A4);
            DecodeETCTiles<true, decltype(outputPixel)>(tile_x, tile_y, tile, outputPixel);
        }
    });
}

void DecodeRGBA8(const u8* source, Math::Vec4<u8>* dest, const TextureInfo& info) {
    switch (info.format) {
    case TextureFormat::RGBA8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) {
            return Color::DecodeRGBA8(tile + offset * 4);
        });
        break;
    case TextureFormat::RGB8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) {
            return Color::DecodeRGB8(tile + offset * 3);
        });
        break;
    case TextureFormat::RGB5A1:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) {
            return Color::DecodeRGB5A1(tile + offset * 2);
        });
        break;
    case TextureFormat::RGB565:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) {
            return Color::DecodeRGB565(tile + offset * 2);
        });
        break;
    case TextureFormat::RGBA4:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) {
            return Color::DecodeRGBA4(tile + offset * 2);
        });
        break;
    case TextureFormat::IA8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) -> Math::Vec4<u8> {
            auto source_ptr = tile + offset * 2;
            return {source_ptr[1], source_ptr[1], source_ptr[1], source_ptr[0]};
        });
        break;
    case TextureFormat::RG8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) -> Math::Vec4<u8> {
            auto res = Color::DecodeRG8(tile + offset * 2);
            return {res.r(), res.g(), 0, 255};
        });
        break;
    case TextureFormat::I8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) -> Math::Vec4<u8> {
            auto source_ptr = tile + offset;
            return {*source_ptr, *source_ptr, *source_ptr, 255};
        });
        break;
    case TextureFormat::A8:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) -> Math::Vec4<u8> {
            auto source_ptr = tile + offset;
            return {0, 0, 0, *source_ptr};
        });
        break;
    case TextureFormat::IA4:
        DecodeToRGBA8(source, dest, info, [](const u8* tile, u32 offset) -> Math::Vec4<u8> {
            auto source_ptr = tile + offset;
            const u8 i = Color::Convert4To8(((*source_ptr) & 0xF0) >> 4);
            const u8 a = Color::Convert4To8((*source_ptr) & 0xF);
            return {i, i, i, a};
        });
        break;
    case TextureFormat::ETC1:
    case TextureFormat::ETC1A4:
        DecodeETCToRGBA8(source, dest, info);
        break;
    default:
        LOG_ERROR(HW_GPU, "Unaccelerated texture copy: %x %dx%d", (u32)info.format, info.width, info.height);
        for (unsigned y = 0; y < info.height; ++y) {
            for (unsigned x = 0; x < info.width; ++x) {
                dest[x + info.width * y] = Pica::Texture::LookupTexture(source, x, info.height - 1 - y, info);
            }
        }
    }
}

TextureInfo TextureInfo::FromPicaRegister(const TexturingRegs::TextureConfig& config,
                                          const TexturingRegs::TextureFormat& format) {
    TextureInfo info;
    info.physical_address = config.GetPhysicalAddress();
    info.width = config.width;
    info.height = config.height;
    info.format = format;
    info.SetDefaultStride();
    return info;
}

} // namespace Texture
} // namespace Pica
