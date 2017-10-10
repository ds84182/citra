// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "common/common_types.h"

namespace VideoCore {

/**
 * Interleave the lower 3 bits of each coordinate to get the intra-block offsets, which are
 * arranged in a Z-order curve. More details on the bit manipulation at:
 * https://fgiesen.wordpress.com/2009/12/13/decoding-morton-codes/
 */
static constexpr inline u32 GenerateMortonInterleave(u32 x, u32 y) {
    u32 i = (x & 7) | ((y & 7) << 8); // ---- -210
    i = (i ^ (i << 2)) & 0x1313;      // ---2 --10
    i = (i ^ (i << 1)) & 0x1515;      // ---2 -1-0
    i = (i | (i >> 7)) & 0x3F;
    return i;
}

constexpr u32 MortonLUT[8 * 8] = {
    GenerateMortonInterleave(0, 0),
    GenerateMortonInterleave(1, 0),
    GenerateMortonInterleave(2, 0),
    GenerateMortonInterleave(3, 0),
    GenerateMortonInterleave(4, 0),
    GenerateMortonInterleave(5, 0),
    GenerateMortonInterleave(6, 0),
    GenerateMortonInterleave(7, 0),

    GenerateMortonInterleave(0, 1),
    GenerateMortonInterleave(1, 1),
    GenerateMortonInterleave(2, 1),
    GenerateMortonInterleave(3, 1),
    GenerateMortonInterleave(4, 1),
    GenerateMortonInterleave(5, 1),
    GenerateMortonInterleave(6, 1),
    GenerateMortonInterleave(7, 1),

    GenerateMortonInterleave(0, 2),
    GenerateMortonInterleave(1, 2),
    GenerateMortonInterleave(2, 2),
    GenerateMortonInterleave(3, 2),
    GenerateMortonInterleave(4, 2),
    GenerateMortonInterleave(5, 2),
    GenerateMortonInterleave(6, 2),
    GenerateMortonInterleave(7, 2),

    GenerateMortonInterleave(0, 3),
    GenerateMortonInterleave(1, 3),
    GenerateMortonInterleave(2, 3),
    GenerateMortonInterleave(3, 3),
    GenerateMortonInterleave(4, 3),
    GenerateMortonInterleave(5, 3),
    GenerateMortonInterleave(6, 3),
    GenerateMortonInterleave(7, 3),

    GenerateMortonInterleave(0, 4),
    GenerateMortonInterleave(1, 4),
    GenerateMortonInterleave(2, 4),
    GenerateMortonInterleave(3, 4),
    GenerateMortonInterleave(4, 4),
    GenerateMortonInterleave(5, 4),
    GenerateMortonInterleave(6, 4),
    GenerateMortonInterleave(7, 4),

    GenerateMortonInterleave(0, 5),
    GenerateMortonInterleave(1, 5),
    GenerateMortonInterleave(2, 5),
    GenerateMortonInterleave(3, 5),
    GenerateMortonInterleave(4, 5),
    GenerateMortonInterleave(5, 5),
    GenerateMortonInterleave(6, 5),
    GenerateMortonInterleave(7, 5),

    GenerateMortonInterleave(0, 6),
    GenerateMortonInterleave(1, 6),
    GenerateMortonInterleave(2, 6),
    GenerateMortonInterleave(3, 6),
    GenerateMortonInterleave(4, 6),
    GenerateMortonInterleave(5, 6),
    GenerateMortonInterleave(6, 6),
    GenerateMortonInterleave(7, 6),

    GenerateMortonInterleave(0, 7),
    GenerateMortonInterleave(1, 7),
    GenerateMortonInterleave(2, 7),
    GenerateMortonInterleave(3, 7),
    GenerateMortonInterleave(4, 7),
    GenerateMortonInterleave(5, 7),
    GenerateMortonInterleave(6, 7),
    GenerateMortonInterleave(7, 7),
};

static constexpr inline u32 MortonInterleave(u32 x, u32 y) {
    return MortonLUT[(x & 7) + ((y & 7) * 8)];
}

/**
 * Calculates the offset of the position of the pixel in Morton order
 */
static inline u32 GetMortonOffset(u32 x, u32 y, u32 bytes_per_pixel) {
    // Images are split into 8x8 tiles. Each tile is composed of four 4x4 subtiles each
    // of which is composed of four 2x2 subtiles each of which is composed of four texels.
    // Each structure is embedded into the next-bigger one in a diagonal pattern, e.g.
    // texels are laid out in a 2x2 subtile like this:
    // 2 3
    // 0 1
    //
    // The full 8x8 tile has the texels arranged like this:
    //
    // 42 43 46 47 58 59 62 63
    // 40 41 44 45 56 57 60 61
    // 34 35 38 39 50 51 54 55
    // 32 33 36 37 48 49 52 53
    // 10 11 14 15 26 27 30 31
    // 08 09 12 13 24 25 28 29
    // 02 03 06 07 18 19 22 23
    // 00 01 04 05 16 17 20 21
    //
    // This pattern is what's called Z-order curve, or Morton order.

    const unsigned int block_height = 8;
    const unsigned int coarse_x = x & ~7;

    u32 i = VideoCore::MortonInterleave(x, y);

    const unsigned int offset = coarse_x * block_height;

    return (i + offset) * bytes_per_pixel;
}

} // namespace
