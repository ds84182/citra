// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <memory>
#include <vector>

#include "common/logging/log.h"
#include "common/make_unique.h"

#include "core/tracer/citrace.h"
#include "core/tracer/player.h"

#include "citrace.h"

namespace Loader {

static bool LoadFile(FileUtil::IOFile& file) {
    if (!file.IsOpen())
        return false;

    // Reset read pointer in case this file has been read before.
    file.Seek(0, SEEK_SET);

    u64 size = file.GetSize();

    // Read the entire file into a vector
    std::vector<u8> trace_data(size);

    if (file.ReadBytes(trace_data.data(), size) != size)
        return false;

    CiTrace::g_player = Common::make_unique<CiTrace::Player>(trace_data);
    CiTrace::g_playback = true;

    return true;
}

FileType AppLoader_CITRACE::IdentifyType(FileUtil::IOFile& file) {
    u32 magic;
    file.Seek(0, SEEK_SET);
    if (1 != file.ReadArray<u32>(&magic, 1))
        return FileType::Error;

    if (MakeMagic('C', 'i', 'T', 'r') == magic)
        return FileType::CITRACE;

    return FileType::Error;
}

ResultStatus AppLoader_CITRACE::Load() {
    if (is_loaded)
        return ResultStatus::ErrorAlreadyLoaded;

    if (!file.IsOpen())
        return ResultStatus::Error;

    if (!LoadFile(file))
        return ResultStatus::Error;

    is_loaded = true;
    return ResultStatus::Success;
}

} // namespace Loader
