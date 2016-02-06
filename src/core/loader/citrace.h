// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>

#include "common/common_types.h"
#include "core/loader/loader.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Loader namespace

namespace Loader {

/// Loads a CiTrace recording
class AppLoader_CITRACE final : public AppLoader {
public:
    AppLoader_CITRACE(FileUtil::IOFile&& file, std::string filename)
        : AppLoader(std::move(file)), filename(std::move(filename)) {}

    /**
     * Returns the type of the file
     * @param file FileUtil::IOFile open file
     * @return FileType found, or FileType::Error if this loader doesn't know it
     */
    static FileType IdentifyType(FileUtil::IOFile& file);

    /**
     * Load the bootable file
     * @return ResultStatus result of function
     */
    ResultStatus Load() override;

private:
    std::string filename;
};

} // namespace Loader
