// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "core/tracer/citrace.h"

namespace CiTrace {

class Player {
public:
    Player(std::vector<u8> data) :
        trace_data(std::move(data)) {};
private:
    std::vector<u8> trace_data;
};

extern std::unique_ptr<Player> g_player;
extern bool g_playback;

}
