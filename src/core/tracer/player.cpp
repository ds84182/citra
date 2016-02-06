// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <memory>

#include "player.h"

namespace CiTrace {

std::unique_ptr<Player> g_player;
bool g_playback = false;

}
