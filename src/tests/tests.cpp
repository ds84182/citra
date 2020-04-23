// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "common/logging/backend.h"

// Catch provides the main function since we've given it the
// CATCH_CONFIG_MAIN preprocessor directive.

struct CitraListener : Catch::TestEventListenerBase {
    using TestEventListenerBase::TestEventListenerBase;

    void testRunStarting(Catch::TestRunInfo const& testInfo) override {
        Log::AddBackend(std::make_unique<Log::ColorConsoleBackend>());
    }
};
CATCH_REGISTER_LISTENER(CitraListener);
