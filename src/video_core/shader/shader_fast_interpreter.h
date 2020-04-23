#include <vector>
#include "common/common_types.h"
#include "common/logging/log.h"
#include "common/microprofile.h"
#include "common/vector_math.h"
#include "video_core/pica_state.h"
#include "video_core/pica_types.h"
#include "video_core/shader/shader.h"

namespace Pica::Shader::Fast {

// TODO: Use intptr_t when everything is converted to Word
using Word = uint32_t;

class CachedBatch {
public:
    CachedBatch(u64 key) : key(key) {}

    bool Compile(const ShaderSetup& setup, UnitState& state);

    void Run(const ShaderSetup& setup, UnitState& state, unsigned int entry_point) const;

    bool IsCompiled() const {
        return compiled;
    }

    u64 Key() const {
        return key;
    }
private:
    u64 key;
    std::vector<Word> code;
    bool compiled = false;
};

} // namespace Pica::Shader::Fast
