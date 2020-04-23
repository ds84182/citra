// Optimize this file for faster math operations
#pragma GCC optimize("fast-math")

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#define CITRA_NEON 1
#endif

#ifdef CITRA_NEON
#include <arm_neon.h>
#endif

#include <array>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <nihstro/shader_bytecode.h>

#include <video_core/shader/shader_fast_interpreter.h>

namespace Pica::Shader::Fast {

using nihstro::Instruction;
using nihstro::OpCode;
using nihstro::RegisterType;
using nihstro::SourceRegister;
using nihstro::SwizzlePattern;
using Selector = nihstro::SwizzlePattern::Selector;

using Pica::Shader::UnitState;
using Pica::Shader::ShaderSetup;

#define _ cc.

#ifdef CITRA_NEON

template <typename T>
struct NeonType4Of;

template <>
struct NeonType4Of<float> {
    using type = float32x4_t;
};

template <>
struct NeonType4Of<unsigned int> {
    using type = uint32x4_t;
};

template <typename T>
using NeonType4 = typename NeonType4Of<T>::type;

#endif

template <typename T>
union Reg4 {
    struct {
        T x, y, z, w;
    };

    std::array<T, 4> v;

#ifdef CITRA_NEON
    // At minimum, including the Neon type in the union improves code generation
    NeonType4<T> neon;
#endif

    Reg4<T> operator+(const Reg4<T> &b) const {
        auto ax = this->x;
        auto ay = this->y;
        auto az = this->z;
        auto aw = this->w;
        auto bx = b.x;
        auto by = b.y;
        auto bz = b.z;
        auto bw = b.w;
        return {
            ax + bx,
            ay + by,
            az + bz,
            aw + bw,
        };
    }

#ifdef CITRA_NEON
    // GCC 7.3.1 Autovectorization has issues with multiply.
    // It instead decomposes the multiply into 4 multiply instructions.
    Reg4<T> operator*(const Reg4<T> &b) const {
        Reg4<T> res;
        res.neon = vmulq_f32(this->neon, b.neon);
        return res;
    }
#else
    Reg4<T> operator*(const Reg4<T> &b) const {
        auto ax = this->x;
        auto ay = this->y;
        auto az = this->z;
        auto aw = this->w;
        auto bx = b.x;
        auto by = b.y;
        auto bz = b.z;
        auto bw = b.w;
        return {
            ax * bx,
            ay * by,
            az * bz,
            aw * bw,
        };
    }
#endif

    Reg4<T> operator-() const {
        return {
            -x,
            -y,
            -z,
            -w,
        };
    }

#ifdef CITRA_NEON
    // ARM Neon assisted select
    Reg4<T> Select(const Reg4<T> &b, const Reg4<unsigned int> &mask) const {
        Reg4<T> res;
        res.neon = vbslq_f32(mask.neon, this->neon, b.neon);
        return res;
    }
#else
    Reg4<T> Select(const Reg4<T> &b, const Reg4<unsigned int> &mask) const {
        Reg4<unsigned int> r = {
            (mask.x & *reinterpret_cast<const int*>(&this->x)) | ((~mask.x) & *reinterpret_cast<const int*>(&b.x)),
            (mask.y & *reinterpret_cast<const int*>(&this->y)) | ((~mask.y) & *reinterpret_cast<const int*>(&b.y)),
            (mask.z & *reinterpret_cast<const int*>(&this->z)) | ((~mask.z) & *reinterpret_cast<const int*>(&b.z)),
            (mask.w & *reinterpret_cast<const int*>(&this->w)) | ((~mask.w) & *reinterpret_cast<const int*>(&b.w)),
        };
        return *reinterpret_cast<Reg4<T>*>(&r);
    }
#endif

#ifdef CITRA_NEON
    Reg4<T> NeonSwizzle(uint8x16_t xyzw) const {
        Reg4<T> res;
        res.neon = (float32x4_t)vcombine_u8(
            vtbl2_u8((const uint8x8x2_t&)neon, vget_low_u8(xyzw)),
            vtbl2_u8((const uint8x8x2_t&)neon, vget_high_u8(xyzw))
        );
        return res;
    }
#endif

    static T Dot3(const Reg4<T> &a, const Reg4<T> &b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static T Dot4(const Reg4<T> &a, const Reg4<T> &b) {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    static constexpr Reg4<T> Splat(T v) {
        return { v, v, v, v };
    }
#ifdef CITRA_NEON
// Use Neon alignment, which is more optimal.
};
#else
// Use general data alignment for autovectorization.
} __attribute((aligned(16)));
#endif

template union Reg4<float>;

static constexpr Reg4<unsigned int> MakeSelectMask(unsigned int mask) {
    return {
        (mask & 8) ? 0xFFFFFFFF : 0x00000000,
        (mask & 4) ? 0xFFFFFFFF : 0x00000000,
        (mask & 2) ? 0xFFFFFFFF : 0x00000000,
        (mask & 1) ? 0xFFFFFFFF : 0x00000000,
    };
}

static constexpr Reg4<unsigned int> SelectMasks[] = {
    MakeSelectMask(0),
    MakeSelectMask(1),
    MakeSelectMask(2),
    MakeSelectMask(3),
    MakeSelectMask(4),
    MakeSelectMask(5),
    MakeSelectMask(6),
    MakeSelectMask(7),
    MakeSelectMask(8),
    MakeSelectMask(9),
    MakeSelectMask(10),
    MakeSelectMask(11),
    MakeSelectMask(12),
    MakeSelectMask(13),
    MakeSelectMask(14),
    MakeSelectMask(15),
};

#ifdef CITRA_NEON

#define X 0
#define Y 4
#define Z 8
#define W 12

#define COMP(A) A, A + 1, A + 2, A + 3
#define LEVEL4(A, B, C, D) { COMP(A), COMP(B), COMP(C), COMP(D) }
#define LEVEL3(A, B, C) LEVEL4(A, B, C, X), LEVEL4(A, B, C, Y), LEVEL4(A, B, C, Z), LEVEL4(A, B, C, W)
#define LEVEL2(A, B) LEVEL3(A, B, X), LEVEL3(A, B, Y), LEVEL3(A, B, Z), LEVEL3(A, B, W)
#define LEVEL1(A) LEVEL2(A, X), LEVEL2(A, Y), LEVEL2(A, Z), LEVEL2(A, W)

static constexpr uint8x16_t NeonSwizzles[] = {
    LEVEL1(X),
    LEVEL1(Y),
    LEVEL1(Z),
    LEVEL1(W),
};

#undef LEVEL1
#undef LEVEL2
#undef LEVEL3
#undef LEVEL4
#undef COMP
#undef W
#undef Z
#undef Y
#undef X

#endif

template <typename T>
struct ScatterReg4 {
    T x;
    T y;
    T z;
    T w;

    Reg4<std::remove_const_t<std::remove_reference_t<decltype(*x)>>> operator*() const {
        return {
            *x,
            *y,
            *z,
            *w,
        };
    }
};

using SwizzleReg4f = ScatterReg4<const float * __restrict>;

#ifdef CITRA_NEON
struct NeonSwizzleReg4f {
    const Reg4<float> * __restrict reg;
    const uint8x16_t * __restrict swizzle;

    Reg4<float> operator*() const {
        return reg->NeonSwizzle(*swizzle);
    }
};
#endif

struct OperandBuffer;

using InstructionFn = void(*)(OperandBuffer&);

template <typename T, size_t N>
struct FixedStack {
    std::array<T, N> entries = {};
    size_t index = 0;

    T &Push() {
        auto current = index;
        index = (index + 1) % N;
        return entries[current];
    }

    T &Pop() {
        index = (index - 1) % N;
        return entries[index];
    }

    T &Peek() {
        return entries[(index - 1) % N];
    }
};

struct OperandBuffer {
    using StackEntry = std::pair<const uint32_t*, const uint32_t*>;

    const uint32_t * __restrict buffer;

    FixedStack<StackEntry, 4> return_stack;
    FixedStack<StackEntry, 8> if_stack;

    OperandBuffer(const uint32_t *buffer) : buffer(buffer) {}

    uint32_t Offset() {
        return *Pointer<uint32_t>();
    }

    uint32_t Value() {
        return *(buffer++);
    }

    template <typename T>
    T *Pointer() {
        return *reinterpret_cast<T* const*>(buffer++);
    }

    uint32_t Swizzle() {
        return *(buffer++);
    }

    Reg4<float> *Reg(uint32_t offset = 0) {
        // TODO: Support 64-bit
        return (*reinterpret_cast<Reg4<float>* const*>(buffer++)) + offset;
    }

    const Reg4<unsigned int> *__restrict Mask() {
        return (*reinterpret_cast<const Reg4<unsigned int> *const*>(buffer++));
    }

#ifdef CITRA_NEON
    const uint8x16_t * __restrict NeonSwizzle() {
        return (*reinterpret_cast<const uint8x16_t *const*>(buffer++));
    }
#endif

    Reg4<float> Input() {
        return *Reg();
    }

#ifdef CITRA_NEON
    using SwizzleReg = NeonSwizzleReg4f;
    NeonSwizzleReg4f InputSwizzle(uint32_t offset) {
        return {
            Reg(offset),
            NeonSwizzle()
        };
    }
#else
    using SwizzleReg = SwizzleReg4f;
    SwizzleReg4f InputSwizzle(uint32_t offset) {
        offset *= 4;
        return {
            Pointer<float>() + offset,
            Pointer<float>() + offset,
            Pointer<float>() + offset,
            Pointer<float>() + offset,
        };
    }
#endif

    void Continue() {
        reinterpret_cast<InstructionFn>(*(buffer++))(*this);
    }

    const uint32_t *JumpTarget() {
        return Pointer<const uint32_t>();
    }

    void Jump(const uint32_t *target) {
        buffer = target;
        Continue();
    }

    void PushReturnAddress(const uint32_t *source) {
        const uint32_t *dest = buffer;
        return_stack.Push() = std::make_pair(source, dest);
    }

    bool CheckReturnStack() {
        auto &entry = return_stack.Peek();
        if (entry.first == buffer) {
            entry.first = nullptr;
            buffer = entry.second;
            return_stack.Pop();
            return true;
        }
        return false;
    }

    // Result is true, push a jump over the false part of the branch
    void PushIfAddress(const uint32_t *source, const uint32_t *dest) {
        if_stack.Push() = std::make_pair(source, dest);
    }

    bool CheckIfStack() {
        auto &entry = if_stack.Peek();
        if (entry.first == buffer) {
            entry.first = nullptr;
            buffer = entry.second;
            if_stack.Pop();
            return true;
        }
        return false;
    }
};

// 0: no offset, 1: dynamic offset
constexpr uint32_t OFFSET_BITS = 1;

struct NoOffset {
    static constexpr uint32_t BITS = 0;

    static constexpr uint32_t Get(OperandBuffer &b) {
        return 0;
    }
};

struct DynamicOffset {
    static constexpr uint32_t BITS = 1;

    static uint32_t Get(OperandBuffer &b) {
        return b.Offset();
    }
};

// nso - (n: Negate, s: Dynamic Swizzle, o: Offset)
constexpr uint32_t SRC_BITS = 2 + OFFSET_BITS;

template <bool Negate, bool Swizzle, typename Offset>
struct Src;

template <typename Offset>
struct Src<false, false, Offset> {
    static constexpr uint32_t BITS = Offset::BITS;

    const Reg4<float> * __restrict reg;

    Src(OperandBuffer &b) : reg(b.Reg(Offset::Get(b))) {}

    const Reg4<float> & __restrict operator*() const {
        return *reg;
    }
};

template <typename Offset>
struct Src<false, true, Offset> {
    static constexpr uint32_t BITS = Offset::BITS | 2;

    OperandBuffer::SwizzleReg reg;

    Src(OperandBuffer &b) : reg(b.InputSwizzle(Offset::Get(b))) {}

    Reg4<float> operator*() const {
        return *reg;
    }
};

template <bool Swizzle, typename Offset>
struct Src<true, Swizzle, Offset> : Src<false, Swizzle, Offset> {
    static constexpr uint32_t BITS = Src<false, Swizzle, Offset>::BITS | 4;

    Src(OperandBuffer &b) : Src<false, Swizzle, Offset>(b) {}

    Reg4<float> operator*() const {
        return -**static_cast<const Src<false, Swizzle, Offset>*>(this);
    }
};

template <typename Src>
static constexpr bool IsSrcNegated;

template <bool Negate, bool Swizzle, typename Offset>
static constexpr bool IsSrcNegated<Src<Negate, Swizzle, Offset>> = Negate;

template <typename Src>
static constexpr bool IsSrcSwizzled;

template <bool Negate, bool Swizzle, typename Offset>
static constexpr bool IsSrcSwizzled<Src<Negate, Swizzle, Offset>> = Swizzle;

template <typename Src>
static constexpr bool IsSrcOffsetDynamic;

template <bool Negate, bool Swizzle>
static constexpr bool IsSrcOffsetDynamic<Src<Negate, Swizzle, DynamicOffset>> = true;

template <bool Negate, bool Swizzle, typename Offset>
static constexpr bool IsSrcOffsetDynamic<Src<Negate, Swizzle, Offset>> = false;

// wzyx - if all zero, dynamic mask
constexpr uint32_t MASK_BITS = 4;

template <bool X, bool Y, bool Z, bool W>
struct ConstMask {
    static constexpr uint32_t BITS = (X ? 1 : 0) | (Y ? 2 : 0) | (Z ? 4 : 0) | (W ? 8 : 0);

    static constexpr uint32_t Of(OperandBuffer &b) {
        return BITS;
    }

    template <typename T>
    static constexpr T Apply(const T &a, const T &b, OperandBuffer &o) {
        if constexpr (X && Y && Z && W) {
            return a;
        } else if constexpr (!(X || Y || Z || W)) {
            return b;
        } else {
            return {
                X ? a.x : b.x,
                Y ? a.y : b.y,
                Z ? a.z : b.z,
                W ? a.w : b.w,
            };
        }
    }
};

using NoMask = ConstMask<true, true, true, true>;

struct DynamicMask {
    static constexpr uint32_t BITS = 0;

    static uint32_t Of(OperandBuffer &b) {
        return b.Value();
    }

    template <typename T>
    static constexpr T Apply(const T &a, const T &b, OperandBuffer &o) {
        return a.Select(b, *o.Mask());
    }
};

template <typename Mask>
struct Dest {
    static constexpr uint32_t BITS = Mask::BITS;

    static void Store(OperandBuffer &b, const Reg4<float> &src) {
        auto &out = *b.Reg();
        out = Mask::Apply(src, out, b);
    }

    static void StoreAddress(OperandBuffer &b, const Reg4<float> &src) {
        auto out = b.Pointer<int32_t>();
        auto mask = Mask::Of(b);
        if (mask & 1) out[0] = src.x;
        if (mask & 2) out[1] = src.y;
    }
};

static void StoreCompare(OperandBuffer &b, const Reg4<float> &lhs, const Reg4<float> &rhs) {
    bool xEQ = lhs.x == rhs.x;
    bool xLT = lhs.x < rhs.x;
    bool xLE = lhs.x <= rhs.x;

    bool yEQ = lhs.y == rhs.y;
    bool yLT = lhs.y < rhs.y;
    bool yLE = lhs.y <= rhs.y;

    auto compare = b.Offset();
    auto cmpX = compare & 0xF;
    auto cmpY = compare >> 8;

    bool *c = b.Pointer<bool>();

    switch (cmpX) {
    case 0:
        c[0] = xEQ;
        break;
    case 1:
        c[0] = !xEQ;
        break;
    case 2:
        c[0] = xLT;
        break;
    case 3:
        c[0] = xLE;
        break;
    case 4:
        c[0] = !xLE;
        break;
    case 5:
        c[0] = !xLT;
        break;
    }

    switch (cmpY) {
    case 0:
        c[1] = yEQ;
        break;
    case 1:
        c[1] = !yEQ;
        break;
    case 2:
        c[1] = yLT;
        break;
    case 3:
        c[1] = yLE;
        break;
    case 4:
        c[1] = !yLE;
        break;
    case 5:
        c[1] = !yLT;
        break;
    }
}

template <typename Dest>
static constexpr bool IsDestMaskDynamic;

template <>
constexpr bool IsDestMaskDynamic<Dest<DynamicMask>> = true;

template <typename Mask>
static constexpr bool IsDestMaskDynamic<Dest<Mask>> = false;

// TODO: Dynamic Cond

constexpr uint32_t COND_BITS = 5;

template <uint32_t Index, bool RefX, bool RefY>
struct Cond {
    static constexpr uint32_t BITS = Index | (RefX ? 8 : 0) | (RefY ? 4 : 0);

    static bool Test(OperandBuffer &b) {
        auto cond = b.Pointer<bool>();

        bool x = cond[0] == RefX;
        bool y = cond[1] == RefY;

        if constexpr (Index == 0) {
            return x || y;
        } else if constexpr (Index == 1) {
            return x && y;
        } else if constexpr (Index == 2) {
            return x;
        } else if constexpr (Index == 3) {
            return y;
        }
    }
};

struct DynamicCond {
    static constexpr uint32_t BITS = 16 | 15;

    static bool Test(OperandBuffer &b) {
        auto cond = b.Pointer<bool>();

        // Layout: abcd y x
        // a - x || y
        // b - x && y
        // c - x
        // d - y
        // x - invx
        // y - invy
        auto flags = b.Value();

        unsigned int test = (cond[0] ? 1 : 0) | (cond[1] ? 2 : 0);
        test = (test ^ flags) & 3;
        unsigned int testHi = (((test >> 1) & test) << 2) | (((test >> 1) | test) << 3);
        test |= testHi;
        return test & (flags >> 2);
    }

    static uint32_t Encode(uint32_t index, bool refX, bool refY) {
        return (refX ? 0 : 1) | (refY ? 0 : 2) | ((3 - index) << 2);
    }
};

template <typename Cond>
static constexpr bool IsCondDynamic;

template <>
constexpr bool IsCondDynamic<DynamicCond> = true;

template <typename Cond>
static constexpr bool IsCondDynamic = false;

struct InstructionInfo {
    bool CheckReturn() const {
        return flags & 1;
    }

    void CheckReturn(bool v) {
        flags = (flags & ~1) | (v ? 1 : 0);
    }

    bool CheckElse() const {
        return flags & 2;
    }

    void CheckElse(bool v) {
        flags = (flags & ~2) | (v ? 2 : 0);
    }

    bool CheckLoop() const {
        return flags & 4;
    }

    void CheckLoop(bool v) {
        flags = (flags & ~4) | (v ? 4 : 0);
    }

    void RecordOffset(uint32_t offset) {
        buffer_offset = offset;
    }

    uint32_t Offset() const {
        return check_buffer_offset;
    }

    uint32_t CheckFlags() const {
        return flags;
    }

    void RecordCheckSource(uint32_t offset) {
        check_buffer_offset = offset;
    }

    uint32_t CheckSource() const {
        return check_buffer_offset;
    }
private:
    uint32_t flags = 0;
    uint32_t buffer_offset = 0;
    uint32_t check_buffer_offset = 0;
};

class CompilerContext {
private:
    const std::array<uint32_t, 4096> &swizzle_data;
    const std::array<uint32_t, 4096> &program;
    const ShaderSetup& setup;
    UnitState& state;

    std::vector<uint32_t> &buffer;

    std::unique_ptr<std::array<InstructionInfo, 4096>> iinfo = std::make_unique<std::array<InstructionInfo, 4096>>();

    std::vector<uint32_t> fixups;

    static constexpr uint32_t Zero = 0;
public:
    CompilerContext(const ShaderSetup &setup, UnitState &state, std::vector<uint32_t> &buffer) :
        setup(setup),
        state(state),
        buffer(buffer),
        swizzle_data(setup.swizzle_data),
        program(setup.program_code) {}

    InstructionInfo &Instr(uint32_t i) {
        return (*iinfo)[i];
    }

    const InstructionInfo &Instr(uint32_t i) const {
        return (*iinfo)[i];
    }

    template <typename T>
    void PushWord(T word) {
        static_assert(sizeof(T) == sizeof(Word));
        buffer.emplace_back(reinterpret_cast<Word>(word));
    }

    // TODO: Don't store pointers into setup and state
    void Call(InstructionFn func) {
        LOG_INFO(HW_GPU, "Call {}", reinterpret_cast<void*>(func));

        buffer.emplace_back(reinterpret_cast<uint32_t>(func));
    }

    void PushRegOffset(uint32_t index) {
        LOG_INFO(HW_GPU, "PushRegOffset {}", index);

        if (index == 0) {
            buffer.emplace_back(reinterpret_cast<uint32_t>(&Zero));
        } else {
            buffer.emplace_back(reinterpret_cast<uint32_t>(&state.address_registers[index - 1]));
        }
    }
    void PushSwizzledRegOffset(uint32_t index) {
        LOG_INFO(HW_GPU, "PushSwizzledRegOffset {}", index);

        if (index == 0) {
            buffer.emplace_back(reinterpret_cast<uint32_t>(&Zero));
        } else {
            buffer.emplace_back(reinterpret_cast<uint32_t>(&state.address_registers[index - 1]));
        }
    }

    const Reg4<float> *LookupSourceRegister(SourceRegister reg) {
        switch (reg.GetRegisterType()) {
        case RegisterType::Input:
            LOG_INFO(HW_GPU, "Lookup v{}", reg.GetIndex());
            return reinterpret_cast<const Reg4<float>*>(&state.registers.input[reg.GetIndex()].x);

        case RegisterType::Temporary:
            LOG_INFO(HW_GPU, "Lookup r{}", reg.GetIndex());
            return reinterpret_cast<const Reg4<float>*>(&state.registers.temporary[reg.GetIndex()].x);

        case RegisterType::FloatUniform:
            LOG_INFO(HW_GPU, "Lookup c{}", reg.GetIndex());
            return reinterpret_cast<const Reg4<float>*>(&setup.uniforms.f[reg.GetIndex()].x);

        default:
            static Reg4<float> ZeroReg = {0.0, 0.0, 0.0, 0.0};
            return &ZeroReg;
        }
    }

    void PushReg(SourceRegister reg) {
        auto ptr = LookupSourceRegister(reg);

        LOG_INFO(HW_GPU, "PushReg");

        buffer.emplace_back(reinterpret_cast<uint32_t>(ptr));
    }
    void PushSwizzledReg(SourceRegister reg, Selector x, Selector y, Selector z, Selector w) {
        auto ptr = LookupSourceRegister(reg);

#ifdef CITRA_NEON
        LOG_INFO(HW_GPU, "PushSwizzledReg(NEON) {}, {}, {}, {}", (uint32_t)x, (uint32_t)y, (uint32_t)z, (uint32_t)w);
        uint32_t index = 0;
        index |= ((uint32_t)w) << 0;
        index |= ((uint32_t)z) << 2;
        index |= ((uint32_t)y) << 4;
        index |= ((uint32_t)x) << 6;
        buffer.emplace_back(reinterpret_cast<Word>(ptr));
        buffer.emplace_back(reinterpret_cast<Word>(&NeonSwizzles[index]));
#else
        LOG_INFO(HW_GPU, "PushSwizzledReg {}, {}, {}, {}", (uint32_t)x, (uint32_t)y, (uint32_t)z, (uint32_t)w);
        buffer.emplace_back(reinterpret_cast<uint32_t>(&ptr->v[(uint32_t)x]));
        buffer.emplace_back(reinterpret_cast<uint32_t>(&ptr->v[(uint32_t)y]));
        buffer.emplace_back(reinterpret_cast<uint32_t>(&ptr->v[(uint32_t)z]));
        buffer.emplace_back(reinterpret_cast<uint32_t>(&ptr->v[(uint32_t)w]));
#endif
    }

    void PushDestReg(DestRegister dest) {
        static Reg4<float> ZeroReg = {0.0, 0.0, 0.0, 0.0};
        Reg4<float> *ptr = (dest < 0x10)
            ? reinterpret_cast<Reg4<float>*>(&state.registers.output[dest.GetIndex()][0])
            : (dest < 0x20)
                    ? reinterpret_cast<Reg4<float>*>(&state.registers.temporary[dest.GetIndex()][0])
                    : &ZeroReg;

        if (dest < 0x10) {
            LOG_INFO(HW_GPU, "PushDestReg o{}", dest.GetIndex());
        } else {
            LOG_INFO(HW_GPU, "PushDestReg r{}", dest.GetIndex());
        }

        buffer.emplace_back(reinterpret_cast<uint32_t>(ptr));
    }

    void PushMask(SwizzlePattern swizzle) {
        const Reg4<uint32_t> *mask = &SelectMasks[swizzle.dest_mask.Value()];
        buffer.emplace_back(reinterpret_cast<Word>(mask));
    }

    void PushPlainMask(SwizzlePattern swizzle) {
        uint32_t bits = 0;

        bits |= swizzle.DestComponentEnabled(0) ? 1 : 0;
        bits |= swizzle.DestComponentEnabled(1) ? 2 : 0;
        bits |= swizzle.DestComponentEnabled(2) ? 4 : 0;
        bits |= swizzle.DestComponentEnabled(3) ? 8 : 0;

        buffer.emplace_back(bits);
    }

    void PushAddressRegs() {
        buffer.emplace_back(reinterpret_cast<uint32_t>(&state.address_registers[0]));
    }

    void PushCompareModes(Instruction::Common::CompareOpType::Op x, Instruction::Common::CompareOpType::Op y) {
        buffer.emplace_back(static_cast<uint32_t>(x) | (static_cast<uint32_t>(y) << 8));
    }

    void PushConditionRegs() {
        buffer.emplace_back(reinterpret_cast<uint32_t>(&state.conditional_code[0]));
    }

    void PushBoolUniform(uint32_t index) {
        buffer.emplace_back(reinterpret_cast<Word>(&setup.uniforms.b[index]));
    }

    void PushDestination(uint32_t pc) {
        buffer.emplace_back(pc);
        fixups.emplace_back(buffer.size() - 1);
    }

    void PushCheckSource(uint32_t pc) {
        buffer.emplace_back(pc | (1 << 20));
        fixups.emplace_back(buffer.size() - 1);
    }

    void PushInstrInfoFlags(uint32_t pc) {
        buffer.emplace_back(Instr(pc).CheckFlags());
    }

    void TraceControlFlow();

    bool CompileInstruction(uint32_t pc);

    void ApplyFixups();

private:
    template <typename F>
    bool CompileFormat1(Instruction instr, SwizzlePattern swizzle, F func);

    template <typename F>
    bool CompileFormat1u(Instruction instr, SwizzlePattern swizzle, F func, uint32_t mask = 15);

    template <typename F>
    bool CompileFormat5(Instruction instr, SwizzlePattern swizzle, F func);
};

#define IMPORT(...) static constexpr uint32_t BITS = __VA_ARGS__::BITS; \
    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) { __VA_ARGS__::Emit(cc, instr, swizzle); };

template <typename Src1, typename Src2, typename Dest, InstructionFn Do>
struct Format1 {
    static constexpr uint32_t BITS = Dest::BITS | (Src2::BITS << MASK_BITS) | (Src1::BITS << (MASK_BITS + SRC_BITS));

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(Do);

        bool is_inverted = instr.opcode.Value().GetInfo().subtype & OpCode::Info::SrcInversed;

        if (IsSrcSwizzled<Src1>) {
            if (!is_inverted && IsSrcOffsetDynamic<Src1>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc1(is_inverted),
                swizzle.src1_selector_0.Value(),
                swizzle.src1_selector_1.Value(),
                swizzle.src1_selector_2.Value(),
                swizzle.src1_selector_3.Value()
            );
        } else {
            if (!is_inverted && IsSrcOffsetDynamic<Src1>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc1(is_inverted));
        }

        if (IsSrcSwizzled<Src2>) {
            if (is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc2(is_inverted),
                swizzle.src2_selector_0.Value(),
                swizzle.src2_selector_1.Value(),
                swizzle.src2_selector_2.Value(),
                swizzle.src2_selector_3.Value()
            );
        } else {
            if (is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc2(is_inverted));
        }

        _ PushDestReg(instr.common.dest);

        if (IsDestMaskDynamic<Dest>) {
            _ PushMask(swizzle);
        }
    }
};

template <typename Src, typename Dest, InstructionFn Do>
struct Format1u {
    static constexpr uint32_t BITS = Dest::BITS | (Src::BITS << MASK_BITS);

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(Do);

        bool is_inverted = instr.opcode.Value().GetInfo().subtype & OpCode::Info::SrcInversed;

        if (IsSrcSwizzled<Src>) {
            if (!is_inverted && IsSrcOffsetDynamic<Src>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc1(is_inverted),
                swizzle.src1_selector_0.Value(),
                swizzle.src1_selector_1.Value(),
                swizzle.src1_selector_2.Value(),
                swizzle.src1_selector_3.Value()
            );
        } else {
            if (!is_inverted && IsSrcOffsetDynamic<Src>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc1(is_inverted));
        }

        _ PushDestReg(instr.common.dest);

        if (IsDestMaskDynamic<Dest>) {
            _ PushMask(swizzle);
        }
    }
};

template <typename Src, typename Dest, InstructionFn Do>
struct Format1a {
    static constexpr uint32_t BITS = Dest::BITS | (Src::BITS << MASK_BITS);

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(Do);

        bool is_inverted = instr.opcode.Value().GetInfo().subtype & OpCode::Info::SrcInversed;

        if (IsSrcSwizzled<Src>) {
            if (!is_inverted && IsSrcOffsetDynamic<Src>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc1(is_inverted),
                swizzle.src1_selector_0.Value(),
                swizzle.src1_selector_1.Value(),
                swizzle.src1_selector_2.Value(),
                swizzle.src1_selector_3.Value()
            );
        } else {
            if (!is_inverted && IsSrcOffsetDynamic<Src>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc1(is_inverted));
        }

        _ PushAddressRegs();

        if (IsDestMaskDynamic<Dest>) {
            _ PushPlainMask(swizzle);
        }
    }
};

template <typename Src1, typename Src2, InstructionFn Do>
struct Format1c {
    static constexpr uint32_t BITS = Src2::BITS | (Src1::BITS << SRC_BITS);

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(Do);

        bool is_inverted = instr.opcode.Value().GetInfo().subtype & OpCode::Info::SrcInversed;

        if (IsSrcSwizzled<Src1>) {
            if (!is_inverted && IsSrcOffsetDynamic<Src1>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc1(is_inverted),
                swizzle.src1_selector_0.Value(),
                swizzle.src1_selector_1.Value(),
                swizzle.src1_selector_2.Value(),
                swizzle.src1_selector_3.Value()
            );
        } else {
            if (!is_inverted && IsSrcOffsetDynamic<Src1>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc1(is_inverted));
        }

        if (IsSrcSwizzled<Src2>) {
            if (is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushSwizzledRegOffset(instr.common.address_register_index);
            }
            _ PushSwizzledReg(
                instr.common.GetSrc2(is_inverted),
                swizzle.src2_selector_0.Value(),
                swizzle.src2_selector_1.Value(),
                swizzle.src2_selector_2.Value(),
                swizzle.src2_selector_3.Value()
            );
        } else {
            if (is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.common.GetSrc2(is_inverted));
        }

        _ PushCompareModes(instr.common.compare_op.x.Value(), instr.common.compare_op.y.Value());

        _ PushConditionRegs();
    }
};

template <typename Src1, typename Src2, typename Src3, typename Dest, InstructionFn Do>
struct Format5 {
    static constexpr uint32_t BITS = Dest::BITS | (Src2::BITS << MASK_BITS) | (Src1::BITS << (MASK_BITS + SRC_BITS)) | (Src3::BITS << (MASK_BITS + SRC_BITS + SRC_BITS));

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(Do);

        bool is_inverted = instr.opcode.Value().GetInfo().subtype & OpCode::Info::SrcInversed;

        if (IsSrcSwizzled<Src1>) {
            _ PushSwizzledReg(
                instr.mad.GetSrc1(is_inverted),
                swizzle.src1_selector_0.Value(),
                swizzle.src1_selector_1.Value(),
                swizzle.src1_selector_2.Value(),
                swizzle.src1_selector_3.Value()
            );
        } else {
            _ PushReg(instr.mad.GetSrc1(is_inverted));
        }

        if (IsSrcSwizzled<Src2>) {
            if (!is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushSwizzledRegOffset(instr.mad.address_register_index);
            }
            _ PushSwizzledReg(
                instr.mad.GetSrc2(is_inverted),
                swizzle.src2_selector_0.Value(),
                swizzle.src2_selector_1.Value(),
                swizzle.src2_selector_2.Value(),
                swizzle.src2_selector_3.Value()
            );
        } else {
            if (!is_inverted && IsSrcOffsetDynamic<Src2>) {
                _ PushRegOffset(instr.mad.address_register_index);
            }
            _ PushReg(instr.mad.GetSrc2(is_inverted));
        }

        if (IsSrcSwizzled<Src3>) {
            if (is_inverted && IsSrcOffsetDynamic<Src3>) {
                _ PushSwizzledRegOffset(instr.mad.address_register_index);
            }
            _ PushSwizzledReg(
                instr.mad.GetSrc2(is_inverted),
                swizzle.src3_selector_0.Value(),
                swizzle.src3_selector_1.Value(),
                swizzle.src3_selector_2.Value(),
                swizzle.src3_selector_3.Value()
            );
        } else {
            if (is_inverted && IsSrcOffsetDynamic<Src3>) {
                _ PushRegOffset(instr.common.address_register_index);
            }
            _ PushReg(instr.mad.GetSrc3(is_inverted));
        }

        _ PushDestReg(instr.mad.dest);

        if (IsDestMaskDynamic<Dest>) {
            _ PushMask(swizzle);
        }
    }
};

#define FORCE_ARM __attribute__((target("arm")))

template <typename Src1, typename Src2, typename Dest>
struct ADD {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        Dest::Store(b, *op1 + *op2);
        b.Continue();
    }

    IMPORT(Format1<Src1, Src2, Dest, &Do>);
};

template <typename Src1, typename Src2, typename Dest>
struct MUL {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        Dest::Store(b, *op1 * *op2);
        b.Continue();
    }

    IMPORT(Format1<Src1, Src2, Dest, &Do>);
};

template <typename Src1, typename Src2, typename Dest>
struct DP3 {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        Dest::Store(b, Reg4<float>::Splat(Reg4<float>::Dot3(*op1, *op2)));
        b.Continue();
    }

    IMPORT(Format1<Src1, Src2, Dest, &Do>);
};

template <typename Src1, typename Src2, typename Dest>
struct DP4 {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        Dest::Store(b, Reg4<float>::Splat(Reg4<float>::Dot4(*op1, *op2)));
        b.Continue();
    }

    IMPORT(Format1<Src1, Src2, Dest, &Do>);
};

template <typename Src, typename Dest>
struct MOV {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src src(b);
        Dest::Store(b, *src);
        b.Continue();
    }

    IMPORT(Format1u<Src, Dest, &Do>);
};

template <typename Src, typename Dest>
struct MOVA {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src src(b);
        Dest::StoreAddress(b, *src);
        b.Continue();
    }

    IMPORT(Format1a<Src, Dest, &Do>);
};

template <typename Src1, typename Src2>
struct CMP {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        StoreCompare(b, *op1, *op2);
        b.Continue();
    }

    IMPORT(Format1c<Src1, Src2, &Do>);
};

template <typename Src1, typename Src2, typename Src3, typename Dest>
struct MAD {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        Src1 op1(b);
        Src2 op2(b);
        Src3 op3(b);
        Dest::Store(b, (*op1 * *op2) + *op3);
        b.Continue();
    }

    IMPORT(Format5<Src1, Src2, Src3, Dest, &Do>);
};

struct CALL {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto returnSource = b.JumpTarget();
        auto target = b.JumpTarget();
        b.PushReturnAddress(returnSource);
        b.Jump(target);
    }

    static constexpr uint32_t BITS = 0;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
        _ PushCheckSource(instr.flow_control.dest_offset + instr.flow_control.num_instructions);
        _ PushDestination(instr.flow_control.dest_offset);
    }
};

struct CALLU {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto returnSource = b.JumpTarget();
        auto target = b.JumpTarget();
        if (*b.Pointer<bool>()) {
            b.PushReturnAddress(returnSource);
            b.Jump(target);
        } else {
            b.Continue();
        }
    }

    static constexpr uint32_t BITS = 0;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
        _ PushCheckSource(instr.flow_control.dest_offset + instr.flow_control.num_instructions);
        _ PushDestination(instr.flow_control.dest_offset);
        _ PushBoolUniform(instr.flow_control.bool_uniform_id);
    }
};

template <typename Cond>
struct CALLC {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto returnSource = b.JumpTarget();
        auto target = b.JumpTarget();
        if (Cond::Test(b)) {
            b.PushReturnAddress(returnSource);
            b.Jump(target);
        } else {
            b.Continue();
        }
    }

    static constexpr uint32_t BITS = Cond::BITS;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
        _ PushCheckSource(instr.flow_control.dest_offset + instr.flow_control.num_instructions);
        _ PushDestination(instr.flow_control.dest_offset);
        _ PushConditionRegs();
        if (IsCondDynamic<Cond>) {
            _ PushWord(Cond::Encode(
                static_cast<uint32_t>(instr.flow_control.op),
                instr.flow_control.refx,
                instr.flow_control.refy
            ));
        }
    }
};

struct IFU {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto elseSource = b.JumpTarget();
        auto elseCheckSource = b.JumpTarget();
        auto elseTarget = b.JumpTarget();
        if (*b.Pointer<bool>()) {
            b.PushIfAddress(elseCheckSource, elseTarget);
            b.Continue();
        } else {
            b.Jump(elseSource);
        }
    }

    static constexpr uint32_t BITS = 0;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
        _ PushDestination(instr.flow_control.dest_offset);
        _ PushCheckSource(instr.flow_control.dest_offset);
        _ PushDestination(instr.flow_control.dest_offset + instr.flow_control.num_instructions);
        _ PushBoolUniform(instr.flow_control.bool_uniform_id);
    }
};

template <typename Cond>
struct IFC {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto elseSource = b.JumpTarget();
        auto elseCheckSource = b.JumpTarget();
        auto elseTarget = b.JumpTarget();
        if (Cond::Test(b)) {
            b.PushIfAddress(elseCheckSource, elseTarget);
            b.Continue();
        } else {
            b.Jump(elseSource);
        }
    }

    static constexpr uint32_t BITS = Cond::BITS;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
        _ PushDestination(instr.flow_control.dest_offset);
        _ PushCheckSource(instr.flow_control.dest_offset);
        _ PushDestination(instr.flow_control.dest_offset + instr.flow_control.num_instructions);
        _ PushConditionRegs();
        if (IsCondDynamic<Cond>) {
            _ PushWord(Cond::Encode(
                static_cast<uint32_t>(instr.flow_control.op),
                instr.flow_control.refx,
                instr.flow_control.refy
            ));
        }
    }
};

struct CHKSTK {
    FORCE_ARM
    static void Do(OperandBuffer &b) {
        auto flags = b.Value();

        if (
            ((flags & 1) && b.CheckReturnStack()) ||
            ((flags & 2) && b.CheckIfStack()) // || ...
        ) {}

        b.Continue();
    }

    static constexpr uint32_t BITS = 0;

    static void Emit(CompilerContext &cc, uint32_t pc) {
        _ Call(&Do);
        _ PushInstrInfoFlags(pc);
    }
};

struct END {
    FORCE_ARM
    static void Do(OperandBuffer &b) {}

    static constexpr uint32_t BITS = 0;

    static void Emit(CompilerContext &cc, Instruction instr, SwizzlePattern swizzle) {
        _ Call(&Do);
    }
};

void CompilerContext::TraceControlFlow() {
    uint32_t pc = 0;
    while (pc < 4096) {
        const Instruction instr = {program[pc]};
        switch (instr.opcode.Value().EffectiveOpCode()) {
        case OpCode::Id::CALL:
        case OpCode::Id::CALLC:
        case OpCode::Id::CALLU: {
            auto ret = instr.flow_control.dest_offset.Value() +
                        instr.flow_control.num_instructions.Value();
            if (ret < 4096) {
                Instr(ret).CheckReturn(true);
            }
        }
        case OpCode::Id::IFU:
        case OpCode::Id::IFC: {
            auto els = instr.flow_control.dest_offset.Value();
            if (els < 4096) {
                Instr(els).CheckElse(true);
            }
        }
        case OpCode::Id::LOOP: {
            auto cont = instr.flow_control.dest_offset.Value() + 1;
            if (cont < 4096) {
                Instr(cont).CheckLoop(true);
            }
        }
        }

        pc++;
    }
}

#define FORMAT1(name) \
    MATCH(name<Src<false, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, NoOffset>, Dest<NoMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, NoOffset>, Dest<NoMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, NoOffset>, Dest<NoMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, NoOffset>, Dest<NoMask>>)

#define FORMAT1u(name) \
    MATCH(name<Src<false, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Dest<DynamicMask>>)

#define FORMAT1a(name) \
    MATCH(name<Src<false, false, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(name<Src<false, true, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(name<Src<false, false, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(name<Src<false, true, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(name<Src<false, false, NoOffset>, Dest<ConstMask<true, true, false, false>>>) \
    MATCH(name<Src<false, true, NoOffset>, Dest<ConstMask<true, true, false, false>>>) \
    MATCH(name<Src<true, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, false, DynamicOffset>, Dest<DynamicMask>>)

#define FORMAT5(name) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, DynamicOffset>, Src<false, true, NoOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, DynamicOffset>, Src<true, true, NoOffset>, Dest<DynamicMask>>)

#define FORMAT5i(name) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, NoOffset>, Src<false, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<false, true, NoOffset>, Src<true, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, NoOffset>, Src<false, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<false, true, NoOffset>, Src<true, true, NoOffset>, Src<true, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, NoOffset>, Src<false, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<false, true, NoOffset>, Src<true, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, NoOffset>, Src<false, true, DynamicOffset>, Dest<DynamicMask>>) \
    MATCH(name<Src<true, true, NoOffset>, Src<true, true, NoOffset>, Src<true, true, DynamicOffset>, Dest<DynamicMask>>)

#define INSTR_ADD \
    FORMAT1(ADD)

#define INSTR_MUL \
    FORMAT1(MUL)

#define INSTR_DP3 \
    FORMAT1(DP3)

#define INSTR_DP4 \
    FORMAT1(DP4) \
    MATCH(DP4<Src<false, false, NoOffset>, Src<false, false, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(DP4<Src<false, false, NoOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(DP4<Src<false, false, NoOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, false, true, false>>>) \
    MATCH(DP4<Src<false, false, NoOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, false, false, true>>>) \
    MATCH(DP4<Src<false, false, DynamicOffset>, Src<false, false, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(DP4<Src<false, false, DynamicOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(DP4<Src<false, false, DynamicOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, false, true, false>>>) \
    MATCH(DP4<Src<false, false, DynamicOffset>, Src<false, false, NoOffset>, Dest<ConstMask<false, false, false, true>>>)

#define INSTR_MOV \
    FORMAT1u(MOV) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<DynamicMask>>) \
    MATCH(MOV<Src<false, true, NoOffset>, Dest<NoMask>>) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<NoMask>>) \
    MATCH(MOV<Src<false, true, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<ConstMask<true, false, false, false>>>) \
    MATCH(MOV<Src<false, true, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<ConstMask<false, true, false, false>>>) \
    MATCH(MOV<Src<false, true, NoOffset>, Dest<ConstMask<false, false, true, false>>>) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<ConstMask<false, false, true, false>>>) \
    MATCH(MOV<Src<false, true, NoOffset>, Dest<ConstMask<false, false, false, true>>>) \
    MATCH(MOV<Src<false, false, NoOffset>, Dest<ConstMask<false, false, false, true>>>)

#define INSTR_MOVA \
    FORMAT1a(MOVA)

#define INSTR_MAD \
    FORMAT5(MAD) \
    FORMAT5i(MAD)

bool CompilerContext::CompileInstruction(uint32_t pc) {
    const Instruction instr = {program[pc]};
    const SwizzlePattern swizzle = {swizzle_data[instr.common.operand_desc_id]};

#define MATCH(...) case __VA_ARGS__::BITS: LOG_WARNING(HW_GPU, #__VA_ARGS__); __VA_ARGS__::Emit(*this, instr, swizzle); return true;

    if (instr.opcode.Value().EffectiveOpCode() != OpCode::Id::NOP)
        LOG_WARNING(HW_GPU, "Instruction: 0x{:02x} (0x{:08x})",
            static_cast<u32>(instr.opcode.Value().EffectiveOpCode()), instr.hex);

    Instr(pc).RecordOffset(buffer.size());

    if (Instr(pc).CheckFlags() & 7) {
        CHKSTK::Emit(*this, pc);
    }

    Instr(pc).RecordCheckSource(buffer.size());

    switch (instr.opcode.Value().EffectiveOpCode()) {
    case OpCode::Id::ADD: {
        if (CompileFormat1(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_ADD
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::MUL: {
        if (CompileFormat1(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_MUL
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::DP3: {
        if (CompileFormat1(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_DP3
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::DP4: {
        if (CompileFormat1(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_DP4
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::MOV: {
        if (CompileFormat1u(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_MOV
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::MOVA: {
        if (CompileFormat1u(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_MOVA
            }
            return false;
        }, 3 /* only X & Y have an effect */)) return true;
        break;
    }
    case OpCode::Id::MAD: {
        const SwizzlePattern swizzle = {swizzle_data[instr.mad.operand_desc_id]};
        if (CompileFormat5(instr, swizzle, [=](uint32_t bits) {
            switch (bits) {
                INSTR_MAD
            }
            return false;
        })) return true;
        break;
    }
    case OpCode::Id::CALL: {
        CALL::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::CALLC: {
        // TODO: Support non-dynamic conditionals
        CALLC<DynamicCond>::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::CALLU: {
        CALLU::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::IFC: {
        // TODO: Support non-dynamic conditionals
        IFC<DynamicCond>::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::IFU: {
        // TODO: Support non-dynamic conditionals
        IFU::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::END: {
        END::Emit(*this, instr, swizzle);
        return true;
    }
    case OpCode::Id::EX2: {
        // TODO: Support log2
        return true;
    }
    case OpCode::Id::LG2: {
        // TODO: Support log2
        return true;
    }
    case OpCode::Id::CMP: {
        // TODO: Support compares
        return true;
    }
    case OpCode::Id::FLR: {
        // TODO: Support floor
        return true;
    }
    case OpCode::Id::MAX: {
        // TODO: Support max
        return true;
    }
    case OpCode::Id::RCP: {
        // TODO: Support 1/x
        return true;
    }
    case OpCode::Id::RSQ: {
        // TODO: Support reverse square root
        return true;
    }
    case OpCode::Id::JMPC: {
        // TODO: Support jumps
        return true;
    }
    case OpCode::Id::JMPU: {
        // TODO: Support jumps
        return true;
    }
    case OpCode::Id::LOOP: {
        // TODO: Support loops
        return true;
    }
    case OpCode::Id::SLTI: {
        // TODO: Support slti
        return true;
    }
    case OpCode::Id::NOP: {
        return true;
    }
    }

#undef MATCH

    LOG_ERROR(HW_GPU, "Unhandled instruction: 0x{:02x} (0x{:08x})",
        static_cast<u32>(instr.opcode.Value().EffectiveOpCode()), instr.hex);

    return false;
}

#define TRY_BITS if (func(bits)) return true; \
    LOG_WARNING(HW_GPU, "Mask not found: {:016b}", bits);

template <typename F>
bool CompilerContext::CompileFormat1(Instruction instr, SwizzlePattern swizzle, F func) {
    uint32_t bits = 0;

    // Most-specific bits:
    bits |= swizzle.DestComponentEnabled(0) ? 1 : 0;
    bits |= swizzle.DestComponentEnabled(1) ? 2 : 0;
    bits |= swizzle.DestComponentEnabled(2) ? 4 : 0;
    bits |= swizzle.DestComponentEnabled(3) ? 8 : 0;

    bits |= (swizzle.src2_selector_0.Value() != Selector::x ||
        swizzle.src2_selector_1.Value() != Selector::y ||
        swizzle.src2_selector_2.Value() != Selector::z ||
        swizzle.src2_selector_3.Value() != Selector::w) ? 2 << MASK_BITS : 0;
    bits |= swizzle.negate_src2 ? 4 << MASK_BITS : 0;

    bits |= instr.common.address_register_index != 0 ? 1 << (MASK_BITS + SRC_BITS) : 0;
    bits |= (swizzle.src1_selector_0.Value() != Selector::x ||
        swizzle.src1_selector_1.Value() != Selector::y ||
        swizzle.src1_selector_2.Value() != Selector::z ||
        swizzle.src1_selector_3.Value() != Selector::w) ? 2 << (MASK_BITS + SRC_BITS) : 0;
    bits |= swizzle.negate_src1 ? 4 << (MASK_BITS + SRC_BITS) : 0;

    TRY_BITS

    int times = 0;

    while (times <= 1) {
        // Dynamic addressing (src1)
        if (instr.common.address_register_index == 0) {
            bits |= 1 << (MASK_BITS + SRC_BITS);
            TRY_BITS
        }

        // Dynamic swizzle (src2)

        if (!(bits & (2 << MASK_BITS))) {
            bits |= (2 << MASK_BITS);
            TRY_BITS
        }

        // Dynamic swizzle (src1)

        if (!(bits & (2 << (MASK_BITS + SRC_BITS)))) {
            bits |= (2 << (MASK_BITS + SRC_BITS));
            TRY_BITS
        }

        // No mask:
        bits &= ~15;
        TRY_BITS

        times++;
    }

    // Nothing found
    return false;
}

template <typename F>
bool CompilerContext::CompileFormat1u(Instruction instr, SwizzlePattern swizzle, F func, uint32_t mask) {
    uint32_t bits = 0;

    // Most-specific bits:
    bits |= swizzle.DestComponentEnabled(0) ? 1 : 0;
    bits |= swizzle.DestComponentEnabled(1) ? 2 : 0;
    bits |= swizzle.DestComponentEnabled(2) ? 4 : 0;
    bits |= swizzle.DestComponentEnabled(3) ? 8 : 0;

    bits &= mask;

    bits |= instr.common.address_register_index != 0 ? 1 << MASK_BITS : 0;
    bits |= swizzle.src1_selector_0.Value() != Selector::x ||
        swizzle.src1_selector_1.Value() != Selector::y ||
        swizzle.src1_selector_2.Value() != Selector::z ||
        swizzle.src1_selector_3.Value() != Selector::w ? 2 << MASK_BITS : 0;
    bits |= swizzle.negate_src1 ? 4 << MASK_BITS : 0;

    TRY_BITS

    // No mask:
    bits &= ~15;
    TRY_BITS

    // Dynamic addressing (src1)
    if (instr.common.address_register_index == 0) {
        bits |= 1 << MASK_BITS;
        TRY_BITS
    }

    // Dynamic swizzle (src1)
    if (!(bits & (2 << MASK_BITS))) {
        bits |= 2 << MASK_BITS;
        TRY_BITS
    }

    // Nothing found
    return false;
}

template <typename F>
bool CompilerContext::CompileFormat5(Instruction instr, SwizzlePattern swizzle, F func) {
    uint32_t bits = 0;

    // Most-specific bits:
    bits |= swizzle.DestComponentEnabled(0) ? 1 : 0;
    bits |= swizzle.DestComponentEnabled(1) ? 2 : 0;
    bits |= swizzle.DestComponentEnabled(2) ? 4 : 0;
    bits |= swizzle.DestComponentEnabled(3) ? 8 : 0;

    bits |= instr.mad.address_register_index != 0 ? 1 << MASK_BITS : 0;
    bits |= (swizzle.src2_selector_0.Value() != Selector::x ||
        swizzle.src2_selector_1.Value() != Selector::y ||
        swizzle.src2_selector_2.Value() != Selector::z ||
        swizzle.src2_selector_3.Value() != Selector::w) ? 2 << MASK_BITS : 0;
    bits |= swizzle.negate_src2 ? 4 << MASK_BITS : 0;

    bits |= (swizzle.src1_selector_0.Value() != Selector::x ||
        swizzle.src1_selector_1.Value() != Selector::y ||
        swizzle.src1_selector_2.Value() != Selector::z ||
        swizzle.src1_selector_3.Value() != Selector::w) ? 2 << (MASK_BITS + SRC_BITS) : 0;
    bits |= swizzle.negate_src1 ? 4 << (MASK_BITS + SRC_BITS) : 0;

    bits |= (swizzle.src3_selector_0.Value() != Selector::x ||
        swizzle.src3_selector_1.Value() != Selector::y ||
        swizzle.src3_selector_2.Value() != Selector::z ||
        swizzle.src3_selector_3.Value() != Selector::w) ? 2 << (MASK_BITS + SRC_BITS + SRC_BITS) : 0;
    bits |= swizzle.negate_src3 ? 4 << (MASK_BITS + SRC_BITS + SRC_BITS) : 0;

    TRY_BITS

    // No mask:
    bits &= ~15;
    TRY_BITS

    // Dynamic addressing (src2)
    if (instr.mad.address_register_index == 0) {
        bits |= 1 << MASK_BITS;
        TRY_BITS
    }

    // Dynamic swizzle (src2)

    if (!(bits & (2 << MASK_BITS))) {
        bits |= (2 << MASK_BITS);
        TRY_BITS
    }

    // Dynamic swizzle (src1)

    if (!(bits & (2 << (MASK_BITS + SRC_BITS)))) {
        bits |= (2 << (MASK_BITS + SRC_BITS));
        TRY_BITS
    }

    // Dynamic swizzle (src3)

    if (!(bits & (2 << (MASK_BITS + SRC_BITS + SRC_BITS)))) {
        bits |= (2 << (MASK_BITS + SRC_BITS + SRC_BITS));
        TRY_BITS
    }

    // Nothing found
    return false;
}

void CompilerContext::ApplyFixups() {
    for (auto fixup : fixups) {
        auto &code = buffer[fixup];

        LOG_WARNING(HW_GPU, "Fixup at {}: {:03x}", fixup, code);

        if (code & (1 << 20)) {
            code = reinterpret_cast<Word>(&buffer[Instr(code & ~(1 << 20)).CheckSource()]);
        } else {
            code = reinterpret_cast<Word>(&buffer[Instr(code).Offset()]);
        }

        LOG_WARNING(HW_GPU, "Fixup result: {:08x}", code);
    }
}

bool CachedBatch::Compile(const ShaderSetup& setup, UnitState& state) {
    CompilerContext compiler(setup, state, code);

    compiler.TraceControlFlow();

    Word pc = 0;
    while (pc < 4096) {
        if (!compiler.CompileInstruction(pc)) {
            return false;
        }
        pc++;
    }

    compiler.ApplyFixups();

    compiled = true;

    return true;
}

void CachedBatch::Run(const ShaderSetup& setup, UnitState& state, unsigned int entry_point) const {
    OperandBuffer b = { reinterpret_cast<const uint32_t* __restrict>(code.data()) };

    b.Continue();
}

} // namespace Pica::Shader::Fast
