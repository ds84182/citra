// Copyright 2017 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <cmath>
#include <memory>
#include <catch2/catch.hpp>
#include <nihstro/inline_assembly.h>
#include "video_core/shader/shader_fast_interpreter.h"

using float24 = Pica::float24;
using Vec4f = Common::Vec4f;
using CachedBatch = Pica::Shader::Fast::CachedBatch;

using DestRegister = nihstro::DestRegister;
using OpCode = nihstro::OpCode;
using SourceRegister = nihstro::SourceRegister;
using Instruction = nihstro::Instruction;
using SwizzlePattern = nihstro::SwizzlePattern;

using ShaderSetup = Pica::Shader::ShaderSetup;
using UnitState = Pica::Shader::UnitState;

using ProgramArray = std::array<Instruction, Pica::Shader::MAX_PROGRAM_CODE_LENGTH>;
using SwizzleArray = std::array<SwizzlePattern, Pica::Shader::MAX_SWIZZLE_DATA_LENGTH>;

struct CompiledShader {
    std::unique_ptr<ProgramArray> program;
    std::unique_ptr<SwizzleArray> swizzle_data;

    CompiledShader(std::unique_ptr<ProgramArray> &&program, std::unique_ptr<SwizzleArray> &&swizzle_data) :
        program(std::move(program)), swizzle_data(std::move(swizzle_data)) {}
};

struct Cond {
    // TODO: Maybe we should separate out JustX and JustY to a different struct
    using Op = Instruction::FlowControlType::Op;

    Op op;
    bool refx, refy;

    constexpr Cond(bool refx, bool refy, Op op) : refx(refx), refy(refy), op(op) {}

    // TODO: Instead of an invert flag, use prefix ! operator
    static constexpr Cond X(bool invert = false) {
        return Cond(!invert, false, Op::JustX);
    }

    static constexpr Cond Y(bool invert = false) {
        return Cond(false, !invert, Op::JustY);
    }

    void Apply(Instruction &instr) const {
        instr.flow_control.op = op;
        instr.flow_control.refx = refx;
        instr.flow_control.refy = refy;
    }
};

class ShaderBuilder {
public:
    struct Label {
        u32 index;

        constexpr Label(u32 index) : index(index) {}

        static constexpr Label Invalid() {
            return Label(0xFFFFFFFF);
        }

        bool IsValid() const {
            return index != Invalid().index;
        }
    };

    ShaderBuilder() : program(std::make_unique<ProgramArray>()), swizzle_data(std::make_unique<SwizzleArray>()) {
        Instruction nop;
        nop.hex = 0x87000000;
        program->fill(nop);
    }

    Label NewLabel() {
        auto label = Label(labels.size());
        labels.emplace_back(LabelData(label));
        return label;
    }

    Label EmitLabel(Label label) {
        labels[label.index].pc = pc;
        return label;
    }

    struct Dest {
        u32 index;
        u32 mask = 15;

        constexpr Dest(u32 index, u32 mask = 15) : index(index), mask(mask) {}

        #define MASK(name, bits) Dest name() const { return Dest(index, bits); }

        MASK(x, 8);
        MASK(y, 4);
        MASK(z, 2);
        MASK(w, 1);

        MASK(xy, 12);
        MASK(xz, 10);
        MASK(xw, 9);

        MASK(yz, 6);
        MASK(yw, 5);
        MASK(zw, 3);

        MASK(xyz, 14);
        MASK(xyw, 13);
        MASK(xzw, 11);
        MASK(yzw, 7);

        MASK(xyzw, 15);

        #undef MASK

        DestRegister ToDestRegister() {
            return index;
        }
    };

    enum Addr {
        None,
        A1,
        A2,
        AL,
    };

    enum Component : u8 {
        X, Y, Z, W, UND
    };

    struct RawSrc {
        u32 index;
        bool negate = false;
        Addr addr = Addr::None;
        Component selector[4] = {Component::X, Component::Y, Component::Z, Component::W};

        SourceRegister ToSourceRegister() {
            return index;
        }
    };

    static constexpr u8 MakeSelector(Component x, Component y, Component z, Component w) {
        return static_cast<u8>(x) | (static_cast<u8>(y) << 2) | (static_cast<u8>(z) << 4) | (static_cast<u8>(w) << 6);
    }

    struct SrcBase {
        u32 index;
        bool negate = false;
        Addr addr = Addr::None;
    };

    template <Component X = Component::X, Component Y = Component::Y, Component Z = Component::Z, Component W = Component::W>
    struct SSrc : SrcBase {
        // TODO: Decay to RawSrc

        using Self = SSrc<X, Y, Z, W>;

        Self &operator-() {
            negate = !negate;
            return *this;
        }

        operator RawSrc() const {
            RawSrc src;
            src.index = index;
            src.negate = negate;
            src.addr = addr;
            if (X != Component::UND) src.selector[0] = X;
            if (Y != Component::UND) src.selector[1] = Y;
            if (Z != Component::UND) src.selector[2] = Z;
            if (W != Component::UND) src.selector[3] = W;
            return src;
        }
    };

    template <Component X = Component::X, Component Y = Component::Y, Component Z = Component::Z, Component W = Component::W>
    union Src {
        using Self = Src<X, Y, Z, W>;

        Src(u32 index) {
            base = {};
            base.index = index;
        }

        SrcBase base;

        #define _ Component::UND

        SSrc<X, _, _, _> x;
        SSrc<Y, _, _, _> y;
        SSrc<Z, _, _, _> z;
        SSrc<W, _, _, _> w;

        SSrc<X, X, _, _> xx;
        SSrc<X, Y, _, _> xy;
        SSrc<X, Z, _, _> xz;
        SSrc<X, W, _, _> xw;

        SSrc<Y, X, _, _> yx;
        SSrc<Y, Y, _, _> yy;
        SSrc<Y, Z, _, _> yz;
        SSrc<Y, W, _, _> yw;

        SSrc<Z, X, _, _> zx;
        SSrc<Z, Y, _, _> zy;
        SSrc<Z, Z, _, _> zz;
        SSrc<Z, W, _, _> zw;

        SSrc<W, X, _, _> wx;
        SSrc<W, Y, _, _> wy;
        SSrc<W, Z, _, _> wz;
        SSrc<W, W, _, _> ww;

        SSrc<X, X, X, _> xxx;
        SSrc<X, X, Y, _> xxy;
        SSrc<X, X, Z, _> xxz;
        SSrc<X, X, W, _> xxw;
        SSrc<X, Y, X, _> xyx;
        SSrc<X, Y, Y, _> xyy;
        SSrc<X, Y, Z, _> xyz;
        SSrc<X, Y, W, _> xyw;
        SSrc<X, Z, X, _> xzx;
        SSrc<X, Z, Y, _> xzy;
        SSrc<X, Z, Z, _> xzz;
        SSrc<X, Z, W, _> xzw;
        SSrc<X, W, X, _> xwx;
        SSrc<X, W, Y, _> xwy;
        SSrc<X, W, Z, _> xwz;
        SSrc<X, W, W, _> xww;

        SSrc<Y, X, X, _> yxx;
        SSrc<Y, X, Y, _> yxy;
        SSrc<Y, X, Z, _> yxz;
        SSrc<Y, X, W, _> yxw;
        SSrc<Y, Y, X, _> yyx;
        SSrc<Y, Y, Y, _> yyy;
        SSrc<Y, Y, Z, _> yyz;
        SSrc<Y, Y, W, _> yyw;
        SSrc<Y, Z, X, _> yzx;
        SSrc<Y, Z, Y, _> yzy;
        SSrc<Y, Z, Z, _> yzz;
        SSrc<Y, Z, W, _> yzw;
        SSrc<Y, W, X, _> ywx;
        SSrc<Y, W, Y, _> ywy;
        SSrc<Y, W, Z, _> ywz;
        SSrc<Y, W, W, _> yww;

        SSrc<Z, X, X, _> zxx;
        SSrc<Z, X, Y, _> zxy;
        SSrc<Z, X, Z, _> zxz;
        SSrc<Z, X, W, _> zxw;
        SSrc<Z, Y, X, _> zyx;
        SSrc<Z, Y, Y, _> zyy;
        SSrc<Z, Y, Z, _> zyz;
        SSrc<Z, Y, W, _> zyw;
        SSrc<Z, Z, X, _> zzx;
        SSrc<Z, Z, Y, _> zzy;
        SSrc<Z, Z, Z, _> zzz;
        SSrc<Z, Z, W, _> zzw;
        SSrc<Z, W, X, _> zwx;
        SSrc<Z, W, Y, _> zwy;
        SSrc<Z, W, Z, _> zwz;
        SSrc<Z, W, W, _> zww;

        SSrc<W, X, X, _> wxx;
        SSrc<W, X, Y, _> wxy;
        SSrc<W, X, Z, _> wxz;
        SSrc<W, X, W, _> wxw;
        SSrc<W, Y, X, _> wyx;
        SSrc<W, Y, Y, _> wyy;
        SSrc<W, Y, Z, _> wyz;
        SSrc<W, Y, W, _> wyw;
        SSrc<W, Z, X, _> wzx;
        SSrc<W, Z, Y, _> wzy;
        SSrc<W, Z, Z, _> wzz;
        SSrc<W, Z, W, _> wzw;
        SSrc<W, W, X, _> wwx;
        SSrc<W, W, Y, _> wwy;
        SSrc<W, W, Z, _> wwz;
        SSrc<W, W, W, _> www;

        SSrc<X, X, X, X> xxxx;
        SSrc<X, X, X, Y> xxxy;
        SSrc<X, X, X, Z> xxxz;
        SSrc<X, X, X, W> xxxw;
        SSrc<X, X, Y, X> xxyx;
        SSrc<X, X, Y, Y> xxyy;
        SSrc<X, X, Y, Z> xxyz;
        SSrc<X, X, Y, W> xxyw;
        SSrc<X, X, Z, X> xxzx;
        SSrc<X, X, Z, Y> xxzy;
        SSrc<X, X, Z, Z> xxzz;
        SSrc<X, X, Z, W> xxzw;
        SSrc<X, X, W, X> xxwx;
        SSrc<X, X, W, Y> xxwy;
        SSrc<X, X, W, Z> xxwz;
        SSrc<X, X, W, W> xxww;
        SSrc<X, Y, X, X> xyxx;
        SSrc<X, Y, X, Y> xyxy;
        SSrc<X, Y, X, Z> xyxz;
        SSrc<X, Y, X, W> xyxw;
        SSrc<X, Y, Y, X> xyyx;
        SSrc<X, Y, Y, Y> xyyy;
        SSrc<X, Y, Y, Z> xyyz;
        SSrc<X, Y, Y, W> xyyw;
        SSrc<X, Y, Z, X> xyzx;
        SSrc<X, Y, Z, Y> xyzy;
        SSrc<X, Y, Z, Z> xyzz;
        SSrc<X, Y, Z, W> xyzw;
        SSrc<X, Y, W, X> xywx;
        SSrc<X, Y, W, Y> xywy;
        SSrc<X, Y, W, Z> xywz;
        SSrc<X, Y, W, W> xyww;
        SSrc<X, Z, X, X> xzxx;
        SSrc<X, Z, X, Y> xzxy;
        SSrc<X, Z, X, Z> xzxz;
        SSrc<X, Z, X, W> xzxw;
        SSrc<X, Z, Y, X> xzyx;
        SSrc<X, Z, Y, Y> xzyy;
        SSrc<X, Z, Y, Z> xzyz;
        SSrc<X, Z, Y, W> xzyw;
        SSrc<X, Z, Z, X> xzzx;
        SSrc<X, Z, Z, Y> xzzy;
        SSrc<X, Z, Z, Z> xzzz;
        SSrc<X, Z, Z, W> xzzw;
        SSrc<X, Z, W, X> xzwx;
        SSrc<X, Z, W, Y> xzwy;
        SSrc<X, Z, W, Z> xzwz;
        SSrc<X, Z, W, W> xzww;
        SSrc<X, W, X, X> xwxx;
        SSrc<X, W, X, Y> xwxy;
        SSrc<X, W, X, Z> xwxz;
        SSrc<X, W, X, W> xwxw;
        SSrc<X, W, Y, X> xwyx;
        SSrc<X, W, Y, Y> xwyy;
        SSrc<X, W, Y, Z> xwyz;
        SSrc<X, W, Y, W> xwyw;
        SSrc<X, W, Z, X> xwzx;
        SSrc<X, W, Z, Y> xwzy;
        SSrc<X, W, Z, Z> xwzz;
        SSrc<X, W, Z, W> xwzw;
        SSrc<X, W, W, X> xwwx;
        SSrc<X, W, W, Y> xwwy;
        SSrc<X, W, W, Z> xwwz;
        SSrc<X, W, W, W> xwww;

        SSrc<Y, X, X, X> yxxx;
        SSrc<Y, X, X, Y> yxxy;
        SSrc<Y, X, X, Z> yxxz;
        SSrc<Y, X, X, W> yxxw;
        SSrc<Y, X, Y, X> yxyx;
        SSrc<Y, X, Y, Y> yxyy;
        SSrc<Y, X, Y, Z> yxyz;
        SSrc<Y, X, Y, W> yxyw;
        SSrc<Y, X, Z, X> yxzx;
        SSrc<Y, X, Z, Y> yxzy;
        SSrc<Y, X, Z, Z> yxzz;
        SSrc<Y, X, Z, W> yxzw;
        SSrc<Y, X, W, X> yxwx;
        SSrc<Y, X, W, Y> yxwy;
        SSrc<Y, X, W, Z> yxwz;
        SSrc<Y, X, W, W> yxww;
        SSrc<Y, Y, X, X> yyxx;
        SSrc<Y, Y, X, Y> yyxy;
        SSrc<Y, Y, X, Z> yyxz;
        SSrc<Y, Y, X, W> yyxw;
        SSrc<Y, Y, Y, X> yyyx;
        SSrc<Y, Y, Y, Y> yyyy;
        SSrc<Y, Y, Y, Z> yyyz;
        SSrc<Y, Y, Y, W> yyyw;
        SSrc<Y, Y, Z, X> yyzx;
        SSrc<Y, Y, Z, Y> yyzy;
        SSrc<Y, Y, Z, Z> yyzz;
        SSrc<Y, Y, Z, W> yyzw;
        SSrc<Y, Y, W, X> yywx;
        SSrc<Y, Y, W, Y> yywy;
        SSrc<Y, Y, W, Z> yywz;
        SSrc<Y, Y, W, W> yyww;
        SSrc<Y, Z, X, X> yzxx;
        SSrc<Y, Z, X, Y> yzxy;
        SSrc<Y, Z, X, Z> yzxz;
        SSrc<Y, Z, X, W> yzxw;
        SSrc<Y, Z, Y, X> yzyx;
        SSrc<Y, Z, Y, Y> yzyy;
        SSrc<Y, Z, Y, Z> yzyz;
        SSrc<Y, Z, Y, W> yzyw;
        SSrc<Y, Z, Z, X> yzzx;
        SSrc<Y, Z, Z, Y> yzzy;
        SSrc<Y, Z, Z, Z> yzzz;
        SSrc<Y, Z, Z, W> yzzw;
        SSrc<Y, Z, W, X> yzwx;
        SSrc<Y, Z, W, Y> yzwy;
        SSrc<Y, Z, W, Z> yzwz;
        SSrc<Y, Z, W, W> yzww;
        SSrc<Y, W, X, X> ywxx;
        SSrc<Y, W, X, Y> ywxy;
        SSrc<Y, W, X, Z> ywxz;
        SSrc<Y, W, X, W> ywxw;
        SSrc<Y, W, Y, X> ywyx;
        SSrc<Y, W, Y, Y> ywyy;
        SSrc<Y, W, Y, Z> ywyz;
        SSrc<Y, W, Y, W> ywyw;
        SSrc<Y, W, Z, X> ywzx;
        SSrc<Y, W, Z, Y> ywzy;
        SSrc<Y, W, Z, Z> ywzz;
        SSrc<Y, W, Z, W> ywzw;
        SSrc<Y, W, W, X> ywwx;
        SSrc<Y, W, W, Y> ywwy;
        SSrc<Y, W, W, Z> ywwz;
        SSrc<Y, W, W, W> ywww;

        SSrc<Z, X, X, X> zxxx;
        SSrc<Z, X, X, Y> zxxy;
        SSrc<Z, X, X, Z> zxxz;
        SSrc<Z, X, X, W> zxxw;
        SSrc<Z, X, Y, X> zxyx;
        SSrc<Z, X, Y, Y> zxyy;
        SSrc<Z, X, Y, Z> zxyz;
        SSrc<Z, X, Y, W> zxyw;
        SSrc<Z, X, Z, X> zxzx;
        SSrc<Z, X, Z, Y> zxzy;
        SSrc<Z, X, Z, Z> zxzz;
        SSrc<Z, X, Z, W> zxzw;
        SSrc<Z, X, W, X> zxwx;
        SSrc<Z, X, W, Y> zxwy;
        SSrc<Z, X, W, Z> zxwz;
        SSrc<Z, X, W, W> zxww;
        SSrc<Z, Y, X, X> zyxx;
        SSrc<Z, Y, X, Y> zyxy;
        SSrc<Z, Y, X, Z> zyxz;
        SSrc<Z, Y, X, W> zyxw;
        SSrc<Z, Y, Y, X> zyyx;
        SSrc<Z, Y, Y, Y> zyyy;
        SSrc<Z, Y, Y, Z> zyyz;
        SSrc<Z, Y, Y, W> zyyw;
        SSrc<Z, Y, Z, X> zyzx;
        SSrc<Z, Y, Z, Y> zyzy;
        SSrc<Z, Y, Z, Z> zyzz;
        SSrc<Z, Y, Z, W> zyzw;
        SSrc<Z, Y, W, X> zywx;
        SSrc<Z, Y, W, Y> zywy;
        SSrc<Z, Y, W, Z> zywz;
        SSrc<Z, Y, W, W> zyww;
        SSrc<Z, Z, X, X> zzxx;
        SSrc<Z, Z, X, Y> zzxy;
        SSrc<Z, Z, X, Z> zzxz;
        SSrc<Z, Z, X, W> zzxw;
        SSrc<Z, Z, Y, X> zzyx;
        SSrc<Z, Z, Y, Y> zzyy;
        SSrc<Z, Z, Y, Z> zzyz;
        SSrc<Z, Z, Y, W> zzyw;
        SSrc<Z, Z, Z, X> zzzx;
        SSrc<Z, Z, Z, Y> zzzy;
        SSrc<Z, Z, Z, Z> zzzz;
        SSrc<Z, Z, Z, W> zzzw;
        SSrc<Z, Z, W, X> zzwx;
        SSrc<Z, Z, W, Y> zzwy;
        SSrc<Z, Z, W, Z> zzwz;
        SSrc<Z, Z, W, W> zzww;
        SSrc<Z, W, X, X> zwxx;
        SSrc<Z, W, X, Y> zwxy;
        SSrc<Z, W, X, Z> zwxz;
        SSrc<Z, W, X, W> zwxw;
        SSrc<Z, W, Y, X> zwyx;
        SSrc<Z, W, Y, Y> zwyy;
        SSrc<Z, W, Y, Z> zwyz;
        SSrc<Z, W, Y, W> zwyw;
        SSrc<Z, W, Z, X> zwzx;
        SSrc<Z, W, Z, Y> zwzy;
        SSrc<Z, W, Z, Z> zwzz;
        SSrc<Z, W, Z, W> zwzw;
        SSrc<Z, W, W, X> zwwx;
        SSrc<Z, W, W, Y> zwwy;
        SSrc<Z, W, W, Z> zwwz;
        SSrc<Z, W, W, W> zwww;

        SSrc<W, X, X, X> wxxx;
        SSrc<W, X, X, Y> wxxy;
        SSrc<W, X, X, Z> wxxz;
        SSrc<W, X, X, W> wxxw;
        SSrc<W, X, Y, X> wxyx;
        SSrc<W, X, Y, Y> wxyy;
        SSrc<W, X, Y, Z> wxyz;
        SSrc<W, X, Y, W> wxyw;
        SSrc<W, X, Z, X> wxzx;
        SSrc<W, X, Z, Y> wxzy;
        SSrc<W, X, Z, Z> wxzz;
        SSrc<W, X, Z, W> wxzw;
        SSrc<W, X, W, X> wxwx;
        SSrc<W, X, W, Y> wxwy;
        SSrc<W, X, W, Z> wxwz;
        SSrc<W, X, W, W> wxww;
        SSrc<W, Y, X, X> wyxx;
        SSrc<W, Y, X, Y> wyxy;
        SSrc<W, Y, X, Z> wyxz;
        SSrc<W, Y, X, W> wyxw;
        SSrc<W, Y, Y, X> wyyx;
        SSrc<W, Y, Y, Y> wyyy;
        SSrc<W, Y, Y, Z> wyyz;
        SSrc<W, Y, Y, W> wyyw;
        SSrc<W, Y, Z, X> wyzx;
        SSrc<W, Y, Z, Y> wyzy;
        SSrc<W, Y, Z, Z> wyzz;
        SSrc<W, Y, Z, W> wyzw;
        SSrc<W, Y, W, X> wywx;
        SSrc<W, Y, W, Y> wywy;
        SSrc<W, Y, W, Z> wywz;
        SSrc<W, Y, W, W> wyww;
        SSrc<W, Z, X, X> wzxx;
        SSrc<W, Z, X, Y> wzxy;
        SSrc<W, Z, X, Z> wzxz;
        SSrc<W, Z, X, W> wzxw;
        SSrc<W, Z, Y, X> wzyx;
        SSrc<W, Z, Y, Y> wzyy;
        SSrc<W, Z, Y, Z> wzyz;
        SSrc<W, Z, Y, W> wzyw;
        SSrc<W, Z, Z, X> wzzx;
        SSrc<W, Z, Z, Y> wzzy;
        SSrc<W, Z, Z, Z> wzzz;
        SSrc<W, Z, Z, W> wzzw;
        SSrc<W, Z, W, X> wzwx;
        SSrc<W, Z, W, Y> wzwy;
        SSrc<W, Z, W, Z> wzwz;
        SSrc<W, Z, W, W> wzww;
        SSrc<W, W, X, X> wwxx;
        SSrc<W, W, X, Y> wwxy;
        SSrc<W, W, X, Z> wwxz;
        SSrc<W, W, X, W> wwxw;
        SSrc<W, W, Y, X> wwyx;
        SSrc<W, W, Y, Y> wwyy;
        SSrc<W, W, Y, Z> wwyz;
        SSrc<W, W, Y, W> wwyw;
        SSrc<W, W, Z, X> wwzx;
        SSrc<W, W, Z, Y> wwzy;
        SSrc<W, W, Z, Z> wwzz;
        SSrc<W, W, Z, W> wwzw;
        SSrc<W, W, W, X> wwwx;
        SSrc<W, W, W, Y> wwwy;
        SSrc<W, W, W, Z> wwwz;
        SSrc<W, W, W, W> wwww;

        #undef _

        Self &operator-() {
            base.negate = !base.negate;
            return *this;
        }

        Self &operator[](Addr addr) {
            base.addr = addr;
            return *this;
        }

        operator RawSrc() const {
            return xyzw;
        }
    };

    #define THIS_BUILDER reinterpret_cast<ShaderBuilder*>(reinterpret_cast<intptr_t>(this)-offsetof(ShaderBuilder, anchor));

    template <OpCode::Id Op>
    struct Format0 {
        void operator()() {
            auto builder = THIS_BUILDER;

            auto &instr = builder->PushInstruction();
            instr.opcode = Op;
        }
    };

    template <OpCode::Id Op>
    struct Format1 {
        template <typename Src1, typename Src2>
        void operator()(Dest dest, Src1 src1, Src2 src2) {
            auto builder = THIS_BUILDER;
            RawSrc src1Raw = src1;
            RawSrc src2Raw = src2;

            SwizzlePattern pattern;
            pattern.src1_selector_0 = static_cast<SwizzlePattern::Selector>(src1Raw.selector[0]);
            pattern.src1_selector_1 = static_cast<SwizzlePattern::Selector>(src1Raw.selector[1]);
            pattern.src1_selector_2 = static_cast<SwizzlePattern::Selector>(src1Raw.selector[2]);
            pattern.src1_selector_3 = static_cast<SwizzlePattern::Selector>(src1Raw.selector[3]);

            pattern.src2_selector_0 = static_cast<SwizzlePattern::Selector>(src2Raw.selector[0]);
            pattern.src2_selector_1 = static_cast<SwizzlePattern::Selector>(src2Raw.selector[1]);
            pattern.src2_selector_2 = static_cast<SwizzlePattern::Selector>(src2Raw.selector[2]);
            pattern.src2_selector_3 = static_cast<SwizzlePattern::Selector>(src2Raw.selector[3]);

            pattern.dest_mask = dest.mask;

            auto &instr = builder->PushInstruction();
            instr.opcode = Op;
            instr.common.operand_desc_id = builder->PushSwizzle(pattern);
            instr.common.src1 = src1Raw.ToSourceRegister();
            instr.common.src2 = src2Raw.ToSourceRegister();
            instr.common.address_register_index = static_cast<uint32_t>(src1Raw.addr);
            instr.common.dest = dest.ToDestRegister();
        }
    };

    template <OpCode::Id Op>
    struct Format2 {
        void operator()(Label start, Label end = Label::Invalid(), Cond cond = Cond(true, true, Cond::Op::Or)) {
            auto builder = THIS_BUILDER;

            builder->UseLabels(start, end);
            auto &instr = builder->PushInstruction();
            instr.opcode = Op;
            cond.Apply(instr);
        }
    };

    union {
        void *anchor;
        // TODO: Format1u
        Format1<OpCode::Id::MOV> MOV;
        Format2<OpCode::Id::CALL> CALL;
        Format2<OpCode::Id::CALLC> CALLC;
        Format2<OpCode::Id::IFC> IFC;
        Format0<OpCode::Id::END> END;
    };

    void LinkLabels() {
        for (auto &link : link_list) {
            auto &instr = (*program)[link.pc];

            auto start = instr.flow_control.dest_offset = labels[link.start.index].pc;

            if (link.end.IsValid()) {
                instr.flow_control.num_instructions = labels[link.end.index].pc - start;
            }
        }
    }

    CompiledShader Compile() {
        LinkLabels();
        return CompiledShader(
            std::move(program),
            std::move(swizzle_data)
        );
    }
private:
    u32 PushSwizzle(SwizzlePattern p) {
        (*swizzle_data)[sidx] = p;
        return sidx++;
    }

    Instruction &PushInstruction() {
        return (*program)[pc++];
    }

    void UseLabels(Label start, Label end = Label::Invalid()) {
        link_list.emplace_back(NeedsLink(pc, start, end));
    }

    struct LabelData {
        Label label;
        u32 pc = 0xFFFFFFFF;

        constexpr LabelData(Label label) : label(label) {}
    };

    struct NeedsLink {
        u32 pc;
        Label start;
        Label end;

        constexpr NeedsLink(u32 pc, Label start, Label end = Label::Invalid()) : pc(pc), start(start), end(end) {}
    };

    u32 pc = 0;
    u32 sidx = 0;
    std::unique_ptr<ProgramArray> program;
    std::unique_ptr<SwizzleArray> swizzle_data;
    std::vector<LabelData> labels;
    std::vector<NeedsLink> link_list;
};

static std::unique_ptr<CachedBatch> CompileShader(ShaderSetup& setup, UnitState& state, CompiledShader &&code) {
    std::transform(code.program->begin(), code.program->end(), setup.program_code.begin(),
                   [](const auto& x) { return x.hex; });
    std::transform(code.swizzle_data->begin(), code.swizzle_data->end(), setup.swizzle_data.begin(),
                   [](const auto& x) { return x.hex; });

    auto shader = std::make_unique<CachedBatch>(0);
    shader->Compile(setup, state);

    return shader;
}

class ShaderTest {
public:
    explicit ShaderTest(CompiledShader &&code)
        : shader(CompileShader(setup, state, std::move(code))) {}

    void Run() {
        shader->Run(setup, state, 0);
    }

    Vec4f &v(unsigned int x) {
        return *reinterpret_cast<Vec4f*>(&state.registers.input[x]);
    }

    Vec4f &o(unsigned int x) {
        return *reinterpret_cast<Vec4f*>(&state.registers.output[x]);
    }

    Vec4f &c(unsigned int x) {
        return *reinterpret_cast<Vec4f*>(&setup.uniforms.f[x]);
    }

    bool &cmpx() {
        return state.conditional_code[0];
    }

public:
    std::unique_ptr<CachedBatch> shader;

    Pica::Shader::ShaderSetup setup;
    Pica::Shader::UnitState state;
};

TEST_CASE("MOV", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    builder.MOV(o0, c0, c0);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).x);
    CHECK(shader.o(0).y == shader.c(0).y);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);
}

TEST_CASE("MOV MASK", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    builder.MOV(o0.yw(), c0, c0);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.Run();

    CHECK(shader.o(0).x == 0.0);
    CHECK(shader.o(0).y == shader.c(0).y);
    CHECK(shader.o(0).z == 0.0);
    CHECK(shader.o(0).w == shader.c(0).w);
}

TEST_CASE("MOV SWIZZLE", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    builder.MOV(o0, c0.ywxz, c0);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).y);
    CHECK(shader.o(0).y == shader.c(0).w);
    CHECK(shader.o(0).z == shader.c(0).x);
    CHECK(shader.o(0).w == shader.c(0).z);
}

TEST_CASE("CALL", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    auto FuncStart = builder.NewLabel();
    auto FuncEnd = builder.NewLabel();

    builder.MOV(o0, c0, c0);
    builder.CALL(FuncStart, FuncEnd);
    builder.MOV(o0.y(), c0.wwww, c0);
    builder.END();
    builder.EmitLabel(FuncStart);
    builder.MOV(o0.x(), c0.yyyy, c0);
    builder.EmitLabel(FuncEnd);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).y);
    CHECK(shader.o(0).y == shader.c(0).w);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);
}

TEST_CASE("CALLC", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    auto FuncStart = builder.NewLabel();
    auto FuncEnd = builder.NewLabel();

    builder.MOV(o0, c0, c0);
    builder.CALLC(FuncStart, FuncEnd, Cond::X(false));
    builder.MOV(o0.y(), c0.wwww, c0);
    builder.END();
    builder.EmitLabel(FuncStart);
    builder.MOV(o0.x(), c0.yyyy, c0);
    builder.EmitLabel(FuncEnd);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.cmpx() = true;
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).y);
    CHECK(shader.o(0).y == shader.c(0).w);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.cmpx() = false;
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).x);
    CHECK(shader.o(0).y == shader.c(0).w);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);
}

TEST_CASE("IFC", "[video_core][shader][shader_fast]") {
    const auto c0 = ShaderBuilder::Src(0x20);
    const auto o0 = ShaderBuilder::Dest(0);

    ShaderBuilder builder;

    auto FuncStart = builder.NewLabel();
    auto FuncEnd = builder.NewLabel();

    builder.MOV(o0, c0, c0);
    builder.IFC(FuncStart, FuncEnd, Cond::X(false));
    builder.MOV(o0.y(), c0.wwww, c0);
    builder.EmitLabel(FuncStart);
    builder.MOV(o0.x(), c0.yyyy, c0);
    builder.EmitLabel(FuncEnd);
    builder.END();

    auto shader = ShaderTest(builder.Compile());

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.cmpx() = true;
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).x);
    CHECK(shader.o(0).y == shader.c(0).w);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);

    shader.c(0) = Vec4f(0.0, 1.0, 2.0, 3.0);
    shader.o(0) = Vec4f();
    shader.cmpx() = false;
    shader.Run();

    CHECK(shader.o(0).x == shader.c(0).y);
    CHECK(shader.o(0).y == shader.c(0).y);
    CHECK(shader.o(0).z == shader.c(0).z);
    CHECK(shader.o(0).w == shader.c(0).w);
}
