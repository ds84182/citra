#include <memory>
#include <boost/range/algorithm/fill.hpp>
#include "common/alignment.h"
#include "common/assert.h"
#include "common/bit_field.h"
#include "common/common_types.h"
#include "common/logging/log.h"
#include "common/vector_math.h"
#include "core/memory.h"
#include "video_core/debug_utils/debug_utils.h"
#include "video_core/pica_state.h"
#include "video_core/pica_types.h"
#include "video_core/regs_pipeline.h"
#include "video_core/shader/shader.h"
#include "video_core/vertex_loader.h"

namespace Pica {

void VertexLoader::Setup(const PipelineRegs& regs) {
    ASSERT_MSG(!is_setup, "VertexLoader is not intended to be setup more than once.");

    const auto& attribute_config = regs.vertex_attributes;
    num_total_attributes = attribute_config.GetNumTotalAttributes();

    boost::fill(vertex_attribute_sources, 0xdeadbeef);

    for (int i = 0; i < 16; i++) {
        vertex_attribute_is_default[i] = attribute_config.IsDefaultAttribute(i);
    }

    // Setup attribute data from loaders
    for (int loader = 0; loader < 12; ++loader) {
        const auto& loader_config = attribute_config.attribute_loaders[loader];

        u32 offset = 0;

        // TODO: What happens if a loader overwrites a previous one's data?
        for (unsigned component = 0; component < loader_config.component_count; ++component) {
            if (component >= 12) {
                LOG_ERROR(HW_GPU,
                          "Overflow in the vertex attribute loader %u trying to load component %u",
                          loader, component);
                continue;
            }

            u32 attribute_index = loader_config.GetComponent(component);
            if (attribute_index < 12) {
                offset = Common::AlignUp(offset,
                                         attribute_config.GetElementSizeInBytes(attribute_index));
                vertex_attribute_sources[attribute_index] = loader_config.data_offset + offset;
                vertex_attribute_strides[attribute_index] =
                    static_cast<u32>(loader_config.byte_count);
                vertex_attribute_formats[attribute_index] =
                    attribute_config.GetFormat(attribute_index);
                vertex_attribute_elements[attribute_index] =
                    attribute_config.GetNumElements(attribute_index);
                offset += attribute_config.GetStride(attribute_index);
            } else if (attribute_index < 16) {
                // Attribute ids 12, 13, 14 and 15 signify 4, 8, 12 and 16-byte paddings,
                // respectively
                offset = Common::AlignUp(offset, 4);
                offset += (attribute_index - 11) * 4;
            } else {
                UNREACHABLE(); // This is truly unreachable due to the number of bits for each
                               // component
            }
        }
    }

    is_setup = true;
}

static void FillDefaultValues(float output[4], unsigned int count) {
    // Default attribute values set if array elements have < 4 components. This
    // is *not* carried over from the default attribute settings even if they're
    // enabled for this attribute.

    // This switch prevents major compilers (MSVC and Clang) from generating a
    // bunch of complicated SSE instructions for a simple loop.

    switch (count) {
    case 0:
        output[0] = 0.0f;
    case 1:
        output[1] = 0.0f;
    case 2:
        output[2] = 0.0f;
    case 3:
        output[3] = 1.0f;
    }
}

template <typename T>
static void LoadAttribute(const T * __restrict input, float * __restrict output, unsigned int count) {
    for (unsigned int i = 0; i<4; i++) {
        output[i] = static_cast<float>(input[i]);
    }
    FillDefaultValues(output, count);
}

void VertexLoader::LoadVertex(u32 base_address, int index, int vertex,
                              Shader::AttributeBuffer& input,
                              DebugUtils::MemoryAccessTracker& memory_accesses) {
    ASSERT_MSG(is_setup, "A VertexLoader needs to be setup before loading vertices.");

    for (int i = 0; i < num_total_attributes; ++i) {
        if (vertex_attribute_elements[i] != 0) {
            // Load per-vertex data from the loader arrays
            u32 source_addr =
                base_address + vertex_attribute_sources[i] + vertex_attribute_strides[i] * vertex;

            if (g_debug_context && Pica::g_debug_context->recorder) {
                memory_accesses.AddAccess(
                    source_addr,
                    vertex_attribute_elements[i] *
                        ((vertex_attribute_formats[i] == PipelineRegs::VertexAttributeFormat::FLOAT)
                             ? 4
                             : (vertex_attribute_formats[i] ==
                                PipelineRegs::VertexAttributeFormat::SHORT)
                                   ? 2
                                   : 1));
            }

            switch (vertex_attribute_formats[i]) {
            case PipelineRegs::VertexAttributeFormat::BYTE: {
                const s8* srcdata =
                    reinterpret_cast<const s8*>(Memory::GetPhysicalPointer(source_addr));
                LoadAttribute<s8>(srcdata, reinterpret_cast<float*>(&input.attr[i]), vertex_attribute_elements[i]);
                break;
            }
            case PipelineRegs::VertexAttributeFormat::UBYTE: {
                const u8* srcdata =
                    reinterpret_cast<const u8*>(Memory::GetPhysicalPointer(source_addr));
                LoadAttribute<u8>(srcdata, reinterpret_cast<float*>(&input.attr[i]), vertex_attribute_elements[i]);
                break;
            }
            case PipelineRegs::VertexAttributeFormat::SHORT: {
                const s16* srcdata =
                    reinterpret_cast<const s16*>(Memory::GetPhysicalPointer(source_addr));
                LoadAttribute<s16>(srcdata, reinterpret_cast<float*>(&input.attr[i]), vertex_attribute_elements[i]);
                break;
            }
            case PipelineRegs::VertexAttributeFormat::FLOAT: {
                const float* srcdata =
                    reinterpret_cast<const float*>(Memory::GetPhysicalPointer(source_addr));
                LoadAttribute<float>(srcdata, reinterpret_cast<float*>(&input.attr[i]), vertex_attribute_elements[i]);
                break;
            }
            }

            LOG_TRACE(HW_GPU, "Loaded %d components of attribute %x for vertex %x (index %x) from "
                              "0x%08x + 0x%08x + 0x%04x: %f %f %f %f",
                      vertex_attribute_elements[i], i, vertex, index, base_address,
                      vertex_attribute_sources[i], vertex_attribute_strides[i] * vertex,
                      input.attr[i][0].ToFloat32(), input.attr[i][1].ToFloat32(),
                      input.attr[i][2].ToFloat32(), input.attr[i][3].ToFloat32());
        } else if (vertex_attribute_is_default[i]) {
            // Load the default attribute if we're configured to do so
            input.attr[i] = g_state.input_default_attributes.attr[i];
            LOG_TRACE(HW_GPU,
                      "Loaded default attribute %x for vertex %x (index %x): (%f, %f, %f, %f)", i,
                      vertex, index, input.attr[i][0].ToFloat32(), input.attr[i][1].ToFloat32(),
                      input.attr[i][2].ToFloat32(), input.attr[i][3].ToFloat32());
        } else {
            // TODO(yuriks): In this case, no data gets loaded and the vertex
            // remains with the last value it had. This isn't currently maintained
            // as global state, however, and so won't work in Citra yet.
        }
    }
}

} // namespace Pica
