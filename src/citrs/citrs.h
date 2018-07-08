#pragma once

#include <cstdint>
#include <cstdlib>

namespace citrs {

static const uintptr_t SOC_COUNT = 64;

struct HLEResumeToken;

struct SOCUContext;

extern "C" {

extern void citra_socu_unpark(HLEResumeToken *token);

void citrs_socu_accept_async(SOCUContext *ctx,
                             int32_t sock,
                             HLEResumeToken *token,
                             uintptr_t result);

int32_t citrs_socu_bind(SOCUContext *ctx, int32_t sock, uint32_t address, uint16_t port);

int32_t citrs_socu_close(SOCUContext *ctx, int32_t sock);

int32_t citrs_socu_consume(SOCUContext *ctx, uintptr_t result);

int32_t citrs_socu_consume_accept(SOCUContext *ctx,
                                  uintptr_t result,
                                  uint8_t (*ipv4)[4],
                                  uint16_t *port);

int32_t citrs_socu_consume_read(SOCUContext *ctx, uintptr_t result, uintptr_t *read_amount);

void citrs_socu_exit(SOCUContext *ctx);

int32_t citrs_socu_get_non_blocking(SOCUContext *ctx, int32_t sock, bool *non_blocking);

SOCUContext *citrs_socu_init();

int32_t citrs_socu_listen(SOCUContext *ctx, int32_t sock);

uintptr_t citrs_socu_prepare_async_result(SOCUContext *ctx);

void citrs_socu_recv(SOCUContext *ctx,
                     int32_t sock,
                     HLEResumeToken *token,
                     uintptr_t result,
                     uint8_t *output_ptr,
                     uintptr_t output_len);

void citrs_socu_send(SOCUContext *ctx,
                     int32_t sock,
                     HLEResumeToken *token,
                     uintptr_t result,
                     const uint8_t *input_ptr,
                     uintptr_t input_len);

int32_t citrs_socu_set_non_blocking(SOCUContext *ctx, int32_t sock, bool non_blocking);

int32_t citrs_socu_socket(SOCUContext *ctx);

bool citrs_socu_try_accept(SOCUContext *ctx,
                           int32_t sock,
                           int32_t *posix_result,
                           uint8_t (*ipv4)[4],
                           uint16_t *port);

bool citrs_socu_try_recv(SOCUContext *ctx,
                         int32_t sock,
                         uint8_t *output_ptr,
                         uintptr_t output_len,
                         int32_t *posix_result,
                         uintptr_t *read_amount);

} // extern "C"

} // namespace citrs
