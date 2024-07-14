/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8 */

#ifndef PB_MBED_RPC_MBED_RPC_PB_H_INCLUDED
#define PB_MBED_RPC_MBED_RPC_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _mbed_rpc_ErrorCode {
    mbed_rpc_ErrorCode_NO_ERROR = 0
} mbed_rpc_ErrorCode;

typedef enum _mbed_rpc_BuiltinServices {
    mbed_rpc_BuiltinServices_SVC_PING = 0
} mbed_rpc_BuiltinServices;

typedef enum _mbed_rpc_BuiltinMessages {
    mbed_rpc_BuiltinMessages_MSG_PING = 0
} mbed_rpc_BuiltinMessages;

/* Struct definitions */
/* Core message header common to all types. Each functional message type **must**
 have this first in their list of declarations. */
typedef struct _mbed_rpc_Header {
    uint16_t crc; /* CRC16 of the message for validity checks */
    uint16_t size; /* Size of the message in bytes */
    uint8_t version; /* Version of this message & RPC protocol. Upper 4 bits are RPC, lower 4 bits are message. */
    uint8_t seqId; /* Sequence ID for the message transaction */
    uint16_t msgId; /* Root message identifier */
} mbed_rpc_Header;

/* Root type that parsers can use to peek at messages and figure out what type the full message is. */
typedef struct _mbed_rpc_BaseMessage {
    mbed_rpc_Header header;
} mbed_rpc_BaseMessage;

/* Simple ping message to test RPC connection. */
typedef struct _mbed_rpc_Ping {
    mbed_rpc_Header header;
    uint32_t timestamp;
} mbed_rpc_Ping;

/* Message to request a list of all available RPC functions. */
typedef struct _mbed_rpc_ListFunctionsRequest {
    mbed_rpc_Header header;
} mbed_rpc_ListFunctionsRequest;

/* Message to respond to a ListFunctionsRequest. */
typedef struct _mbed_rpc_ListFunctionsResponse {
    mbed_rpc_Header header;
    pb_callback_t functions;
} mbed_rpc_ListFunctionsResponse;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _mbed_rpc_ErrorCode_MIN mbed_rpc_ErrorCode_NO_ERROR
#define _mbed_rpc_ErrorCode_MAX mbed_rpc_ErrorCode_NO_ERROR
#define _mbed_rpc_ErrorCode_ARRAYSIZE ((mbed_rpc_ErrorCode)(mbed_rpc_ErrorCode_NO_ERROR+1))

#define _mbed_rpc_BuiltinServices_MIN mbed_rpc_BuiltinServices_SVC_PING
#define _mbed_rpc_BuiltinServices_MAX mbed_rpc_BuiltinServices_SVC_PING
#define _mbed_rpc_BuiltinServices_ARRAYSIZE ((mbed_rpc_BuiltinServices)(mbed_rpc_BuiltinServices_SVC_PING+1))

#define _mbed_rpc_BuiltinMessages_MIN mbed_rpc_BuiltinMessages_MSG_PING
#define _mbed_rpc_BuiltinMessages_MAX mbed_rpc_BuiltinMessages_MSG_PING
#define _mbed_rpc_BuiltinMessages_ARRAYSIZE ((mbed_rpc_BuiltinMessages)(mbed_rpc_BuiltinMessages_MSG_PING+1))







/* Initializer values for message structs */
#define mbed_rpc_Header_init_default             {0, 0, 0, 0, 0}
#define mbed_rpc_BaseMessage_init_default        {mbed_rpc_Header_init_default}
#define mbed_rpc_Ping_init_default               {mbed_rpc_Header_init_default, 0}
#define mbed_rpc_ListFunctionsRequest_init_default {mbed_rpc_Header_init_default}
#define mbed_rpc_ListFunctionsResponse_init_default {mbed_rpc_Header_init_default, {{NULL}, NULL}}
#define mbed_rpc_Header_init_zero                {0, 0, 0, 0, 0}
#define mbed_rpc_BaseMessage_init_zero           {mbed_rpc_Header_init_zero}
#define mbed_rpc_Ping_init_zero                  {mbed_rpc_Header_init_zero, 0}
#define mbed_rpc_ListFunctionsRequest_init_zero  {mbed_rpc_Header_init_zero}
#define mbed_rpc_ListFunctionsResponse_init_zero {mbed_rpc_Header_init_zero, {{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define mbed_rpc_Header_crc_tag                  1
#define mbed_rpc_Header_size_tag                 2
#define mbed_rpc_Header_version_tag              3
#define mbed_rpc_Header_seqId_tag                4
#define mbed_rpc_Header_msgId_tag                5
#define mbed_rpc_BaseMessage_header_tag          1
#define mbed_rpc_Ping_header_tag                 1
#define mbed_rpc_Ping_timestamp_tag              2
#define mbed_rpc_ListFunctionsRequest_header_tag 1
#define mbed_rpc_ListFunctionsResponse_header_tag 1
#define mbed_rpc_ListFunctionsResponse_functions_tag 2

/* Struct field encoding specification for nanopb */
#define mbed_rpc_Header_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   crc,               1) \
X(a, STATIC,   REQUIRED, UINT32,   size,              2) \
X(a, STATIC,   REQUIRED, UINT32,   version,           3) \
X(a, STATIC,   REQUIRED, UINT32,   seqId,             4) \
X(a, STATIC,   REQUIRED, UINT32,   msgId,             5)
#define mbed_rpc_Header_CALLBACK NULL
#define mbed_rpc_Header_DEFAULT NULL

#define mbed_rpc_BaseMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define mbed_rpc_BaseMessage_CALLBACK NULL
#define mbed_rpc_BaseMessage_DEFAULT NULL
#define mbed_rpc_BaseMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_Ping_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   timestamp,         2)
#define mbed_rpc_Ping_CALLBACK NULL
#define mbed_rpc_Ping_DEFAULT NULL
#define mbed_rpc_Ping_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_ListFunctionsRequest_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define mbed_rpc_ListFunctionsRequest_CALLBACK NULL
#define mbed_rpc_ListFunctionsRequest_DEFAULT NULL
#define mbed_rpc_ListFunctionsRequest_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_ListFunctionsResponse_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, CALLBACK, REPEATED, STRING,   functions,         2)
#define mbed_rpc_ListFunctionsResponse_CALLBACK pb_default_field_callback
#define mbed_rpc_ListFunctionsResponse_DEFAULT NULL
#define mbed_rpc_ListFunctionsResponse_header_MSGTYPE mbed_rpc_Header

extern const pb_msgdesc_t mbed_rpc_Header_msg;
extern const pb_msgdesc_t mbed_rpc_BaseMessage_msg;
extern const pb_msgdesc_t mbed_rpc_Ping_msg;
extern const pb_msgdesc_t mbed_rpc_ListFunctionsRequest_msg;
extern const pb_msgdesc_t mbed_rpc_ListFunctionsResponse_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define mbed_rpc_Header_fields &mbed_rpc_Header_msg
#define mbed_rpc_BaseMessage_fields &mbed_rpc_BaseMessage_msg
#define mbed_rpc_Ping_fields &mbed_rpc_Ping_msg
#define mbed_rpc_ListFunctionsRequest_fields &mbed_rpc_ListFunctionsRequest_msg
#define mbed_rpc_ListFunctionsResponse_fields &mbed_rpc_ListFunctionsResponse_msg

/* Maximum encoded size of messages (where known) */
/* mbed_rpc_ListFunctionsResponse_size depends on runtime parameters */
#define MBED_RPC_MBED_RPC_PB_H_MAX_SIZE          mbed_rpc_Ping_size
#define mbed_rpc_BaseMessage_size                20
#define mbed_rpc_Header_size                     18
#define mbed_rpc_ListFunctionsRequest_size       20
#define mbed_rpc_Ping_size                       26

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* Message descriptors for nanopb */
namespace nanopb {
template <>
struct MessageDescriptor<mbed_rpc_Header> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 5;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_Header_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_BaseMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_BaseMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_Ping> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_Ping_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_ListFunctionsRequest> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_ListFunctionsRequest_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_ListFunctionsResponse> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_ListFunctionsResponse_msg;
    }
};
}  // namespace nanopb

#endif  /* __cplusplus */


#endif