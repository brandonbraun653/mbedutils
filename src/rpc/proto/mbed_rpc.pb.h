/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8 */

#ifndef PB_MBED_RPC_MBED_RPC_PB_H_INCLUDED
#define PB_MBED_RPC_MBED_RPC_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _mbed_rpc_ProtocolVersion {
    mbed_rpc_ProtocolVersion_RPC_PROTOCOL_VER = 1
} mbed_rpc_ProtocolVersion;

typedef enum _mbed_rpc_ErrorCode {
    mbed_rpc_ErrorCode_ERR_NO_ERROR = 0, /* No error occurred */
    mbed_rpc_ErrorCode_ERR_SVC_ASYNC = 1, /* Used to indicate that the service is processing the request asynchronously without error so far */
    mbed_rpc_ErrorCode_ERR_SVC_BUSY = 2, /* Service is busy and cannot process the request */
    mbed_rpc_ErrorCode_ERR_RPC_VER = 3, /* RPC protocol version mismatch */
    mbed_rpc_ErrorCode_ERR_MSG_VER = 4, /* Message version mismatch */
    mbed_rpc_ErrorCode_ERR_MSG_SIZE = 5, /* Message size is incorrect */
    mbed_rpc_ErrorCode_ERR_MSG_CRC = 6, /* Message CRC is incorrect */
    mbed_rpc_ErrorCode_ERR_MSG_DECODE = 7, /* Message failed to decode */
    mbed_rpc_ErrorCode_ERR_SVC_NOT_FOUND = 8, /* Service not found */
    mbed_rpc_ErrorCode_ERR_MSG_NOT_FOUND = 9, /* Message not found */
    mbed_rpc_ErrorCode_ERR_SVC_MSG = 10, /* Service does not support the message */
    mbed_rpc_ErrorCode_ERR_SVC_FAILED = 11, /* Service failed to process the message */
    mbed_rpc_ErrorCode_ERR_SVC_NO_RSP = 12, /* Service has no response to the message */
    mbed_rpc_ErrorCode_ERR_MAX_ERROR = 255 /* Maximum error value */
} mbed_rpc_ErrorCode;

typedef enum _mbed_rpc_BuiltinService {
    mbed_rpc_BuiltinService_SVC_PING = 0, /* Accepts ping requests from clients */
    mbed_rpc_BuiltinService_SVC_TEST_ERROR = 1, /* Test error handling */
    mbed_rpc_BuiltinService_SVC_NOTIFY_TIME_ELAPSED = 2 /* Notify caller of elapsed system time */
} mbed_rpc_BuiltinService;

typedef enum _mbed_rpc_BuiltinMessage {
    mbed_rpc_BuiltinMessage_MSG_NULL = 0,
    mbed_rpc_BuiltinMessage_MSG_ERROR = 1,
    mbed_rpc_BuiltinMessage_MSG_PING = 2,
    mbed_rpc_BuiltinMessage_MSG_ACK_NACK = 3,
    mbed_rpc_BuiltinMessage_MSG_TICK = 4,
    mbed_rpc_BuiltinMessage_MSG_CONSOLE = 5,
    mbed_rpc_BuiltinMessage_MSG_SYSTEM_INFO = 6,
    mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_REQ = 7,
    mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_RSP = 8
} mbed_rpc_BuiltinMessage;

typedef enum _mbed_rpc_BuiltinMessageVersion {
    mbed_rpc_BuiltinMessageVersion_MSG_VER_NULL = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_ERROR = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_PING = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_ACK_NACK = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_TICK = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_CONSOLE = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_SYSTEM_INFO = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_NOTIFY_TIME_ELAPSED_REQ = 0,
    mbed_rpc_BuiltinMessageVersion_MSG_VER_NOTIFY_TIME_ELAPSED_RSP = 0
} mbed_rpc_BuiltinMessageVersion;

/* Struct definitions */
/* Core message header common to all types. Each functional message type **must**
 have this first in their list of declarations. */
typedef struct _mbed_rpc_Header {
    uint8_t version; /* Version of this message. */
    uint8_t seqId; /* Sequence ID for the message transaction */
    uint8_t msgId; /* Root message identifier */
    uint8_t svcId; /* Service identifier the message is for */
} mbed_rpc_Header;

/* Root type that parsers can use to peek at messages and figure out what type the full message is. */
typedef struct _mbed_rpc_BaseMessage {
    mbed_rpc_Header header;
} mbed_rpc_BaseMessage;

/* Simple empty message type */
typedef struct _mbed_rpc_NullMessage {
    mbed_rpc_Header header;
} mbed_rpc_NullMessage;

typedef PB_BYTES_ARRAY_T(256) mbed_rpc_ErrorMessage_detail_t;
/* Message type for error responses. */
typedef struct _mbed_rpc_ErrorMessage {
    mbed_rpc_Header header;
    mbed_rpc_ErrorCode error;
    mbed_rpc_ErrorMessage_detail_t detail;
} mbed_rpc_ErrorMessage;

typedef PB_BYTES_ARRAY_T(64) mbed_rpc_AckNackMessage_data_t;
/* Generic ACK or NACK to a previous message, with optional data payload */
typedef struct _mbed_rpc_AckNackMessage {
    mbed_rpc_Header header;
    bool acknowledge; /* True if this is an ACK, false if it's a NACK */
    bool has_status_code;
    mbed_rpc_ErrorCode status_code; /* Optional error code for the ACK/NACK */
    bool has_data;
    mbed_rpc_AckNackMessage_data_t data; /* Optional data payload for small responses */
} mbed_rpc_AckNackMessage;

/* Advertise the current system tick time */
typedef struct _mbed_rpc_TickMessage {
    mbed_rpc_Header header;
    uint32_t tick; /* System time in milliseconds */
} mbed_rpc_TickMessage;

typedef PB_BYTES_ARRAY_T(128) mbed_rpc_ConsoleMessage_data_t;
/* Stream console messages in real time */
typedef struct _mbed_rpc_ConsoleMessage {
    mbed_rpc_Header header;
    uint8_t this_frame; /* Which frame is this? */
    uint8_t total_frames; /* How many frames are there? */
    mbed_rpc_ConsoleMessage_data_t data; /* Data payload */
} mbed_rpc_ConsoleMessage;

/* Message type for announcing some device descriptions */
typedef struct _mbed_rpc_SystemInfoMessage {
    mbed_rpc_Header header;
    char sw_version[16]; /* Software version */
    char serial_number[16]; /* Serial number */
    char description[32]; /* Device description */
} mbed_rpc_SystemInfoMessage;

/* Simple ping message to test RPC connection. */
typedef struct _mbed_rpc_PingMessage {
    mbed_rpc_Header header;
} mbed_rpc_PingMessage;

typedef struct _mbed_rpc_NotifyTimeElapsedRequest {
    mbed_rpc_Header header;
    uint32_t delay_time; /* Time to delay in ms */
} mbed_rpc_NotifyTimeElapsedRequest;

typedef struct _mbed_rpc_NotifyTimeElapsedResponse {
    mbed_rpc_Header header;
    uint32_t elapsed_time; /* Time elapsed in ms */
} mbed_rpc_NotifyTimeElapsedResponse;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _mbed_rpc_ProtocolVersion_MIN mbed_rpc_ProtocolVersion_RPC_PROTOCOL_VER
#define _mbed_rpc_ProtocolVersion_MAX mbed_rpc_ProtocolVersion_RPC_PROTOCOL_VER
#define _mbed_rpc_ProtocolVersion_ARRAYSIZE ((mbed_rpc_ProtocolVersion)(mbed_rpc_ProtocolVersion_RPC_PROTOCOL_VER+1))

#define _mbed_rpc_ErrorCode_MIN mbed_rpc_ErrorCode_ERR_NO_ERROR
#define _mbed_rpc_ErrorCode_MAX mbed_rpc_ErrorCode_ERR_MAX_ERROR
#define _mbed_rpc_ErrorCode_ARRAYSIZE ((mbed_rpc_ErrorCode)(mbed_rpc_ErrorCode_ERR_MAX_ERROR+1))

#define _mbed_rpc_BuiltinService_MIN mbed_rpc_BuiltinService_SVC_PING
#define _mbed_rpc_BuiltinService_MAX mbed_rpc_BuiltinService_SVC_NOTIFY_TIME_ELAPSED
#define _mbed_rpc_BuiltinService_ARRAYSIZE ((mbed_rpc_BuiltinService)(mbed_rpc_BuiltinService_SVC_NOTIFY_TIME_ELAPSED+1))

#define _mbed_rpc_BuiltinMessage_MIN mbed_rpc_BuiltinMessage_MSG_NULL
#define _mbed_rpc_BuiltinMessage_MAX mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_RSP
#define _mbed_rpc_BuiltinMessage_ARRAYSIZE ((mbed_rpc_BuiltinMessage)(mbed_rpc_BuiltinMessage_MSG_NOTIFY_TIME_ELAPSED_RSP+1))

#define _mbed_rpc_BuiltinMessageVersion_MIN mbed_rpc_BuiltinMessageVersion_MSG_VER_ACK_NACK
#define _mbed_rpc_BuiltinMessageVersion_MAX mbed_rpc_BuiltinMessageVersion_MSG_VER_TICK
#define _mbed_rpc_BuiltinMessageVersion_ARRAYSIZE ((mbed_rpc_BuiltinMessageVersion)(mbed_rpc_BuiltinMessageVersion_MSG_VER_TICK+1))




#define mbed_rpc_ErrorMessage_error_ENUMTYPE mbed_rpc_ErrorCode

#define mbed_rpc_AckNackMessage_status_code_ENUMTYPE mbed_rpc_ErrorCode








/* Initializer values for message structs */
#define mbed_rpc_Header_init_default             {0, 0, 0, 0}
#define mbed_rpc_BaseMessage_init_default        {mbed_rpc_Header_init_default}
#define mbed_rpc_NullMessage_init_default        {mbed_rpc_Header_init_default}
#define mbed_rpc_ErrorMessage_init_default       {mbed_rpc_Header_init_default, _mbed_rpc_ErrorCode_MIN, {0, {0}}}
#define mbed_rpc_AckNackMessage_init_default     {mbed_rpc_Header_init_default, 0, false, _mbed_rpc_ErrorCode_MIN, false, {0, {0}}}
#define mbed_rpc_TickMessage_init_default        {mbed_rpc_Header_init_default, 0}
#define mbed_rpc_ConsoleMessage_init_default     {mbed_rpc_Header_init_default, 0, 0, {0, {0}}}
#define mbed_rpc_SystemInfoMessage_init_default  {mbed_rpc_Header_init_default, "", "", ""}
#define mbed_rpc_PingMessage_init_default        {mbed_rpc_Header_init_default}
#define mbed_rpc_NotifyTimeElapsedRequest_init_default {mbed_rpc_Header_init_default, 0}
#define mbed_rpc_NotifyTimeElapsedResponse_init_default {mbed_rpc_Header_init_default, 0}
#define mbed_rpc_Header_init_zero                {0, 0, 0, 0}
#define mbed_rpc_BaseMessage_init_zero           {mbed_rpc_Header_init_zero}
#define mbed_rpc_NullMessage_init_zero           {mbed_rpc_Header_init_zero}
#define mbed_rpc_ErrorMessage_init_zero          {mbed_rpc_Header_init_zero, _mbed_rpc_ErrorCode_MIN, {0, {0}}}
#define mbed_rpc_AckNackMessage_init_zero        {mbed_rpc_Header_init_zero, 0, false, _mbed_rpc_ErrorCode_MIN, false, {0, {0}}}
#define mbed_rpc_TickMessage_init_zero           {mbed_rpc_Header_init_zero, 0}
#define mbed_rpc_ConsoleMessage_init_zero        {mbed_rpc_Header_init_zero, 0, 0, {0, {0}}}
#define mbed_rpc_SystemInfoMessage_init_zero     {mbed_rpc_Header_init_zero, "", "", ""}
#define mbed_rpc_PingMessage_init_zero           {mbed_rpc_Header_init_zero}
#define mbed_rpc_NotifyTimeElapsedRequest_init_zero {mbed_rpc_Header_init_zero, 0}
#define mbed_rpc_NotifyTimeElapsedResponse_init_zero {mbed_rpc_Header_init_zero, 0}

/* Field tags (for use in manual encoding/decoding) */
#define mbed_rpc_Header_version_tag              1
#define mbed_rpc_Header_seqId_tag                2
#define mbed_rpc_Header_msgId_tag                3
#define mbed_rpc_Header_svcId_tag                4
#define mbed_rpc_BaseMessage_header_tag          1
#define mbed_rpc_NullMessage_header_tag          1
#define mbed_rpc_ErrorMessage_header_tag         1
#define mbed_rpc_ErrorMessage_error_tag          2
#define mbed_rpc_ErrorMessage_detail_tag         3
#define mbed_rpc_AckNackMessage_header_tag       1
#define mbed_rpc_AckNackMessage_acknowledge_tag  2
#define mbed_rpc_AckNackMessage_status_code_tag  3
#define mbed_rpc_AckNackMessage_data_tag         4
#define mbed_rpc_TickMessage_header_tag          1
#define mbed_rpc_TickMessage_tick_tag            2
#define mbed_rpc_ConsoleMessage_header_tag       1
#define mbed_rpc_ConsoleMessage_this_frame_tag   2
#define mbed_rpc_ConsoleMessage_total_frames_tag 3
#define mbed_rpc_ConsoleMessage_data_tag         4
#define mbed_rpc_SystemInfoMessage_header_tag    1
#define mbed_rpc_SystemInfoMessage_sw_version_tag 2
#define mbed_rpc_SystemInfoMessage_serial_number_tag 3
#define mbed_rpc_SystemInfoMessage_description_tag 4
#define mbed_rpc_PingMessage_header_tag          1
#define mbed_rpc_NotifyTimeElapsedRequest_header_tag 1
#define mbed_rpc_NotifyTimeElapsedRequest_delay_time_tag 2
#define mbed_rpc_NotifyTimeElapsedResponse_header_tag 1
#define mbed_rpc_NotifyTimeElapsedResponse_elapsed_time_tag 2

/* Struct field encoding specification for nanopb */
#define mbed_rpc_Header_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   version,           1) \
X(a, STATIC,   REQUIRED, UINT32,   seqId,             2) \
X(a, STATIC,   REQUIRED, UINT32,   msgId,             3) \
X(a, STATIC,   REQUIRED, UINT32,   svcId,             4)
#define mbed_rpc_Header_CALLBACK NULL
#define mbed_rpc_Header_DEFAULT NULL

#define mbed_rpc_BaseMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define mbed_rpc_BaseMessage_CALLBACK NULL
#define mbed_rpc_BaseMessage_DEFAULT NULL
#define mbed_rpc_BaseMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_NullMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define mbed_rpc_NullMessage_CALLBACK NULL
#define mbed_rpc_NullMessage_DEFAULT NULL
#define mbed_rpc_NullMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_ErrorMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UENUM,    error,             2) \
X(a, STATIC,   REQUIRED, BYTES,    detail,            3)
#define mbed_rpc_ErrorMessage_CALLBACK NULL
#define mbed_rpc_ErrorMessage_DEFAULT NULL
#define mbed_rpc_ErrorMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_AckNackMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, BOOL,     acknowledge,       2) \
X(a, STATIC,   OPTIONAL, UENUM,    status_code,       3) \
X(a, STATIC,   OPTIONAL, BYTES,    data,              4)
#define mbed_rpc_AckNackMessage_CALLBACK NULL
#define mbed_rpc_AckNackMessage_DEFAULT NULL
#define mbed_rpc_AckNackMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_TickMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   tick,              2)
#define mbed_rpc_TickMessage_CALLBACK NULL
#define mbed_rpc_TickMessage_DEFAULT NULL
#define mbed_rpc_TickMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_ConsoleMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   this_frame,        2) \
X(a, STATIC,   REQUIRED, UINT32,   total_frames,      3) \
X(a, STATIC,   REQUIRED, BYTES,    data,              4)
#define mbed_rpc_ConsoleMessage_CALLBACK NULL
#define mbed_rpc_ConsoleMessage_DEFAULT NULL
#define mbed_rpc_ConsoleMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_SystemInfoMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, STRING,   sw_version,        2) \
X(a, STATIC,   REQUIRED, STRING,   serial_number,     3) \
X(a, STATIC,   REQUIRED, STRING,   description,       4)
#define mbed_rpc_SystemInfoMessage_CALLBACK NULL
#define mbed_rpc_SystemInfoMessage_DEFAULT NULL
#define mbed_rpc_SystemInfoMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_PingMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1)
#define mbed_rpc_PingMessage_CALLBACK NULL
#define mbed_rpc_PingMessage_DEFAULT NULL
#define mbed_rpc_PingMessage_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_NotifyTimeElapsedRequest_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   delay_time,        2)
#define mbed_rpc_NotifyTimeElapsedRequest_CALLBACK NULL
#define mbed_rpc_NotifyTimeElapsedRequest_DEFAULT NULL
#define mbed_rpc_NotifyTimeElapsedRequest_header_MSGTYPE mbed_rpc_Header

#define mbed_rpc_NotifyTimeElapsedResponse_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   REQUIRED, UINT32,   elapsed_time,      2)
#define mbed_rpc_NotifyTimeElapsedResponse_CALLBACK NULL
#define mbed_rpc_NotifyTimeElapsedResponse_DEFAULT NULL
#define mbed_rpc_NotifyTimeElapsedResponse_header_MSGTYPE mbed_rpc_Header

extern const pb_msgdesc_t mbed_rpc_Header_msg;
extern const pb_msgdesc_t mbed_rpc_BaseMessage_msg;
extern const pb_msgdesc_t mbed_rpc_NullMessage_msg;
extern const pb_msgdesc_t mbed_rpc_ErrorMessage_msg;
extern const pb_msgdesc_t mbed_rpc_AckNackMessage_msg;
extern const pb_msgdesc_t mbed_rpc_TickMessage_msg;
extern const pb_msgdesc_t mbed_rpc_ConsoleMessage_msg;
extern const pb_msgdesc_t mbed_rpc_SystemInfoMessage_msg;
extern const pb_msgdesc_t mbed_rpc_PingMessage_msg;
extern const pb_msgdesc_t mbed_rpc_NotifyTimeElapsedRequest_msg;
extern const pb_msgdesc_t mbed_rpc_NotifyTimeElapsedResponse_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define mbed_rpc_Header_fields &mbed_rpc_Header_msg
#define mbed_rpc_BaseMessage_fields &mbed_rpc_BaseMessage_msg
#define mbed_rpc_NullMessage_fields &mbed_rpc_NullMessage_msg
#define mbed_rpc_ErrorMessage_fields &mbed_rpc_ErrorMessage_msg
#define mbed_rpc_AckNackMessage_fields &mbed_rpc_AckNackMessage_msg
#define mbed_rpc_TickMessage_fields &mbed_rpc_TickMessage_msg
#define mbed_rpc_ConsoleMessage_fields &mbed_rpc_ConsoleMessage_msg
#define mbed_rpc_SystemInfoMessage_fields &mbed_rpc_SystemInfoMessage_msg
#define mbed_rpc_PingMessage_fields &mbed_rpc_PingMessage_msg
#define mbed_rpc_NotifyTimeElapsedRequest_fields &mbed_rpc_NotifyTimeElapsedRequest_msg
#define mbed_rpc_NotifyTimeElapsedResponse_fields &mbed_rpc_NotifyTimeElapsedResponse_msg

/* Maximum encoded size of messages (where known) */
#define MBED_RPC_MBED_RPC_PB_H_MAX_SIZE          mbed_rpc_ErrorMessage_size
#define mbed_rpc_AckNackMessage_size             85
#define mbed_rpc_BaseMessage_size                14
#define mbed_rpc_ConsoleMessage_size             151
#define mbed_rpc_ErrorMessage_size               276
#define mbed_rpc_Header_size                     12
#define mbed_rpc_NotifyTimeElapsedRequest_size   20
#define mbed_rpc_NotifyTimeElapsedResponse_size  20
#define mbed_rpc_NullMessage_size                14
#define mbed_rpc_PingMessage_size                14
#define mbed_rpc_SystemInfoMessage_size          81
#define mbed_rpc_TickMessage_size                20

#ifdef __cplusplus
} /* extern "C" */
#endif

#ifdef __cplusplus
/* Message descriptors for nanopb */
namespace nanopb {
template <>
struct MessageDescriptor<mbed_rpc_Header> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
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
struct MessageDescriptor<mbed_rpc_NullMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_NullMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_ErrorMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 3;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_ErrorMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_AckNackMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_AckNackMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_TickMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_TickMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_ConsoleMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_ConsoleMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_SystemInfoMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 4;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_SystemInfoMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_PingMessage> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 1;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_PingMessage_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_NotifyTimeElapsedRequest> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_NotifyTimeElapsedRequest_msg;
    }
};
template <>
struct MessageDescriptor<mbed_rpc_NotifyTimeElapsedResponse> {
    static PB_INLINE_CONSTEXPR const pb_size_t fields_array_length = 2;
    static inline const pb_msgdesc_t* fields() {
        return &mbed_rpc_NotifyTimeElapsedResponse_msg;
    }
};
}  // namespace nanopb

#endif  /* __cplusplus */


#endif
