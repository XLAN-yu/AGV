#ifndef AVG_PROTOCOL_H
#define AVG_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AVG_PROTOCOL_SYNC_0                0xA5U
#define AVG_PROTOCOL_SYNC_1                0x5AU
#define AVG_PROTOCOL_VERSION               0x01U
#define AVG_PROTOCOL_HEADER_SIZE           11U
#define AVG_PROTOCOL_CRC_SIZE              2U
#define AVG_PROTOCOL_MAX_PAYLOAD           256U
#define AVG_PROTOCOL_MAX_FRAME_SIZE        \
  (AVG_PROTOCOL_HEADER_SIZE + AVG_PROTOCOL_MAX_PAYLOAD + \
   AVG_PROTOCOL_CRC_SIZE)

/* STATUS frame flags.  Payload layout remains V1-compatible. */
#define AVG_STATUS_FLAG_BLE_PAIR_REQUEST   0x01U
#define AVG_STATUS_FLAG_OBSTACLE_GUARD_ENABLED 0x02U
#define AVG_STATUS_FLAG_OBSTACLE_GUARD_CAPABLE 0x04U
#define AVG_STATUS_FLAGS_KNOWN             (AVG_STATUS_FLAG_BLE_PAIR_REQUEST | \
                                            AVG_STATUS_FLAG_OBSTACLE_GUARD_ENABLED | \
                                            AVG_STATUS_FLAG_OBSTACLE_GUARD_CAPABLE)

typedef enum
{
  AVG_MESSAGE_DRIVE = 0x01,
  AVG_MESSAGE_ESTOP = 0x02,
  AVG_MESSAGE_CLEAR_ESTOP = 0x03,
  AVG_MESSAGE_DRIVE_HOLONOMIC = 0x04,
  AVG_MESSAGE_SET_OBSTACLE_GUARD = 0x05,
  AVG_MESSAGE_ORANGE_PI_SHUTDOWN_REQUEST = 0x06,
  AVG_MESSAGE_WHEEL_TEST = 0x07,
  AVG_MESSAGE_STATUS = 0x80,
  AVG_MESSAGE_ACK = 0x81
} AvgMessageType_t;

typedef enum
{
  AVG_ACK_OK = 0,
  AVG_ACK_INVALID_PAYLOAD = 1,
  AVG_ACK_ESTOP_LATCHED = 2,
  AVG_ACK_HARDWARE_FAULT = 3,
  AVG_ACK_UNSUPPORTED = 4
} AvgAckResult_t;

typedef enum
{
  AVG_FAULT_NONE = 0,
  AVG_FAULT_COMM_TIMEOUT = 1,
  AVG_FAULT_PHYSICAL_ESTOP = 2,
  AVG_FAULT_ENCODER_FRONT_LEFT = 0x11,
  AVG_FAULT_ENCODER_FRONT_RIGHT = 0x12,
  AVG_FAULT_ENCODER_REAR_LEFT = 0x13,
  AVG_FAULT_ENCODER_REAR_RIGHT = 0x14,
  AVG_FAULT_UART_RX = 0x20,
  AVG_FAULT_OBSTACLE = 0x30,
  AVG_FAULT_ULTRASONIC_UNAVAILABLE = 0x31
} AvgFaultCode_t;

#if defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)
#define AVG_PACKED __attribute__((packed))
#else
#define AVG_PACKED
#pragma pack(push, 1)
#endif

typedef struct AVG_PACKED
{
  float linear_mps;
  float angular_rps;
} AvgDrivePayload_t;

typedef struct AVG_PACKED
{
  float linear_mps;
  float lateral_mps;
  float angular_rps;
} AvgHolonomicDrivePayload_t;

typedef struct AVG_PACKED
{
  uint8_t wheel_index;
  int16_t target_rpm;
} AvgWheelTestPayload_t;

typedef struct AVG_PACKED
{
  uint16_t battery_mv;
  uint16_t ultrasonic_mm;
  int32_t encoder_fl;
  int32_t encoder_fr;
  int32_t encoder_rl;
  int32_t encoder_rr;
  float measured_linear_mps;
  float measured_angular_rps;
  float imu_yaw_rad;
  uint8_t estop;
  uint8_t fault_code;
} AvgStatusPayload_t;

typedef struct AVG_PACKED
{
  uint8_t acked_type;
  uint8_t result;
} AvgAckPayload_t;

#if !defined(__GNUC__) && !defined(__clang__) && !defined(__ARMCC_VERSION)
#pragma pack(pop)
#endif

typedef char AvgDrivePayloadSizeMustBe8[(sizeof(AvgDrivePayload_t) == 8U) ? 1 : -1];
typedef char AvgHolonomicDrivePayloadSizeMustBe12[(sizeof(AvgHolonomicDrivePayload_t) == 12U) ? 1 : -1];
typedef char AvgWheelTestPayloadSizeMustBe3[(sizeof(AvgWheelTestPayload_t) == 3U) ? 1 : -1];
typedef char AvgStatusPayloadSizeMustBe34[(sizeof(AvgStatusPayload_t) == 34U) ? 1 : -1];
typedef char AvgAckPayloadSizeMustBe2[(sizeof(AvgAckPayload_t) == 2U) ? 1 : -1];

typedef struct
{
  uint8_t message_type;
  uint8_t flags;
  uint16_t payload_len;
  uint32_t transport_sequence;
  const uint8_t *payload;
} AvgProtocolFrame_t;

typedef void (*AvgProtocolFrameHandler_t)(const AvgProtocolFrame_t *frame,
                                          void *context);

typedef struct
{
  uint8_t bytes[AVG_PROTOCOL_MAX_FRAME_SIZE];
  uint16_t used;
} AvgProtocolParser_t;

uint16_t AVG_ProtocolCrc16(const uint8_t *data, size_t length);
void AVG_ProtocolParserInit(AvgProtocolParser_t *parser);
void AVG_ProtocolParserFeed(AvgProtocolParser_t *parser,
                            const uint8_t *data, size_t length,
                            AvgProtocolFrameHandler_t handler, void *context);
uint16_t AVG_ProtocolBuildFrame(uint8_t *destination,
                                uint16_t destination_size,
                                uint8_t message_type, uint8_t flags,
                                uint32_t transport_sequence,
                                const uint8_t *payload,
                                uint16_t payload_len);
uint8_t AVG_ProtocolDecodeDrive(const AvgProtocolFrame_t *frame,
                                float *linear_mps, float *angular_rps);
uint8_t AVG_ProtocolDecodeHolonomicDrive(const AvgProtocolFrame_t *frame,
                                         float *linear_mps,
                                         float *lateral_mps,
                                         float *angular_rps);
uint8_t AVG_ProtocolDecodeWheelTest(const AvgProtocolFrame_t *frame,
                                    uint8_t *wheel_index, int16_t *target_rpm);
uint16_t AVG_ProtocolEncodeAckPayload(uint8_t destination[2],
                                      uint8_t acked_type,
                                      AvgAckResult_t result);
uint16_t AVG_ProtocolEncodeStatusPayload(uint8_t destination[34],
                                         const AvgStatusPayload_t *status);

#ifdef __cplusplus
}
#endif

#endif /* AVG_PROTOCOL_H */
