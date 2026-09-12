#include "avg_protocol.h"

#include <string.h>

static uint16_t AVG_ReadU16Le(const uint8_t *source)
{
  return (uint16_t)((uint16_t)source[0] |
                    ((uint16_t)source[1] << 8U));
}

static uint32_t AVG_ReadU32Le(const uint8_t *source)
{
  return (uint32_t)source[0] |
         ((uint32_t)source[1] << 8U) |
         ((uint32_t)source[2] << 16U) |
         ((uint32_t)source[3] << 24U);
}

static void AVG_WriteU16Le(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
}

static void AVG_WriteU32Le(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
  destination[2] = (uint8_t)(value >> 16U);
  destination[3] = (uint8_t)(value >> 24U);
}

static void AVG_WriteFloatLe(uint8_t *destination, float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  AVG_WriteU32Le(destination, bits);
}

static float AVG_ReadFloatLe(const uint8_t *source)
{
  uint32_t bits;
  float value;

  bits = AVG_ReadU32Le(source);
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static uint8_t AVG_FloatIsFinite(float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  return ((bits & 0x7F800000UL) != 0x7F800000UL) ? 1U : 0U;
}

static void AVG_ParserDiscard(AvgProtocolParser_t *parser, uint16_t count)
{
  if (count >= parser->used)
  {
    parser->used = 0U;
    return;
  }

  memmove(parser->bytes, &parser->bytes[count],
          (size_t)(parser->used - count));
  parser->used = (uint16_t)(parser->used - count);
}

static void AVG_ProtocolParserProcess(AvgProtocolParser_t *parser,
                                      AvgProtocolFrameHandler_t handler,
                                      void *context)
{
  AvgProtocolFrame_t frame;
  uint16_t payload_len;
  uint16_t frame_len;
  uint16_t expected_crc;
  uint16_t actual_crc;
  uint16_t sync_index;

  for (;;)
  {
    if (parser->used < 2U)
    {
      return;
    }

    sync_index = 0U;
    while ((uint16_t)(sync_index + 1U) < parser->used)
    {
      if ((parser->bytes[sync_index] == AVG_PROTOCOL_SYNC_0) &&
          (parser->bytes[sync_index + 1U] == AVG_PROTOCOL_SYNC_1))
      {
        break;
      }
      ++sync_index;
    }

    if ((uint16_t)(sync_index + 1U) >= parser->used)
    {
      if (parser->bytes[parser->used - 1U] == AVG_PROTOCOL_SYNC_0)
      {
        parser->bytes[0] = AVG_PROTOCOL_SYNC_0;
        parser->used = 1U;
      }
      else
      {
        parser->used = 0U;
      }
      return;
    }

    if (sync_index != 0U)
    {
      AVG_ParserDiscard(parser, sync_index);
    }

    if (parser->used < AVG_PROTOCOL_HEADER_SIZE)
    {
      return;
    }

    payload_len = AVG_ReadU16Le(&parser->bytes[5]);
    if ((parser->bytes[2] != AVG_PROTOCOL_VERSION) ||
        (payload_len > AVG_PROTOCOL_MAX_PAYLOAD))
    {
      AVG_ParserDiscard(parser, 1U);
      continue;
    }

    frame_len = (uint16_t)(AVG_PROTOCOL_HEADER_SIZE + payload_len +
                           AVG_PROTOCOL_CRC_SIZE);
    if (parser->used < frame_len)
    {
      return;
    }

    expected_crc = AVG_ReadU16Le(&parser->bytes[AVG_PROTOCOL_HEADER_SIZE +
                                                payload_len]);
    actual_crc = AVG_ProtocolCrc16(&parser->bytes[2],
                                   (size_t)(9U + payload_len));
    if (actual_crc != expected_crc)
    {
      AVG_ParserDiscard(parser, 1U);
      continue;
    }

    frame.message_type = parser->bytes[3];
    frame.flags = parser->bytes[4];
    frame.payload_len = payload_len;
    frame.transport_sequence = AVG_ReadU32Le(&parser->bytes[7]);
    frame.payload = &parser->bytes[AVG_PROTOCOL_HEADER_SIZE];
    if (handler != NULL)
    {
      handler(&frame, context);
    }
    AVG_ParserDiscard(parser, frame_len);
  }
}

uint16_t AVG_ProtocolCrc16(const uint8_t *data, size_t length)
{
  uint16_t crc;
  uint8_t bit;

  crc = 0xFFFFU;
  while (length-- != 0U)
  {
    crc ^= (uint16_t)((uint16_t)(*data++) << 8U);
    for (bit = 0U; bit < 8U; ++bit)
    {
      crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1U) ^ 0x1021U) :
            (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

void AVG_ProtocolParserInit(AvgProtocolParser_t *parser)
{
  if (parser != NULL)
  {
    parser->used = 0U;
  }
}

void AVG_ProtocolParserFeed(AvgProtocolParser_t *parser,
                            const uint8_t *data, size_t length,
                            AvgProtocolFrameHandler_t handler, void *context)
{
  if ((parser == NULL) || ((data == NULL) && (length != 0U)))
  {
    return;
  }

  while (length-- != 0U)
  {
    if (parser->used >= AVG_PROTOCOL_MAX_FRAME_SIZE)
    {
      AVG_ParserDiscard(parser, 1U);
    }
    parser->bytes[parser->used++] = *data++;
    AVG_ProtocolParserProcess(parser, handler, context);
  }
}

uint16_t AVG_ProtocolBuildFrame(uint8_t *destination,
                                uint16_t destination_size,
                                uint8_t message_type, uint8_t flags,
                                uint32_t transport_sequence,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
  uint16_t frame_len;
  uint16_t crc;

  if ((destination == NULL) ||
      ((payload == NULL) && (payload_len != 0U)) ||
      (payload_len > AVG_PROTOCOL_MAX_PAYLOAD))
  {
    return 0U;
  }

  frame_len = (uint16_t)(AVG_PROTOCOL_HEADER_SIZE + payload_len +
                         AVG_PROTOCOL_CRC_SIZE);
  if (destination_size < frame_len)
  {
    return 0U;
  }

  destination[0] = AVG_PROTOCOL_SYNC_0;
  destination[1] = AVG_PROTOCOL_SYNC_1;
  destination[2] = AVG_PROTOCOL_VERSION;
  destination[3] = message_type;
  destination[4] = flags;
  AVG_WriteU16Le(&destination[5], payload_len);
  AVG_WriteU32Le(&destination[7], transport_sequence);
  if (payload_len != 0U)
  {
    memcpy(&destination[AVG_PROTOCOL_HEADER_SIZE], payload, payload_len);
  }
  crc = AVG_ProtocolCrc16(&destination[2], (size_t)(9U + payload_len));
  AVG_WriteU16Le(&destination[AVG_PROTOCOL_HEADER_SIZE + payload_len], crc);
  return frame_len;
}

uint8_t AVG_ProtocolDecodeDrive(const AvgProtocolFrame_t *frame,
                                float *linear_mps, float *angular_rps)
{
  float linear;
  float angular;

  if ((frame == NULL) || (linear_mps == NULL) || (angular_rps == NULL) ||
      (frame->message_type != AVG_MESSAGE_DRIVE) ||
      (frame->flags != 0U) ||
      (frame->payload_len != sizeof(AvgDrivePayload_t)))
  {
    return 0U;
  }

  linear = AVG_ReadFloatLe(&frame->payload[0]);
  angular = AVG_ReadFloatLe(&frame->payload[4]);
  if ((AVG_FloatIsFinite(linear) == 0U) ||
      (AVG_FloatIsFinite(angular) == 0U))
  {
    return 0U;
  }

  *linear_mps = linear;
  *angular_rps = angular;
  return 1U;
}

uint8_t AVG_ProtocolDecodeHolonomicDrive(const AvgProtocolFrame_t *frame,
                                         float *linear_mps,
                                         float *lateral_mps,
                                         float *angular_rps)
{
  float linear;
  float lateral;
  float angular;

  if ((frame == NULL) || (linear_mps == NULL) || (lateral_mps == NULL) ||
      (angular_rps == NULL) ||
      (frame->message_type != AVG_MESSAGE_DRIVE_HOLONOMIC) ||
      (frame->flags != 0U) ||
      (frame->payload_len != sizeof(AvgHolonomicDrivePayload_t)))
  {
    return 0U;
  }

  linear = AVG_ReadFloatLe(&frame->payload[0]);
  lateral = AVG_ReadFloatLe(&frame->payload[4]);
  angular = AVG_ReadFloatLe(&frame->payload[8]);
  if ((AVG_FloatIsFinite(linear) == 0U) ||
      (AVG_FloatIsFinite(lateral) == 0U) ||
      (AVG_FloatIsFinite(angular) == 0U))
  {
    return 0U;
  }

  *linear_mps = linear;
  *lateral_mps = lateral;
  *angular_rps = angular;
  return 1U;
}

uint8_t AVG_ProtocolDecodeWheelTest(const AvgProtocolFrame_t *frame,
                                    uint8_t *wheel_index, int16_t *target_rpm)
{
  if ((frame == NULL) || (wheel_index == NULL) || (target_rpm == NULL) ||
      (frame->message_type != AVG_MESSAGE_WHEEL_TEST) ||
      (frame->flags != 0U) ||
      (frame->payload_len != sizeof(AvgWheelTestPayload_t)))
  {
    return 0U;
  }
  *wheel_index = frame->payload[0];
  *target_rpm = (int16_t)AVG_ReadU16Le(&frame->payload[1]);
  return 1U;
}

uint16_t AVG_ProtocolEncodeAckPayload(uint8_t destination[2],
                                      uint8_t acked_type,
                                      AvgAckResult_t result)
{
  if (destination == NULL)
  {
    return 0U;
  }
  destination[0] = acked_type;
  destination[1] = (uint8_t)result;
  return sizeof(AvgAckPayload_t);
}

uint16_t AVG_ProtocolEncodeStatusPayload(uint8_t destination[34],
                                         const AvgStatusPayload_t *status)
{
  if ((destination == NULL) || (status == NULL))
  {
    return 0U;
  }

  AVG_WriteU16Le(&destination[0], status->battery_mv);
  AVG_WriteU16Le(&destination[2], status->ultrasonic_mm);
  AVG_WriteU32Le(&destination[4], (uint32_t)status->encoder_fl);
  AVG_WriteU32Le(&destination[8], (uint32_t)status->encoder_fr);
  AVG_WriteU32Le(&destination[12], (uint32_t)status->encoder_rl);
  AVG_WriteU32Le(&destination[16], (uint32_t)status->encoder_rr);
  AVG_WriteFloatLe(&destination[20], status->measured_linear_mps);
  AVG_WriteFloatLe(&destination[24], status->measured_angular_rps);
  AVG_WriteFloatLe(&destination[28], status->imu_yaw_rad);
  destination[32] = status->estop;
  destination[33] = status->fault_code;
  return sizeof(AvgStatusPayload_t);
}
