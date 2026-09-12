#include "avg_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
  unsigned int count;
  uint8_t type[8];
  uint8_t flags[8];
  uint32_t sequence[8];
  uint16_t payload_len[8];
  uint8_t payload[8][AVG_PROTOCOL_MAX_PAYLOAD];
} Capture_t;

static void capture_frame(const AvgProtocolFrame_t *frame, void *context)
{
  Capture_t *capture;
  unsigned int index;

  capture = (Capture_t *)context;
  index = capture->count++;
  assert(index < 8U);
  capture->type[index] = frame->message_type;
  capture->flags[index] = frame->flags;
  capture->sequence[index] = frame->transport_sequence;
  capture->payload_len[index] = frame->payload_len;
  if (frame->payload_len != 0U)
  {
    memcpy(capture->payload[index], frame->payload, frame->payload_len);
  }
}

static void write_u32_le(uint8_t destination[4], uint32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
  destination[2] = (uint8_t)(value >> 16U);
  destination[3] = (uint8_t)(value >> 24U);
}

static void write_float_le(uint8_t destination[4], float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  write_u32_le(destination, bits);
}

static void test_crc_vector(void)
{
  static const uint8_t vector[] = "123456789";

  assert(AVG_ProtocolCrc16(vector, 9U) == 0x29B1U);
}

static void test_fragmented_drive(void)
{
  AvgProtocolParser_t parser;
  Capture_t capture = {0};
  AvgProtocolFrame_t decoded_frame;
  uint8_t payload[8];
  uint8_t frame[32];
  uint16_t length;
  uint16_t index;
  float linear;
  float angular;

  write_float_le(&payload[0], 0.25f);
  write_float_le(&payload[4], -1.0f);
  length = AVG_ProtocolBuildFrame(frame, sizeof(frame), AVG_MESSAGE_DRIVE,
                                  0U, 0x78563412UL, payload,
                                  sizeof(payload));
  assert(length == 21U);
  assert(frame[0] == 0xA5U && frame[1] == 0x5AU);
  assert(frame[5] == 8U && frame[6] == 0U);
  assert(frame[7] == 0x12U && frame[10] == 0x78U);

  AVG_ProtocolParserInit(&parser);
  for (index = 0U; index < length; ++index)
  {
    AVG_ProtocolParserFeed(&parser, &frame[index], 1U,
                           capture_frame, &capture);
  }
  assert(capture.count == 1U);
  assert(capture.type[0] == AVG_MESSAGE_DRIVE);
  assert(capture.sequence[0] == 0x78563412UL);

  decoded_frame.message_type = capture.type[0];
  decoded_frame.flags = 0U;
  decoded_frame.payload_len = capture.payload_len[0];
  decoded_frame.transport_sequence = capture.sequence[0];
  decoded_frame.payload = capture.payload[0];
  assert(AVG_ProtocolDecodeDrive(&decoded_frame, &linear, &angular) == 1U);
  assert(linear == 0.25f);
  assert(angular == -1.0f);
}

static void test_noise_bad_crc_and_sticky_frames(void)
{
  AvgProtocolParser_t parser;
  Capture_t capture = {0};
  uint8_t estop_payload = 0x01U;
  uint8_t clear_payload = 0xA5U;
  uint8_t first[24];
  uint8_t bad[24];
  uint8_t second[24];
  uint8_t stream[80];
  uint16_t first_len;
  uint16_t second_len;
  size_t offset;

  first_len = AVG_ProtocolBuildFrame(first, sizeof(first),
                                     AVG_MESSAGE_ESTOP, 0U, 10U,
                                     &estop_payload, 1U);
  second_len = AVG_ProtocolBuildFrame(second, sizeof(second),
                                      AVG_MESSAGE_CLEAR_ESTOP, 0U, 11U,
                                      &clear_payload, 1U);
  memcpy(bad, first, first_len);
  bad[11] ^= 0x40U;

  offset = 0U;
  stream[offset++] = 0x00U;
  stream[offset++] = 0xA5U;
  stream[offset++] = 0x00U;
  memcpy(&stream[offset], bad, first_len);
  offset += first_len;
  memcpy(&stream[offset], first, first_len);
  offset += first_len;
  memcpy(&stream[offset], second, second_len);
  offset += second_len;

  AVG_ProtocolParserInit(&parser);
  AVG_ProtocolParserFeed(&parser, stream, offset, capture_frame, &capture);
  assert(capture.count == 2U);
  assert(capture.type[0] == AVG_MESSAGE_ESTOP);
  assert(capture.sequence[0] == 10U);
  assert(capture.type[1] == AVG_MESSAGE_CLEAR_ESTOP);
  assert(capture.sequence[1] == 11U);
}

static void test_non_finite_drive_rejected(void)
{
  AvgProtocolFrame_t frame;
  uint8_t payload[8] = {0};
  float linear;
  float angular;

  write_u32_le(&payload[0], 0x7FC00000UL);
  write_float_le(&payload[4], 0.0f);
  frame.message_type = AVG_MESSAGE_DRIVE;
  frame.flags = 0U;
  frame.payload_len = sizeof(payload);
  frame.transport_sequence = 1U;
  frame.payload = payload;
  assert(AVG_ProtocolDecodeDrive(&frame, &linear, &angular) == 0U);
}

static void test_holonomic_drive_decode(void)
{
  AvgProtocolFrame_t frame;
  uint8_t payload[sizeof(AvgHolonomicDrivePayload_t)];
  float linear;
  float lateral;
  float angular;

  write_float_le(&payload[0], 0.20f);
  write_float_le(&payload[4], -0.35f);
  write_float_le(&payload[8], 0.80f);
  frame.message_type = AVG_MESSAGE_DRIVE_HOLONOMIC;
  frame.flags = 0U;
  frame.payload_len = sizeof(payload);
  frame.transport_sequence = 2U;
  frame.payload = payload;
  assert(AVG_ProtocolDecodeHolonomicDrive(&frame, &linear, &lateral,
                                          &angular) == 1U);
  assert(linear == 0.20f);
  assert(lateral == -0.35f);
  assert(angular == 0.80f);
}

static void test_status_pairing_flag_round_trip(void)
{
  AvgProtocolParser_t parser;
  Capture_t capture = {0};
  uint8_t payload[sizeof(AvgStatusPayload_t)] = {0};
  uint8_t frame[AVG_PROTOCOL_MAX_FRAME_SIZE];
  uint16_t length;

  length = AVG_ProtocolBuildFrame(frame, sizeof(frame), AVG_MESSAGE_STATUS,
                                  AVG_STATUS_FLAG_BLE_PAIR_REQUEST, 77U,
                                  payload, sizeof(payload));
  assert(length != 0U);
  AVG_ProtocolParserInit(&parser);
  AVG_ProtocolParserFeed(&parser, frame, length, capture_frame, &capture);
  assert(capture.count == 1U);
  assert(capture.type[0] == AVG_MESSAGE_STATUS);
  assert(capture.flags[0] == AVG_STATUS_FLAG_BLE_PAIR_REQUEST);
  assert(capture.sequence[0] == 77U);
}

static void test_max_payload_and_status_layout(void)
{
  AvgProtocolParser_t parser;
  Capture_t capture = {0};
  AvgStatusPayload_t status = {0};
  uint8_t payload[AVG_PROTOCOL_MAX_PAYLOAD];
  uint8_t frame[AVG_PROTOCOL_MAX_FRAME_SIZE];
  uint8_t status_bytes[sizeof(AvgStatusPayload_t)];
  uint16_t length;
  unsigned int index;

  for (index = 0U; index < sizeof(payload); ++index)
  {
    payload[index] = (uint8_t)index;
  }
  length = AVG_ProtocolBuildFrame(frame, sizeof(frame), 0x55U, 0U,
                                  99U, payload, sizeof(payload));
  assert(length == AVG_PROTOCOL_MAX_FRAME_SIZE);
  AVG_ProtocolParserInit(&parser);
  AVG_ProtocolParserFeed(&parser, frame, length, capture_frame, &capture);
  assert(capture.count == 1U);
  assert(capture.payload_len[0] == AVG_PROTOCOL_MAX_PAYLOAD);
  assert(memcmp(capture.payload[0], payload, sizeof(payload)) == 0);

  status.battery_mv = 12345U;
  status.ultrasonic_mm = 0xFFFFU;
  status.encoder_fl = -2;
  status.encoder_rr = 0x12345678L;
  status.measured_linear_mps = 0.5f;
  status.estop = 1U;
  status.fault_code = AVG_FAULT_COMM_TIMEOUT;
  assert(AVG_ProtocolEncodeStatusPayload(status_bytes, &status) == 34U);
  assert(status_bytes[0] == 0x39U && status_bytes[1] == 0x30U);
  assert(status_bytes[2] == 0xFFU && status_bytes[3] == 0xFFU);
  assert(status_bytes[4] == 0xFEU && status_bytes[7] == 0xFFU);
  assert(status_bytes[16] == 0x78U && status_bytes[19] == 0x12U);
  assert(status_bytes[32] == 1U);
  assert(status_bytes[33] == AVG_FAULT_COMM_TIMEOUT);
}

int main(void)
{
  test_crc_vector();
  test_fragmented_drive();
  test_noise_bad_crc_and_sticky_frames();
  test_non_finite_drive_rejected();
  test_holonomic_drive_decode();
  test_status_pairing_flag_round_trip();
  test_max_payload_and_status_layout();
  puts("avg_protocol tests passed");
  return 0;
}
