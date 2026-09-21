#include "hal/sil_wire.hpp"

#include <cstddef>
#include <cstring>

namespace drone::wire {

namespace {
constexpr uint32_t kPoly = 0xEDB88320u;
uint32_t table_[256];
bool table_ready_ = false;

void buildTable() {
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) c = (c & 1u) ? (kPoly ^ (c >> 1)) : (c >> 1);
    table_[i] = c;
  }
  table_ready_ = true;
}
}  // namespace

uint32_t crc32Init() {
  if (!table_ready_) buildTable();
  return 0xFFFFFFFFu;
}

uint32_t crc32Update(uint32_t crc, const void* data, int len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  for (int i = 0; i < len; ++i) crc = table_[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
  return crc;
}

uint32_t crc32Final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

uint32_t crc32(const void* data, int len) {
  uint32_t c = crc32Init();
  c = crc32Update(c, data, len);
  return crc32Final(c);
}

int packMsg(uint8_t* buf, int cap, MsgType type, uint32_t seq, const void* payload,
            int payload_len) {
  if (payload_len < 0) return -1;
  const int total = static_cast<int>(sizeof(Hdr)) + payload_len;
  if (total > cap) return -1;

  Hdr h{};
  h.magic = kMagic;
  h.ver = kProtoVersion;
  h.type = static_cast<uint8_t>(type);
  h.len = static_cast<uint16_t>(payload_len);
  h.seq = seq;
  h.crc = 0;
  std::memcpy(buf, &h, sizeof(Hdr));
  if (payload_len > 0) std::memcpy(buf + sizeof(Hdr), payload, static_cast<size_t>(payload_len));

  const uint32_t crc = crc32(buf, total);
  std::memcpy(buf + offsetof(Hdr, crc), &crc, sizeof(crc));
  return total;
}

int unpackMsg(const uint8_t* buf, int len, Hdr& hdr, const uint8_t*& payload) {
  if (len < static_cast<int>(sizeof(Hdr))) return -1;
  std::memcpy(&hdr, buf, sizeof(Hdr));
  if (hdr.magic != kMagic) return -1;
  if (hdr.ver != kProtoVersion) return -2;
  const int total = static_cast<int>(sizeof(Hdr)) + static_cast<int>(hdr.len);
  if (total > len) return -3;

  uint8_t tmp[sizeof(Hdr)];
  std::memcpy(tmp, buf, sizeof(Hdr));
  const uint32_t zero = 0;
  std::memcpy(tmp + offsetof(Hdr, crc), &zero, sizeof(zero));

  uint32_t c = crc32Init();
  c = crc32Update(c, tmp, static_cast<int>(sizeof(Hdr)));
  c = crc32Update(c, buf + sizeof(Hdr), static_cast<int>(hdr.len));
  c = crc32Final(c);
  if (c != hdr.crc) return -4;

  payload = buf + sizeof(Hdr);
  return total;
}

void fillHello(SilHello& h) {
  h.dt_us = kSilDtUs;
  for (int i = 0; i < 7; ++i) h.sizes[i] = static_cast<uint32_t>(kHelloSizes[i]);
}

bool helloMatches(const SilHello& local, const SilHello& peer) {
  if (local.dt_us != peer.dt_us) return false;
  for (int i = 0; i < 7; ++i) {
    if (local.sizes[i] != peer.sizes[i]) return false;
  }
  return true;
}

}  // namespace drone::wire
