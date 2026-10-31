#include "util/ImageWriter.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>

namespace app {

namespace {

uint32_t Crc32(const uint8_t* data, size_t len, uint32_t crc = 0) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t n = 0; n < 256; ++n) {
            uint32_t c = n;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[n] = c;
        }
        return t;
    }();
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

void PutU32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

void WriteChunk(std::ofstream& f, const char type[4], const std::vector<uint8_t>& data) {
    std::vector<uint8_t> buf;
    PutU32(buf, static_cast<uint32_t>(data.size()));
    buf.insert(buf.end(), type, type + 4);
    buf.insert(buf.end(), data.begin(), data.end());
    const uint32_t crc = Crc32(buf.data() + 4, buf.size() - 4);  // CRC 覆盖类型 + 数据
    PutU32(buf, crc);
    f.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
}

}  // namespace

bool WritePng(const std::string& path, int width, int height, const std::vector<uint8_t>& rgba) {
    if (width <= 0 || height <= 0 || rgba.size() < static_cast<size_t>(width) * static_cast<size_t>(height) * 4) {
        return false;
    }
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    static const uint8_t kSignature[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    f.write(reinterpret_cast<const char*>(kSignature), 8);

    // IHDR：宽、高、位深 8、颜色类型 6 (RGBA)、压缩 0、滤波 0、隔行 0
    std::vector<uint8_t> ihdr;
    PutU32(ihdr, static_cast<uint32_t>(width));
    PutU32(ihdr, static_cast<uint32_t>(height));
    ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
    WriteChunk(f, "IHDR", ihdr);

    // 原始扫描线：每行前加一个滤波类型字节 0 (None)
    const size_t stride = static_cast<size_t>(width) * 4;
    std::vector<uint8_t> raw;
    raw.reserve((stride + 1) * static_cast<size_t>(height));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0);
        const uint8_t* row = rgba.data() + static_cast<size_t>(y) * stride;
        raw.insert(raw.end(), row, row + stride);
    }

    // zlib 流 = 2 字节头 + deflate 存储块 + Adler-32 校验
    std::vector<uint8_t> z;
    z.push_back(0x78);
    z.push_back(0x01);
    size_t pos = 0;
    do {
        const size_t len = std::min<size_t>(65535, raw.size() - pos);
        const bool last = pos + len == raw.size();
        z.push_back(last ? 1 : 0);  // BFINAL + BTYPE=00(存储)
        z.push_back(static_cast<uint8_t>(len & 0xFF));
        z.push_back(static_cast<uint8_t>(len >> 8));
        z.push_back(static_cast<uint8_t>(~len & 0xFF));
        z.push_back(static_cast<uint8_t>((~len >> 8) & 0xFF));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                 raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
    } while (pos < raw.size());

    uint32_t a = 1, b = 0;
    for (uint8_t byte : raw) {
        a = (a + byte) % 65521;
        b = (b + a) % 65521;
    }
    PutU32(z, (b << 16) | a);

    WriteChunk(f, "IDAT", z);
    WriteChunk(f, "IEND", {});
    return static_cast<bool>(f);
}

}  // namespace app
