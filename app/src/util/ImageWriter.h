// =============================================================================
// 无依赖的 PNG 写入器
//
// PNG = 8 字节签名 + 若干数据块（chunk）：IHDR(图像头) IDAT(压缩数据) IEND(结束)
// 压缩数据使用 zlib 格式；为了不引入压缩库，这里使用 deflate 的"存储"模式（不压缩），
// 文件会大一些，但格式完全合法，任何看图软件都能打开。
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace app {

/// rgba：width*height*4 字节，行从上到下
bool WritePng(const std::string& path, int width, int height, const std::vector<uint8_t>& rgba);

}  // namespace app
