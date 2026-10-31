// =============================================================================
// Kinema2D 桌面客户端入口
//
// 命令行参数：
//   --scene <id>          启动时加载的场景（pyramid、car、funnel ...）
//   --lang <en|zh>        界面语言
//   --light               使用浅色主题
//   --paused              启动时暂停
//   --size <w> <h>        窗口大小
//   --screenshot <file>   运行若干帧后保存 PNG 截图并退出（自动化测试/文档用）
//   --frames <n>          截图前运行的帧数（默认 120）
//   --list-scenes         列出所有场景
// =============================================================================
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <kinema/Version.h>
#include <kinema_scene/SceneLibrary.h>

#include "core/Application.h"

namespace {

void PrintUsage() {
    std::printf(
        "Kinema2D %s - 2D physics sandbox\n\n"
        "Usage: Kinema2D [options]\n"
        "  --scene <id>          Scene to load at startup (see --list-scenes)\n"
        "  --lang <en|zh>        UI language\n"
        "  --light               Light theme\n"
        "  --paused              Start paused\n"
        "  --size <w> <h>        Window size\n"
        "  --screenshot <file>   Save a PNG screenshot after N frames and exit\n"
        "  --frames <n>          Frames to run before the screenshot (default 120)\n"
        "  --no-vsync            Disable vertical sync\n"
        "  --list-scenes         Print all scene ids\n"
        "  --help                Show this help\n",
        kinema::kVersionString);
}

}  // namespace

int main(int argc, char** argv) {
    app::AppConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--scene") config.scene = next("--scene");
        else if (arg == "--lang") config.language = next("--lang");
        else if (arg == "--light") config.lightTheme = true;
        else if (arg == "--paused") config.startPaused = true;
        else if (arg == "--no-vsync") config.vsync = false;
        else if (arg == "--screenshot") config.screenshotPath = next("--screenshot");
        else if (arg == "--frames") config.screenshotFrames = std::max(1, std::atoi(next("--frames")));
        else if (arg == "--size") {
            config.width = std::atoi(next("--size"));
            config.height = std::atoi(next("--size"));
        } else if (arg == "--list-scenes") {
            for (const auto& s : kinema::scene::GetScenes()) std::printf("%-10s %s\n", s.id.c_str(), s.name.c_str());
            return 0;
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return 0;
        } else {
            std::fprintf(stderr, "Unknown option: %s\n\n", arg.c_str());
            PrintUsage();
            return 2;
        }
    }

    app::Application application(config);
    return application.Run();
}
