#include "editor/Theme.h"

#include <filesystem>
#include <string>
#include <vector>

#include "util/I18n.h"
#include "util/Log.h"

namespace app::theme {

namespace {

ThemeMode g_mode = ThemeMode::Dark;
ViewportPalette g_palette;
bool g_hasCjk = false;
ImFont* g_largeFont = nullptr;

constexpr ImVec4 Rgb(int hex, float a = 1.0f) {
    return ImVec4(static_cast<float>((hex >> 16) & 0xFF) / 255.0f, static_cast<float>((hex >> 8) & 0xFF) / 255.0f,
                  static_cast<float>(hex & 0xFF) / 255.0f, a);
}

void ApplyDark(ImGuiStyle& s) {
    ImVec4* c = s.Colors;
    const ImVec4 accent = Rgb(0x4F8CFF);

    c[ImGuiCol_Text] = Rgb(0xE6E9EF);
    c[ImGuiCol_TextDisabled] = Rgb(0x7A8194);
    c[ImGuiCol_WindowBg] = Rgb(0x1A1D23);
    c[ImGuiCol_ChildBg] = Rgb(0x1A1D23, 0.0f);
    c[ImGuiCol_PopupBg] = Rgb(0x1F232B, 0.98f);
    c[ImGuiCol_Border] = Rgb(0x2E333D);
    c[ImGuiCol_BorderShadow] = Rgb(0x000000, 0.0f);
    c[ImGuiCol_FrameBg] = Rgb(0x262A33);
    c[ImGuiCol_FrameBgHovered] = Rgb(0x2F3440);
    c[ImGuiCol_FrameBgActive] = Rgb(0x363C4A);
    c[ImGuiCol_TitleBg] = Rgb(0x15171C);
    c[ImGuiCol_TitleBgActive] = Rgb(0x15171C);
    c[ImGuiCol_TitleBgCollapsed] = Rgb(0x15171C);
    c[ImGuiCol_MenuBarBg] = Rgb(0x15171C);
    c[ImGuiCol_ScrollbarBg] = Rgb(0x1A1D23, 0.0f);
    c[ImGuiCol_ScrollbarGrab] = Rgb(0x353B47);
    c[ImGuiCol_ScrollbarGrabHovered] = Rgb(0x424958);
    c[ImGuiCol_ScrollbarGrabActive] = Rgb(0x4E5668);
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = Rgb(0x6FA2FF);
    c[ImGuiCol_Button] = Rgb(0x2A2F39);
    c[ImGuiCol_ButtonHovered] = Rgb(0x353B48);
    c[ImGuiCol_ButtonActive] = Rgb(0x3F4656);
    c[ImGuiCol_Header] = Rgb(0x4F8CFF, 0.22f);
    c[ImGuiCol_HeaderHovered] = Rgb(0x4F8CFF, 0.32f);
    c[ImGuiCol_HeaderActive] = Rgb(0x4F8CFF, 0.45f);
    c[ImGuiCol_Separator] = Rgb(0x2E333D);
    c[ImGuiCol_SeparatorHovered] = Rgb(0x4F8CFF, 0.6f);
    c[ImGuiCol_SeparatorActive] = accent;
    c[ImGuiCol_ResizeGrip] = Rgb(0x4F8CFF, 0.15f);
    c[ImGuiCol_ResizeGripHovered] = Rgb(0x4F8CFF, 0.5f);
    c[ImGuiCol_ResizeGripActive] = accent;
    c[ImGuiCol_Tab] = Rgb(0x1F232A);
    c[ImGuiCol_TabHovered] = Rgb(0x2E3440);
    c[ImGuiCol_TabSelected] = Rgb(0x262B34);
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_TabDimmed] = Rgb(0x1A1D23);
    c[ImGuiCol_TabDimmedSelected] = Rgb(0x22262E);
    c[ImGuiCol_TabDimmedSelectedOverline] = Rgb(0x4F8CFF, 0.0f);
    c[ImGuiCol_DockingPreview] = Rgb(0x4F8CFF, 0.4f);
    c[ImGuiCol_DockingEmptyBg] = Rgb(0x121418);
    c[ImGuiCol_PlotLines] = accent;
    c[ImGuiCol_PlotLinesHovered] = Rgb(0xFFD166);
    c[ImGuiCol_PlotHistogram] = accent;
    c[ImGuiCol_PlotHistogramHovered] = Rgb(0xFFD166);
    c[ImGuiCol_TableHeaderBg] = Rgb(0x22262E);
    c[ImGuiCol_TableBorderStrong] = Rgb(0x2E333D);
    c[ImGuiCol_TableBorderLight] = Rgb(0x262A33);
    c[ImGuiCol_TableRowBg] = Rgb(0x000000, 0.0f);
    c[ImGuiCol_TableRowBgAlt] = Rgb(0xFFFFFF, 0.025f);
    c[ImGuiCol_TextSelectedBg] = Rgb(0x4F8CFF, 0.35f);
    c[ImGuiCol_DragDropTarget] = Rgb(0xFFD166);
    c[ImGuiCol_NavCursor] = accent;
    c[ImGuiCol_ModalWindowDimBg] = Rgb(0x000000, 0.55f);

    g_palette.background = Color::Hex(0x12151B);
    g_palette.gridMinor = Color::Hex(0xFFFFFF, 10);
    g_palette.gridMajor = Color::Hex(0xFFFFFF, 24);
    g_palette.axisX = Color::Hex(0xFF6B6B, 90);
    g_palette.axisY = Color::Hex(0x06D6A0, 90);
    g_palette.staticFill = Color::Hex(0x3B4252);
    g_palette.selection = Color::Hex(0xFFB020);
    g_palette.hover = Color::Hex(0xFFFFFF, 200);
    g_palette.contactPoint = Color::Hex(0xFF4D6D);
    g_palette.contactNormal = Color::Hex(0xFFD166);
    g_palette.impulse = Color::Hex(0x06D6A0);
    g_palette.joint = Color::Hex(0x4CC9F0);
    g_palette.aabb = Color::Hex(0x06D6A0, 140);
    g_palette.velocity = Color::Hex(0xA78BFA);
    g_palette.toolPreview = Color::Hex(0xFFFFFF, 170);
}

void ApplyLight(ImGuiStyle& s) {
    ImGui::StyleColorsLight(&s);
    ImVec4* c = s.Colors;
    const ImVec4 accent = Rgb(0x2F6FEB);
    c[ImGuiCol_Text] = Rgb(0x1F2328);
    c[ImGuiCol_WindowBg] = Rgb(0xF6F7F9);
    c[ImGuiCol_PopupBg] = Rgb(0xFFFFFF, 0.98f);
    c[ImGuiCol_Border] = Rgb(0xD5D9E0);
    c[ImGuiCol_FrameBg] = Rgb(0xE9ECF1);
    c[ImGuiCol_FrameBgHovered] = Rgb(0xDDE3EC);
    c[ImGuiCol_FrameBgActive] = Rgb(0xD0D8E6);
    c[ImGuiCol_TitleBg] = Rgb(0xE9ECF1);
    c[ImGuiCol_TitleBgActive] = Rgb(0xE9ECF1);
    c[ImGuiCol_MenuBarBg] = Rgb(0xE9ECF1);
    c[ImGuiCol_Button] = Rgb(0xE2E6ED);
    c[ImGuiCol_ButtonHovered] = Rgb(0xD3DAE6);
    c[ImGuiCol_ButtonActive] = Rgb(0xC4CEDF);
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = Rgb(0x1F5AD1);
    c[ImGuiCol_Header] = Rgb(0x2F6FEB, 0.16f);
    c[ImGuiCol_HeaderHovered] = Rgb(0x2F6FEB, 0.26f);
    c[ImGuiCol_HeaderActive] = Rgb(0x2F6FEB, 0.36f);
    c[ImGuiCol_Tab] = Rgb(0xE3E7EE);
    c[ImGuiCol_TabHovered] = Rgb(0xD3DAE6);
    c[ImGuiCol_TabSelected] = Rgb(0xF6F7F9);
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_TabDimmed] = Rgb(0xE3E7EE);
    c[ImGuiCol_TabDimmedSelected] = Rgb(0xEEF0F4);
    c[ImGuiCol_DockingPreview] = Rgb(0x2F6FEB, 0.35f);
    c[ImGuiCol_PlotLines] = accent;
    c[ImGuiCol_PlotHistogram] = accent;

    g_palette.background = Color::Hex(0xF2F4F7);
    g_palette.gridMinor = Color::Hex(0x000000, 12);
    g_palette.gridMajor = Color::Hex(0x000000, 28);
    g_palette.axisX = Color::Hex(0xE5484D, 110);
    g_palette.axisY = Color::Hex(0x12A37F, 110);
    g_palette.staticFill = Color::Hex(0x9AA3B2);
    g_palette.selection = Color::Hex(0xF08C00);
    g_palette.hover = Color::Hex(0x000000, 160);
    g_palette.contactPoint = Color::Hex(0xE5484D);
    g_palette.contactNormal = Color::Hex(0xD99A00);
    g_palette.impulse = Color::Hex(0x12A37F);
    g_palette.joint = Color::Hex(0x0B84C6);
    g_palette.aabb = Color::Hex(0x12A37F, 160);
    g_palette.velocity = Color::Hex(0x7C5CDB);
    g_palette.toolPreview = Color::Hex(0x000000, 150);
}

}  // namespace

void Apply(ThemeMode mode, float dpiScale) {
    g_mode = mode;
    ImGuiStyle style;  // 从默认值开始，避免多次缩放叠加

    style.WindowPadding = ImVec2(10, 10);
    style.FramePadding = ImVec2(8, 5);
    style.CellPadding = ImVec2(6, 4);
    style.ItemSpacing = ImVec2(8, 6);
    style.ItemInnerSpacing = ImVec2(6, 4);
    style.IndentSpacing = 16;
    style.ScrollbarSize = 12;
    style.GrabMinSize = 10;

    style.WindowBorderSize = 1;
    style.ChildBorderSize = 1;
    style.PopupBorderSize = 1;
    style.FrameBorderSize = 0;
    style.TabBorderSize = 0;

    style.WindowRounding = 6;
    style.ChildRounding = 6;
    style.FrameRounding = 5;
    style.PopupRounding = 6;
    style.ScrollbarRounding = 8;
    style.GrabRounding = 4;
    style.TabRounding = 5;
    style.WindowMenuButtonPosition = ImGuiDir_None;
    style.SeparatorTextBorderSize = 1;
    style.DockingSeparatorSize = 2;

    if (mode == ThemeMode::Dark) ApplyDark(style);
    else ApplyLight(style);

    style.ScaleAllSizes(dpiScale);
    ImGui::GetStyle() = style;
}

ThemeMode Current() { return g_mode; }
const ViewportPalette& Palette() { return g_palette; }

ImVec4 Accent() { return g_mode == ThemeMode::Dark ? Rgb(0x4F8CFF) : Rgb(0x2F6FEB); }

ImU32 AccentU32(float alpha) {
    ImVec4 c = Accent();
    c.w = alpha;
    return ImGui::ColorConvertFloat4ToU32(c);
}

bool HasCjkFont() { return g_hasCjk; }
ImFont* LargeFont() { return g_largeFont; }

bool LoadFonts(float dpiScale) {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const float size = 16.0f * dpiScale;
    namespace fs = std::filesystem;

    auto firstExisting = [](const std::vector<const char*>& candidates) -> std::string {
        for (const char* p : candidates) {
            std::error_code ec;
            if (fs::exists(p, ec)) return p;
        }
        return {};
    };

    // 1) 西文字体：优先项目自带 assets/fonts，再找系统字体（仅 TrueType，stb_truetype 不支持 CFF/OTF）
    const std::string mainFont = firstExisting({
        "assets/fonts/Main.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    });

    // 2) 中文字体
    const std::string cjkFont = firstExisting({
        "assets/fonts/CJK.ttf",
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/simhei.ttf",
        "C:/Windows/Fonts/simsun.ttc",
        "/System/Library/Fonts/STHeiti Medium.ttc",
        "/System/Library/Fonts/Hiragino Sans GB.ttc",
        "/Library/Fonts/Arial Unicode.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/wqy-microhei/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    });

    // 只为实际用到的汉字生成字形：常用 2500 字 + 所有界面译文中的字
    static ImVector<ImWchar> cjkRanges;
    cjkRanges.clear();
    {
        ImFontGlyphRangesBuilder builder;
        builder.AddRanges(io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
        for (const char* text : AllTranslations()) builder.AddText(text);
        builder.BuildRanges(&cjkRanges);
    }

    auto addFontSet = [&](float px) -> ImFont* {
        ImFontConfig cfg;
        cfg.OversampleH = 2;
        cfg.OversampleV = 2;
        cfg.PixelSnapH = false;
        ImFont* font = nullptr;
        // 除基本拉丁字符外，再加入标点、箭头、数学符号、几何图形（● ▶ → ≈ 等）
        static const ImWchar kMainRanges[] = {0x0020, 0x00FF, 0x0370, 0x03FF, 0x2000, 0x206F, 0x2190, 0x21FF,
                                              0x2200, 0x22FF, 0x25A0, 0x25FF, 0};
        if (!mainFont.empty()) font = io.Fonts->AddFontFromFileTTF(mainFont.c_str(), px, &cfg, kMainRanges);
        if (!font) {
            ImFontConfig def;
            def.SizePixels = px;
            font = io.Fonts->AddFontDefault(&def);
        }
        if (!cjkFont.empty()) {
            ImFontConfig merge;
            merge.MergeMode = true;
            merge.OversampleH = 1;
            merge.OversampleV = 1;
            merge.PixelSnapH = true;
            if (io.Fonts->AddFontFromFileTTF(cjkFont.c_str(), px, &merge, cjkRanges.Data)) g_hasCjk = true;
        }
        return font;
    };

    g_hasCjk = false;
    addFontSet(size);
    g_largeFont = addFontSet(size * 1.35f);

    Log::Info("UI font: %s", mainFont.empty() ? "ImGui default" : mainFont.c_str());
    if (g_hasCjk) Log::Info("CJK font: %s", cjkFont.c_str());
    else Log::Warn("No CJK font found - Chinese UI disabled");
    return g_hasCjk;
}

}  // namespace app::theme
