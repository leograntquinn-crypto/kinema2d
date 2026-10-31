// =============================================================================
// 国际化（i18n）：界面支持 English / 简体中文 切换
//
// 用法：ImGui::Button(Tr("Reset"))
//   * 英文原文本身就是查找键，代码里读起来仍然自然；
//   * 找不到翻译时直接返回原文，永远不会显示空字符串。
//
// 注意：ImGui 用标签计算控件 ID，切换语言会改变标签。
// 所以窗口名统一写成  Tr("Inspector") + "###Inspector"  —— "###" 之后的部分才是 ID。
// =============================================================================
#pragma once

#include <string>
#include <vector>

namespace app {

enum class Language { English = 0, Chinese = 1 };

void SetLanguage(Language lang);
Language GetLanguage();

/// 翻译（返回的指针在程序生命周期内有效）
const char* Tr(const char* english);

/// 生成带稳定 ID 的窗口/控件名：  "<翻译>###<id>"
std::string TrId(const char* english, const char* id);

/// 所有中文译文（用于构建字体字形范围，保证每个汉字都能显示）
std::vector<const char*> AllTranslations();

}  // namespace app
