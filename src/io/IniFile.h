#pragma once

#include <string>
#include <vector>

namespace logistics {

// INI 中的一行。
// - [general] 这类节用 key = value 形式，isKeyValue 为 true；
// - [nodes] / [edges] / [vehicles] / [orders] 用逗号分隔的记录行，isKeyValue 为 false。
struct IniRecord {
    int         lineNo = 0;
    bool        isKeyValue = false;
    std::string key;
    std::string value;
    std::string text;   // 记录行的内容（已去注释与首尾空白）
};

struct IniSection {
    std::string            name;
    std::vector<IniRecord> records;

    // 取 key = value 形式的配置项；未命中返回 nullptr。
    // 返回记录本体而非值，便于报错时携带行号。
    const IniRecord* findValue(const std::string& key) const;
};

// 手写 INI 解析器，零第三方依赖。
// 规则：# 起为注释、[section] 分节、空行忽略。
// 所有错误都带行号，便于定位配置问题。
class IniFile {
public:
    static bool parse(const std::string& text, IniFile& out, std::string& error);
    static bool loadFromFile(const std::string& path, IniFile& out, std::string& error);

    const IniSection* findSection(const std::string& name) const;

private:
    std::vector<IniSection> sections_;
};

} // namespace logistics
