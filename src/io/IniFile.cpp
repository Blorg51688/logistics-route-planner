#include "io/IniFile.h"

#include <fstream>
#include <sstream>

#include "io/StringUtil.h"

namespace logistics {

const IniRecord* IniSection::findValue(const std::string& key) const {
    for (const IniRecord& r : records) {
        if (r.isKeyValue && r.key == key) {
            return &r;
        }
    }
    return nullptr;
}

namespace {

void fail(std::string& error, int lineNo, const std::string& msg) {
    error = "第 " + std::to_string(lineNo) + " 行: " + msg;
}

} // namespace

bool IniFile::parse(const std::string& text, IniFile& out, std::string& error) {
    out.sections_.clear();

    std::istringstream in(text);
    std::string raw;
    int lineNo = 0;
    IniSection* current = nullptr;

    while (std::getline(in, raw)) {
        ++lineNo;

        // 去注释后按空白裁剪
        const std::size_t hash = raw.find('#');
        const std::string line = trim(hash == std::string::npos ? raw : raw.substr(0, hash));
        if (line.empty()) {
            continue;
        }

        if (line.front() == '[') {
            if (line.back() != ']') {
                fail(error, lineNo, "[section] 缺少右括号: " + line);
                return false;
            }
            const std::string name = trim(line.substr(1, line.size() - 2));
            if (name.empty()) {
                fail(error, lineNo, "[section] 名为空");
                return false;
            }
            if (out.findSection(name) != nullptr) {
                fail(error, lineNo, "重复的 [section]: " + name);
                return false;
            }
            out.sections_.push_back(IniSection{name, {}});
            current = &out.sections_.back();
            continue;
        }

        if (current == nullptr) {
            fail(error, lineNo, "配置项出现在任何 [section] 之前: " + line);
            return false;
        }

        IniRecord rec;
        rec.lineNo = lineNo;
        const std::size_t eq = line.find('=');
        if (eq != std::string::npos) {
            rec.isKeyValue = true;
            rec.key = trim(line.substr(0, eq));
            rec.value = trim(line.substr(eq + 1));
            if (rec.key.empty()) {
                fail(error, lineNo, "配置项名为空: " + line);
                return false;
            }
        } else {
            rec.isKeyValue = false;
            rec.text = line;
        }
        current->records.push_back(rec);
    }
    return true;
}

bool IniFile::loadFromFile(const std::string& path, IniFile& out, std::string& error) {
    std::ifstream in(path.c_str());
    if (!in) {
        error = "无法打开配置文件: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return parse(buffer.str(), out, error);
}

const IniSection* IniFile::findSection(const std::string& name) const {
    for (const IniSection& s : sections_) {
        if (s.name == name) {
            return &s;
        }
    }
    return nullptr;
}

} // namespace logistics
