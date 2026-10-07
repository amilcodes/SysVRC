// visbot/json.hpp — just enough JSON to read a robot.json.
//
// Header-only, no dependencies, like the rest of core/. Objects keep their key
// order; numbers are doubles. Not fast and not meant to be: the files it reads
// are a few kilobytes.
#pragma once

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace visbot {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    double number(double fallback = 0.0) const { return type_ == Type::Number ? num_ : fallback; }
    bool boolean(bool fallback = false) const { return type_ == Type::Bool ? bool_ : fallback; }
    const std::string& string() const { return str_; }
    const std::vector<Json>& items() const { return items_; }
    const std::vector<std::pair<std::string, Json>>& members() const { return members_; }

    /// Member lookup; a null value if this isn't an object or has no such key.
    const Json& operator[](const std::string& key) const {
        for (const auto& [k, v] : members_)
            if (k == key) return v;
        return null();
    }
    bool has(const std::string& key) const {
        for (const auto& kv : members_)
            if (kv.first == key) return true;
        return false;
    }

    /// Parse a document. On failure returns null and sets `error` (with the
    /// byte offset), so a bad robot.json says where it's bad.
    static Json parse(const std::string& text, std::string* error = nullptr) {
        Parser p{text, 0, {}};
        Json v = p.value();
        p.ws();
        if (p.err.empty() && p.i != text.size()) p.fail("trailing characters");
        if (!p.err.empty()) {
            if (error) *error = p.err;
            return Json{};
        }
        if (error) error->clear();
        return v;
    }

private:
    static const Json& null() { static const Json n; return n; }

    struct Parser {
        const std::string& s;
        size_t i;
        std::string err;

        void fail(const char* what) {
            if (err.empty()) err = std::string(what) + " at byte " + std::to_string(i);
        }
        void ws() { while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i; }
        bool lit(const char* w) {
            size_t n = 0;
            while (w[n]) ++n;
            if (s.compare(i, n, w) != 0) return false;
            i += n;
            return true;
        }

        Json value(int depth = 0) {
            Json v;
            if (depth > 64) { fail("nested too deep"); return v; }
            ws();
            if (i >= s.size()) { fail("unexpected end"); return v; }
            const char c = s[i];
            if (c == '{') return object(depth);
            if (c == '[') return array(depth);
            if (c == '"') { v.type_ = Type::String; v.str_ = str(); return v; }
            if (lit("true")) { v.type_ = Type::Bool; v.bool_ = true; return v; }
            if (lit("false")) { v.type_ = Type::Bool; v.bool_ = false; return v; }
            if (lit("null")) return v;
            if (c == '-' || (c >= '0' && c <= '9')) {
                const char* start = s.c_str() + i;
                char* end = nullptr;
                v.num_ = std::strtod(start, &end);
                if (end == start) { fail("bad number"); return v; }
                i += static_cast<size_t>(end - start);
                v.type_ = Type::Number;
                return v;
            }
            fail("unexpected character");
            return v;
        }

        Json object(int depth) {
            Json v;
            v.type_ = Type::Object;
            ++i;  // {
            ws();
            if (i < s.size() && s[i] == '}') { ++i; return v; }
            while (err.empty()) {
                ws();
                if (i >= s.size() || s[i] != '"') { fail("expected a key"); break; }
                std::string k = str();
                ws();
                if (i >= s.size() || s[i] != ':') { fail("expected ':'"); break; }
                ++i;
                v.members_.emplace_back(std::move(k), value(depth + 1));
                ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; break; }
                fail("expected ',' or '}'");
            }
            return v;
        }

        Json array(int depth) {
            Json v;
            v.type_ = Type::Array;
            ++i;  // [
            ws();
            if (i < s.size() && s[i] == ']') { ++i; return v; }
            while (err.empty()) {
                v.items_.push_back(value(depth + 1));
                ws();
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; break; }
                fail("expected ',' or ']'");
            }
            return v;
        }

        std::string str() {
            std::string out;
            ++i;  // opening quote
            while (i < s.size() && s[i] != '"') {
                char c = s[i++];
                if (c != '\\') { out += c; continue; }
                if (i >= s.size()) break;
                const char e = s[i++];
                switch (e) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'u': {
                        if (i + 4 > s.size()) { fail("bad \\u escape"); return out; }
                        const unsigned cp = static_cast<unsigned>(std::strtoul(s.substr(i, 4).c_str(), nullptr, 16));
                        i += 4;
                        // BMP only; robot.json is ASCII in practice
                        if (cp < 0x80) out += static_cast<char>(cp);
                        else if (cp < 0x800) { out += static_cast<char>(0xC0 | (cp >> 6)); out += static_cast<char>(0x80 | (cp & 0x3F)); }
                        else { out += static_cast<char>(0xE0 | (cp >> 12)); out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                               out += static_cast<char>(0x80 | (cp & 0x3F)); }
                        break;
                    }
                    default: out += e;  // \" \\ \/
                }
            }
            if (i >= s.size()) { fail("unterminated string"); return out; }
            ++i;  // closing quote
            return out;
        }
    };

    Type type_ = Type::Null;
    bool bool_ = false;
    double num_ = 0.0;
    std::string str_;
    std::vector<Json> items_;
    std::vector<std::pair<std::string, Json>> members_;
};

}  // namespace visbot
