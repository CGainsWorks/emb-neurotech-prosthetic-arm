// Just enough JSON writing for the command line tools.
//
// The tools exist so that every number quoted in the documentation comes out
// of a program rather than out of somebody's head, which means their output
// has to be machine readable. A whole JSON library for that would be a
// dependency on the critical path of the build for the sake of writing
// commas in the right places.
#pragma once

#include <cstdio>
#include <string>
#include <vector>

namespace tools {

// Round-trip-exact formatting. %.17g is ugly and it is the only precision at
// which a double survives being written and read back.
inline std::string number(double v) {
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.17g", v);
    return buffer;
}

inline std::string array(const std::vector<double>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) out += ", ";
        out += number(values[i]);
    }
    return out + "]";
}

inline std::string quote(const std::string& s) { return "\"" + s + "\""; }

// A tiny object builder that keeps the emitted JSON indented and comma-correct
// without anybody having to track state at the call site.
class Object {
  public:
    explicit Object(int indent = 0) : indent_(indent) {}

    void field(const std::string& key, const std::string& raw_value) {
        if (!first_) body_ += ",\n";
        body_ += pad(indent_ + 2) + quote(key) + ": " + raw_value;
        first_ = false;
    }
    void string_field(const std::string& key, const std::string& value) {
        field(key, quote(value));
    }
    void number_field(const std::string& key, double value) { field(key, number(value)); }
    void bool_field(const std::string& key, bool value) { field(key, value ? "true" : "false"); }

    std::string str() const { return "{\n" + body_ + "\n" + pad(indent_) + "}"; }

    static std::string pad(int n) { return std::string(static_cast<std::size_t>(n), ' '); }

  private:
    int indent_;
    bool first_{true};
    std::string body_;
};

}  // namespace tools
