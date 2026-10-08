#include "vsim/common/config.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "vsim/common/logging.hpp"

namespace vsim {
namespace {
constexpr const char* kTag = "config";

std::string strip_brackets(const std::string& s) {
  std::string t = trim(s);
  if (t.size() >= 2 && t.front() == '[' && t.back() == ']') t = t.substr(1, t.size() - 2);
  return t;
}

std::vector<std::string> split_list(const std::string& s) {
  // 逗号或空白分隔
  std::string t = strip_brackets(s);
  std::replace(t.begin(), t.end(), ',', ' ');
  std::istringstream is(t);
  std::vector<std::string> out;
  std::string tok;
  while (is >> tok) out.push_back(tok);
  return out;
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}
}  // namespace

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      out.push_back(trim(cur));
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(trim(cur));
  return out;
}

bool parse_double(const std::string& s, double& out) {
  const std::string t = trim(s);
  if (t.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(t.c_str(), &end);
  if (errno != 0 || end == t.c_str() || *end != '\0') return false;
  out = v;
  return true;
}

bool parse_int(const std::string& s, std::int64_t& out) {
  const std::string t = trim(s);
  if (t.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long long v = std::strtoll(t.c_str(), &end, 0);  // 支持 0x 前缀（CAN ID 等）
  if (errno != 0 || end == t.c_str() || *end != '\0') return false;
  out = static_cast<std::int64_t>(v);
  return true;
}

bool parse_bool(const std::string& s, bool& out) {
  const std::string t = lower(trim(s));
  if (t == "1" || t == "true" || t == "yes" || t == "on") {
    out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "off") {
    out = false;
    return true;
  }
  return false;
}

std::string join_doubles(const std::vector<double>& v) {
  std::ostringstream os;
  os.precision(17);
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) os << ",";
    os << v[i];
  }
  return os.str();
}

Config Config::from_file(const std::string& path, Status* status) {
  Config c;
  Status st = c.merge_file(path);
  if (status) *status = st;
  return c;
}

Status Config::merge_file(const std::string& path) {
  std::ifstream f(path);
  if (!f) return Status::InvalidArgument("cannot open config file: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return merge_string(ss.str());
}

Status Config::merge_string(const std::string& text) {
  std::istringstream is(text);
  std::string line;
  std::string section;
  int lineno = 0;
  while (std::getline(is, line)) {
    ++lineno;
    std::string t = trim(line);
    if (t.empty() || t[0] == '#' || t[0] == ';') continue;
    for (const char* marker : {" #", "\t#"}) {  // 行尾注释
      const auto pos = t.find(marker);
      if (pos != std::string::npos) t = trim(t.substr(0, pos));
    }
    if (t.front() == '[' && t.back() == ']') {
      section = trim(t.substr(1, t.size() - 2));
      continue;
    }
    const auto eq = t.find('=');
    if (eq == std::string::npos) {
      return Status::InvalidArgument("line " + std::to_string(lineno) + ": expected key = value");
    }
    std::string key = trim(t.substr(0, eq));
    std::string value = trim(t.substr(eq + 1));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }
    if (!section.empty()) key = section + "." + key;
    kv_[key] = value;
  }
  return Status::Ok();
}

void Config::set(const std::string& key, const std::string& value) { kv_[key] = value; }

bool Config::apply_override(const std::string& kv) {
  const auto eq = kv.find('=');
  if (eq == std::string::npos || eq == 0) return false;
  kv_[trim(kv.substr(0, eq))] = trim(kv.substr(eq + 1));
  return true;
}

void Config::merge(const Config& other) {
  for (const auto& [k, v] : other.kv_) kv_[k] = v;
}

bool Config::has(const std::string& key) const { return kv_.count(key) != 0; }

std::string Config::get_string(const std::string& key, const std::string& def) const {
  auto it = kv_.find(key);
  return it == kv_.end() ? def : it->second;
}

double Config::get_double(const std::string& key, double def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  double v = def;
  if (!parse_double(it->second, v)) {
    log_warn(kTag, "key '", key, "': cannot parse '", it->second, "' as number, using default ", def);
    return def;
  }
  return v;
}

std::int64_t Config::get_int(const std::string& key, std::int64_t def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  std::int64_t v = def;
  if (!parse_int(it->second, v)) {
    double d = 0.0;  // 允许 "10.0" 这类写法（ROS 参数常见）
    if (parse_double(it->second, d) && d == static_cast<double>(static_cast<std::int64_t>(d))) {
      return static_cast<std::int64_t>(d);
    }
    log_warn(kTag, "key '", key, "': cannot parse '", it->second, "' as integer, using default ", def);
    return def;
  }
  return v;
}

bool Config::get_bool(const std::string& key, bool def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  bool v = def;
  if (!parse_bool(it->second, v)) {
    log_warn(kTag, "key '", key, "': cannot parse '", it->second, "' as bool, using default");
    return def;
  }
  return v;
}

std::vector<double> Config::get_doubles(const std::string& key,
                                        const std::vector<double>& def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  std::vector<double> out;
  for (const auto& tok : split_list(it->second)) {
    double v = 0.0;
    if (!parse_double(tok, v)) {
      log_warn(kTag, "key '", key, "': bad number '", tok, "', using default");
      return def;
    }
    out.push_back(v);
  }
  return out;
}

std::vector<std::int64_t> Config::get_ints(const std::string& key,
                                           const std::vector<std::int64_t>& def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  std::vector<std::int64_t> out;
  for (const auto& tok : split_list(it->second)) {
    std::int64_t v = 0;
    if (!parse_int(tok, v)) {
      log_warn(kTag, "key '", key, "': bad integer '", tok, "', using default");
      return def;
    }
    out.push_back(v);
  }
  return out;
}

std::vector<std::string> Config::get_strings(const std::string& key,
                                             const std::vector<std::string>& def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  return split_list(it->second);
}

std::vector<double> Config::get_doubles_n(const std::string& key, std::size_t n, double def) const {
  std::vector<double> v = get_doubles(key, {});
  if (v.empty()) return std::vector<double>(n, def);
  if (v.size() == 1) return std::vector<double>(n, v[0]);
  if (v.size() == n) return v;
  log_warn(kTag, "key '", key, "': expected 1 or ", n, " values, got ", v.size(), "; using default");
  return std::vector<double>(n, def);
}

std::vector<std::vector<double>> Config::get_matrix(
    const std::string& key, const std::vector<std::vector<double>>& def) const {
  auto it = kv_.find(key);
  if (it == kv_.end()) return def;
  std::vector<std::vector<double>> rows;
  for (const auto& row : split(strip_brackets(it->second), ';')) {
    if (trim(row).empty()) continue;
    std::vector<double> r;
    for (const auto& tok : split_list(row)) {
      double v = 0.0;
      if (!parse_double(tok, v)) {
        log_warn(kTag, "key '", key, "': bad matrix entry '", tok, "', using default");
        return def;
      }
      r.push_back(v);
    }
    rows.push_back(std::move(r));
  }
  return rows;
}

Config Config::sub(const std::string& prefix) const {
  Config c;
  const std::string p = prefix.empty() ? "" : prefix + ".";
  for (const auto& [k, v] : kv_) {
    if (k.compare(0, p.size(), p) == 0) c.kv_[k.substr(p.size())] = v;
  }
  return c;
}

std::string Config::dump() const {
  std::ostringstream os;
  for (const auto& [k, v] : kv_) os << k << " = " << v << "\n";
  return os.str();
}

}  // namespace vsim
