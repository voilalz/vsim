// vsim/common/config.hpp —— 与中间件无关的键值配置
//
// 所有模块的参数默认值只在核心库里定义一次：
//   * 独立程序从 INI 文件 / 命令行 key=value 读入；
//   * ROS 节点把 ROS 参数（点分层级，如 controller.kp）逐项转成字符串写进 Config。
// 数组写成 "1.0, 2.0" 或 "[1.0, 2.0]"；矩阵行之间用 ';' 分隔。
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "vsim/common/status.hpp"

namespace vsim {

class Config {
 public:
  Config() = default;

  /// 读取 INI 文件：`[section]` 下的键会加上 "section." 前缀；# 或 ; 开头为注释。
  static Config from_file(const std::string& path, Status* status = nullptr);
  Status merge_file(const std::string& path);
  /// 解析一段 INI 文本（测试与内嵌默认配置用）。
  Status merge_string(const std::string& text);

  void set(const std::string& key, const std::string& value);
  /// 形如 "key=value" 的覆盖项（命令行 --set）。
  bool apply_override(const std::string& key_equals_value);
  void merge(const Config& other);  // other 中的键覆盖本对象

  bool has(const std::string& key) const;
  std::string get_string(const std::string& key, const std::string& def) const;
  double get_double(const std::string& key, double def) const;
  std::int64_t get_int(const std::string& key, std::int64_t def) const;
  bool get_bool(const std::string& key, bool def) const;
  std::vector<double> get_doubles(const std::string& key, const std::vector<double>& def) const;
  std::vector<std::int64_t> get_ints(const std::string& key, const std::vector<std::int64_t>& def) const;
  std::vector<std::string> get_strings(const std::string& key, const std::vector<std::string>& def) const;
  /// 每关节参数：配置里写 1 个值时广播到 n 个关节，写 n 个值时逐个使用；否则回退默认值并告警。
  std::vector<double> get_doubles_n(const std::string& key, std::size_t n, double def) const;
  /// 矩阵（如路径点）："0,0; 0.8,-0.5; -0.6,0.9"。
  std::vector<std::vector<double>> get_matrix(const std::string& key,
                                              const std::vector<std::vector<double>>& def) const;

  /// 取出 "prefix." 开头的子配置（去掉前缀）。
  Config sub(const std::string& prefix) const;
  const std::map<std::string, std::string>& entries() const { return kv_; }
  std::string dump() const;

 private:
  std::map<std::string, std::string> kv_;
};

// 字符串解析工具（也供 ROS 层复用）
std::string trim(const std::string& s);
std::vector<std::string> split(const std::string& s, char sep);
bool parse_double(const std::string& s, double& out);
bool parse_int(const std::string& s, std::int64_t& out);
bool parse_bool(const std::string& s, bool& out);
std::string join_doubles(const std::vector<double>& v);

}  // namespace vsim
