#pragma once

#include "json.hpp"
#include <algorithm>
#include <cstdarg>
#include <exception>
#include <map>
#include <regex>
#include <stack>
#include <string>

namespace mjson {

/// Declaration
template <class T> struct Matrix {
  std::vector<T> data;
  std::vector<int> shape;
};

class Reader {
public:
  Reader() = default;
  explicit Reader(const Json &json, std::string hint_prefix = "")
      : json_(json), hint_prefix_(hint_prefix){};
  explicit Reader(const std::string &json_string, std::string hint_prefix = "");

  template <typename T>
  T get(const std::string &key_path, bool is_require = true,
        const T &default_value = T{}) const;

  void override_value(std::string key_path, mjson::Json value);
  void override_value(std::string key_path, std::string value);
  void
  set_override_param(const std::map<std::string, std::string> &parameters) {
    for (auto item : parameters) {
      override_value(item.first, item.second);
    }
  }
  std::vector<std::string> get_json_keys() const {
    std::vector<std::string> result;
    auto object_value = json_.object_value();
    for (auto i : object_value) {
      result.push_back(i.first);
    }
    return result;
  }
  std::map<std::string, std::string> expand_environment_varialbe();

  const Json &raw() const { return json_; }

private:
  template <typename T> struct Type {};

  void check_or_error(bool condition, const char *format, ...) const;

  template <typename T>
  T get_json_value(const mjson::Json *json, const std::string &key,
                   Type<T>) const;

  template <typename T>
  Matrix<T> get_json_value(const mjson::Json *json, const std::string &key,
                           Type<Matrix<T>>) const;

  template <typename T>
  std::vector<T> get_json_value(const mjson::Json *json, const std::string &key,
                                Type<std::vector<T>>) const;

  template <typename T>
  std::map<std::string, T> get_json_value(const mjson::Json *json,
                                          const std::string &key,
                                          Type<std::map<std::string, T>>) const;

  Json json_;
  std::string hint_prefix_;
};

class KeySpliter {
public:
  enum class IndicatorType {
    INVALID,
    ARRAY_INDEX,
    OBJECT_KEY,
  };

  struct Indicator {
    IndicatorType type;
    std::string key;
    int index;
  };

  KeySpliter(std::string key) : key_(key){};

  bool is_end() { return iter_ == key_.length(); }

  Indicator fetch();

private:
  enum class State {
    RECEIVING_ARRAY_INDEX,
    RECEIVING_OBJECT_KEY,
    RESTART,
  };

  size_t start_ = 0;
  size_t end_ = 0;
  size_t iter_ = 0;
  std::string key_;
  State state_ = State::RESTART;
};

class Exception : public std::exception {
public:
  Exception(std::string msg) : msg_(msg){};
  const char *what() const noexcept { return msg_.c_str(); };

private:
  std::string msg_;
};

/// Implementation

template <typename T>
inline T mjson::Reader::get(const std::string &key_path, bool is_require,
                            const T &default_value) const {
  auto spliter = KeySpliter(key_path);
  const Json *p = &json_;
  for (auto indicator = spliter.fetch();
       indicator.type != KeySpliter::IndicatorType::INVALID;
       indicator = spliter.fetch()) {
    if (indicator.type == KeySpliter::IndicatorType::ARRAY_INDEX) {
      p = &((*p)[indicator.index]);
    } else if (indicator.type == KeySpliter::IndicatorType::OBJECT_KEY) {
      p = &((*p)[indicator.key]);
    }
    if (p->is_null())
      break;
  }

  check_or_error(!is_require || (is_require && !p->is_null()),
                 "key %s is not found in json.", key_path.c_str());

  if (!p->is_null()) {
    return get_json_value(p, key_path, Type<T>{});
  } else {
    return default_value;
  }
}

inline std::map<std::string, std::string>
mjson::Reader::expand_environment_varialbe() {
  std::string raw = this->raw().dump();
  std::string convert_json{};
  std::smatch match{};
  std::regex pattern("\\$\\{(\\w+)\\}");

  std::map<std::string, std::string> variable_dict;
  while (std::regex_search(raw, match, pattern)) {
    convert_json += match.prefix().str();
    assert(match.size() >= 1);
    auto environment_variable = match[1].str();

    auto env = std::getenv(environment_variable.c_str());
    auto value = env == nullptr ? "" : env;
    variable_dict[environment_variable] = value;
    convert_json += value;
    raw = match.suffix().str();
  }
  convert_json += raw;

  std::string error;
  auto json = Json::parse(convert_json, error);
  if (error != "")
    throw Exception(error);
  json_ = json;
  return variable_dict;
}

inline void mjson::Reader::override_value(std::string key_path,
                                          std::string value) {
  std::string error;
  auto target_value = mjson::Json::parse(value, error);
  if (error != "") {
    target_value = mjson::Json(value);
  }
  override_value(key_path, target_value);
}

inline void mjson::Reader::override_value(std::string key_path,
                                          mjson::Json value) {
  auto spliter = KeySpliter(key_path);
  Json *p = &json_;
  std::string prefix = "";
  for (auto indicator = spliter.fetch();
       indicator.type != KeySpliter::IndicatorType::INVALID;
       indicator = spliter.fetch()) {
    if (indicator.type == KeySpliter::IndicatorType::ARRAY_INDEX) {
      prefix += "[" + std::to_string(indicator.index) + "]";
      check_or_error(p->is_array() &&
                         static_cast<int>(p->array_value().size()) >
                             indicator.index &&
                         indicator.index >= 0,
                     "when overriding key '%s', fail in routing '%s'",
                     key_path.c_str(), prefix.c_str());
      p = &(p->ref_element(indicator.index));
    } else if (indicator.type == KeySpliter::IndicatorType::OBJECT_KEY) {
      prefix += (prefix.empty() ? "" : ".") + indicator.key;
      check_or_error(p->is_object(),
                     "when overriding key '%s', fail in routing '%s'",
                     key_path.c_str(), prefix.c_str());
      if (!p->has_key(indicator.key))
        p->ref_element(indicator.key) = mjson::Json(mjson::Json::object());
      p = &(p->ref_element(indicator.key));
    }
  }
  *p = value;
}

inline void mjson::Reader::check_or_error(bool condition, const char *format,
                                          ...) const {
  if (!condition) {
    constexpr int BUFFER_LENGTH = 2000;
    char buffer[BUFFER_LENGTH];
    va_list args;
    va_start(args, format);
    auto size = vsnprintf(buffer, BUFFER_LENGTH - 1, format, args);
    va_end(args);
    std::string msg(buffer, buffer + size);
    auto complete_msg = hint_prefix_.empty() ? msg : hint_prefix_ + ": " + msg;
    throw Exception(complete_msg);
  }
}

inline mjson::Reader::Reader(const std::string &json_string,
                             std::string hint_prefix) {
  std::string error;
  auto json = Json::parse(json_string, error);
  if (error != "")
    throw Exception(error);
  json_ = json;
  hint_prefix_ = hint_prefix;
};

template <>
inline int mjson::Reader::get_json_value<int>(const mjson::Json *json,
                                              const std::string &key_name,
                                              Type<int>) const {
  check_or_error(json->is_int(), "value of '%s' in json is not interger.",
                 key_name.c_str());
  return json->int_value();
}

template <>
inline std::string
mjson::Reader::get_json_value<std::string>(const mjson::Json *json,
                                           const std::string &key_name,
                                           Type<std::string>) const {
  if (json->is_string())
    return json->string_value();
  else
    return json->dump();
}
template <>
inline float mjson::Reader::get_json_value<float>(const mjson::Json *json,
                                                  const std::string &key_name,
                                                  Type<float>) const {
  check_or_error(json->is_number(), "value of '%s' in json is not number.",
                 key_name.c_str());
  return json->number_value();
}

template <>
inline double mjson::Reader::get_json_value<double>(const mjson::Json *json,
                                                    const std::string &key_name,
                                                    Type<double>) const {
  check_or_error(json->is_number(), "value of '%s' in json is not number.",
                 key_name.c_str());
  return json->number_value();
}

template <>
inline bool mjson::Reader::get_json_value<bool>(const mjson::Json *json,
                                                const std::string &key_name,
                                                Type<bool>) const {
  check_or_error(json->is_bool(), "value of '%s' in json is not bool.",
                 key_name.c_str());
  return json->bool_value();
}
template <>
inline mjson::Json
mjson::Reader::get_json_value<mjson::Json>(const mjson::Json *json,
                                           const std::string &key_name,
                                           Type<mjson::Json>) const {
  return *json;
}

template <typename T>
inline std::map<std::string, T>
mjson::Reader::get_json_value(const mjson::Json *json,
                              const std::string &key_name,
                              Type<std::map<std::string, T>>) const {
  check_or_error(json->is_object(), "value of '%s' in json is not array.",
                 key_name.c_str());
  std::map<std::string, T> result;
  const auto &items = json->object_value();
  for (const auto &pair : items) {
    std::string sub_key = key_name + "." + pair.first;
    result[pair.first] = get_json_value(&(pair.second), sub_key, Type<T>{});
  }
  return result;
}

template <typename T>
inline std::vector<T>
mjson::Reader::get_json_value(const mjson::Json *json,
                              const std::string &key_name,
                              Type<std::vector<T>>) const {
  check_or_error(json->is_array(), "value of '%s' in json is not array.",
                 key_name.c_str());
  std::vector<T> result;
  auto &items = json->array_value();
  for (size_t i = 0, n = items.size(); i < n; i++) {
    std::string sub_key = key_name + "[" + std::to_string(i) + "]";
    result.push_back(get_json_value(&(items[i]), sub_key, Type<T>{}));
  }
  return result;
}

template <typename T>
Matrix<T> mjson::Reader::get_json_value(const mjson::Json *json,
                                        const std::string &key_name,
                                        Type<Matrix<T>>) const {
  check_or_error(json->is_array(), "value of '%s' in json is not array.",
                 key_name.c_str());
  Matrix<T> result{};
  const mjson::Json *p = json;
  while (p->is_array()) {
    const mjson::Json::array &array = p->array_value();
    result.shape.push_back(array.size());
    if (array.size() == 0)
      break;
    p = &(array[0]);
  }

  class MatrixParser {
  public:
    MatrixParser(const mjson::Reader *reader) : reader_(reader) {}
    int parse(Matrix<T> &mat, int depth, const mjson::Json &json,
              const std::string &hint) {
      if (depth == mat.shape.size()) {
        auto data = reader_->get_json_value(&json, hint, Type<T>{});
        mat.data.push_back(std::move(data));
        return 1;
      }

      reader_->check_or_error(json.is_array(),
                              "fail to parse '%s' as an array.", hint.c_str());
      const mjson::Json::array &array = json.array_value();
      for (size_t i = 0; i < array.size(); ++i) {
        auto parse_size = parse(mat, depth + 1, array[i],
                                hint + "[" + std::to_string(i) + "]");
        if (depth + 1 < mat.shape.size())
          reader_->check_or_error(parse_size == mat.shape[depth + 1],
                                  "size of submatrices in '%s' must be aligned",
                                  hint.c_str());
      }
      return array.size();
    }

  private:
    const mjson::Reader *reader_;
  };

  MatrixParser parser(this);
  parser.parse(result, 0, *json, key_name);
  return result;
}

inline mjson::KeySpliter::Indicator mjson::KeySpliter::fetch() {
  Indicator ret{.type = IndicatorType::INVALID};
  if (iter_ > key_.size())
    return ret;

  while (iter_ <= key_.size()) {

    switch (state_) {
    case State::RECEIVING_ARRAY_INDEX:
      if (iter_ == key_.size()) {
        // error and end, return INVALID
      } else if (key_[iter_] == ']') {
        end_ = iter_;
        ret = {.type = IndicatorType::ARRAY_INDEX,
               .key = "",
               .index = std::stoi(key_.substr(start_, end_ - start_))};
        state_ = State::RESTART;
      } else {
        // keep state, continue
      }
      break;
    case State::RECEIVING_OBJECT_KEY: {
      if (iter_ == key_.size() || key_[iter_] == '.') {
        end_ = iter_;
        ret = {.type = IndicatorType::OBJECT_KEY,
               .key = key_.substr(start_, end_ - start_)};
        state_ = State::RESTART;
      } else if (key_[iter_] == '[') {
        end_ = iter_;
        ret = {.type = IndicatorType::OBJECT_KEY,
               .key = key_.substr(start_, end_ - start_)};
        start_ = iter_ + 1;
        state_ = State::RECEIVING_ARRAY_INDEX;
      } else {
        // keep state, continue
      }

      break;
    }
    case State::RESTART: {
      if (iter_ == key_.size()) {
        // end, return INVALID
      } else if (key_[iter_] == '.') {
        // ignore and continue
      } else if (key_[iter_] == '[') {
        start_ = iter_ + 1;
        state_ = State::RECEIVING_ARRAY_INDEX;
      } else {
        start_ = iter_;
        state_ = State::RECEIVING_OBJECT_KEY;
      }
      break;
    }
    }
    ++iter_;
    if (ret.type != IndicatorType::INVALID)
      break;
  }
  return ret;
}

} // namespace mjson
