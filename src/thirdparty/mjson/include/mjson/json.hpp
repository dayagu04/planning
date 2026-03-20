#pragma once

#include <assert.h>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mjson {

class JsonValue;

class Json {
public:
  enum class JsonValueType {
    JSON_VALUE_NULL,
    JSON_VALUE_DOUBLE,
    JSON_VALUE_INT,
    JSON_VALUE_BOOL,
    JSON_VALUE_STRING,
    JSON_VALUE_ARRAY,
    JSON_VALUE_OBJECT,
  };

  // define json data type of array
  typedef std::vector<Json> array;
  // define json data type of object
  typedef std::map<std::string, Json> object;

  /* Constructors for the various types of json value:
     null
     a number
     a boolean
     a string
     an array
     an object (JSON Object)
  */
  explicit Json();
  explicit Json(double value);
  explicit Json(int value);
  explicit Json(bool value);
  explicit Json(const char *value);
  explicit Json(const std::string &value);
  explicit Json(const array &value);
  explicit Json(const object &value);
  Json(const Json &value) { *this = value; };
  Json(Json &&value) : value_ptr_(std::move(value.value_ptr_)){};

  Json &operator=(double value) { return *this = Json(value); }
  Json &operator=(int value) { return *this = Json(value); }
  Json &operator=(bool value) { return *this = Json(value); }
  Json &operator=(const char *value) { return *this = Json(value); };
  Json &operator=(const std::string &value) { return *this = Json(value); };
  Json &operator=(const array &value) { return *this = Json(value); };
  Json &operator=(const object &value) { return *this = Json(value); };
  Json &operator=(const Json &value);

  // Access different json data types of value
  double number_value() const;
  double &number_value();

  int int_value() const;
  int &int_value();

  bool bool_value() const;
  bool &bool_value();

  const std::string &string_value() const;
  std::string &string_value();

  const array &array_value() const;
  array &array_value();

  const object &object_value() const;
  object &object_value();

  const Json &operator[](size_t index) const;
  Json &operator[](size_t index);

  const Json &operator[](const std::string &key) const;
  Json &operator[](const std::string &key);

  Json &ref_element(const std::string &key);
  Json &ref_element(size_t index);

  bool has_key(const std::string &key) const;
  bool is_null() const;
  bool is_number() const;
  bool is_int() const;
  bool is_bool() const;
  bool is_string() const;
  bool is_array() const;
  bool is_object() const;

  JsonValueType type() const;

  // Methods of parser
  // conver Json to std::string
  std::string dump() const;
  void dump(std::string &) const;

  // parse string(std::string) to Json
  static Json parse(const std::string &input, std::string &error_info);
  static Json parse(const char *input, std::string &error_info) {
    return parse(std::string(input), error_info);
  }

  inline bool operator==(double other) const {
    if (!is_number()) {
      return false;
    }
    return number_value() == other;
  }

  inline bool operator==(int other) const {
    if (!is_int()) {
      return false;
    }
    return int_value() == other;
  }

  inline bool operator==(bool other) const {
    if (!is_bool()) {
      return false;
    }
    return bool_value() == other;
  }

  inline bool operator==(const std::string &other) const {
    if (!is_string()) {
      return false;
    }
    return string_value() == other;
  }

private:
  std::shared_ptr<JsonValue> value_ptr_;
};

inline bool operator==(const Json &lhs, const Json &rhs) {
  if (lhs.type() != rhs.type()) {
    return false;
  }

  auto type = lhs.type();
  switch (type) {
  case Json::JsonValueType::JSON_VALUE_NULL:
    return false;
  case Json::JsonValueType::JSON_VALUE_DOUBLE:
    return lhs.number_value() == rhs.number_value();
  case Json::JsonValueType::JSON_VALUE_INT:
    return lhs.int_value() == rhs.int_value();
  case Json::JsonValueType::JSON_VALUE_BOOL:
    return lhs.bool_value() == rhs.bool_value();
  case Json::JsonValueType::JSON_VALUE_STRING:
    return lhs.string_value() == rhs.string_value();
  case Json::JsonValueType::JSON_VALUE_ARRAY:
    return lhs.array_value() == rhs.array_value();
  case Json::JsonValueType::JSON_VALUE_OBJECT:
    return lhs.object_value() == rhs.object_value();
  default:
    return false;
  }
  return false;
}

inline bool operator!=(const Json &lhs, const Json &rhs) {
  return !(lhs == rhs);
}

static constexpr double ERROR_NUMBER_VALUE = 0;
static constexpr int ERROR_INT_VALUE = 0;
static constexpr bool ERROR_BOOL_VALUE = 0;
static const Json EMPTY_JSON_VALUE;
static const Json::array EMPTY_JSON_ARRAY;
static const Json::object EMPTY_JSON_OBJECT;
static const std::string EMPTY_JSON_STRING;

/*
  class JsonValue is used to define interfaces for different Json data types
  number_value() get a number (double)
  int_value() get a number (int)
  bool_value() get a boolean
  string_value() get a string(std::string)
  array_value() get a array
  object_value() get a object(std::map<std::string, Json>)
*/
class JsonValue {
public:
  virtual double number_value() const {
    assert(false);
    return 0;
  }
  virtual double &number_value() {
    assert(false);
    static double res = 0;
    return res;
  }
  virtual int int_value() const {
    assert(false);
    return 0;
  }
  virtual int &int_value() {
    assert(false);
    static int res = 0;
    return res;
  }
  virtual bool bool_value() const {
    assert(false);
    return false;
  }
  virtual bool &bool_value() {
    assert(false);
    static bool res = false;
    return res;
  }
  virtual const std::string &string_value() const {
    assert(false);
    return EMPTY_JSON_STRING;
  }
  virtual std::string &string_value() {
    assert(false);
    static std::string res;
    return res;
  }
  virtual const Json::array &array_value() const {
    assert(false);
    return EMPTY_JSON_ARRAY;
  }
  virtual Json::array &array_value() {
    assert(false);
    static Json::array res;
    return res;
  }
  virtual const Json::object &object_value() const {
    assert(false);
    return EMPTY_JSON_OBJECT;
  }
  virtual Json::object &object_value() {
    assert(false);
    static Json::object res;
    return res;
  }
  virtual const Json &operator[](size_t) const {
    assert(false);
    return EMPTY_JSON_VALUE;
  }
  virtual Json &operator[](size_t) {
    assert(false);
    static Json res;
    return res;
  }
  virtual const Json &operator[](const std::string &) const {
    assert(false);
    return EMPTY_JSON_VALUE;
  }
  virtual Json &operator[](const std::string &) {
    assert(false);
    static Json res;
    return res;
  }
  virtual bool has_key(const std::string &) const {
    assert(false);
    return false;
  }
  virtual Json &ref_element(const std::string &) {
    assert(false);
    static Json res;
    return res;
  }
  virtual Json &ref_element(size_t) {
    assert(false);
    static Json res;
    return res;
  }
  virtual Json::JsonValueType get_value_type() const {
    assert(false);
    return Json::JsonValueType::JSON_VALUE_NULL;
  }
  virtual void dump(std::string &) const { assert(false); }
  virtual ~JsonValue() {}
};
} // namespace mjson

namespace {

static constexpr size_t DOUBLE_TO_STRING_SIZE = 50;
static constexpr size_t INT_TO_STRING_SIZE = 30;

static constexpr auto TRUE_STRING = "true";
static constexpr auto FALSE_STRING = "false";
static constexpr auto NULL_STRING = "null";
static constexpr size_t TRUE_LENGTH = 4;
static constexpr size_t FALSE_LENGTH = 5;
static constexpr size_t NULL_LENGTH = 4;

static constexpr size_t ERROR_INDEX_DIGIT_LENGTH = 12;

#define MJSON_FAIL()                                                           \
  do {                                                                         \
    flag = false;                                                              \
    return mjson::Json();                                                      \
  } while (false)

#define IS_ADD_MJSON_FAIL()                                                    \
  do {                                                                         \
    if (index + 1 >= obj_string_size) {                                        \
      MJSON_FAIL();                                                            \
    }                                                                          \
  } while (false)

#define IS_NOW_MJSON_FAIL()                                                    \
  do {                                                                         \
    if (index >= obj_string_size) {                                            \
      MJSON_FAIL();                                                            \
    }                                                                          \
  } while (false)

#define IS_MJSON_FAIL()                                                        \
  do {                                                                         \
    if (!flag) {                                                               \
      return mjson::Json();                                                    \
    }                                                                          \
  } while (false)

class JsonUtil {
public:
  static inline bool is_hex(char ch) {
    return ('0' <= ch && ch <= '9') || ('a' <= ch && ch <= 'z') ||
           ('A' <= ch && ch <= 'Z');
  }
  static inline uint8_t from_hex(char ch) {
    if ('0' <= ch && ch <= '9')
      return ch - '0';
    if ('a' <= ch && ch <= 'z')
      return ch - 'a' + 10;
    if ('A' <= ch && ch <= 'Z')
      return ch - 'A' + 10;
    assert(false);
    abort();
  }

  static inline char to_hex(uint8_t digit) {
    assert(digit <= 16);
    if (0 <= digit && digit <= 9)
      return '0' + digit;
    return 'a' + digit - 10;
  }

  static inline std::string to_utf8(int32_t value) {
    if (value < 0)
      return "";
    std::string ans;
    if (value < 0x80) {
      ans.push_back(static_cast<char>(value));
    } else if (value < 0x800) {
      ans.push_back(static_cast<char>((value >> 6) | 0xC0));
      ans.push_back(static_cast<char>((value & 0x3F) | 0x80));
    } else if (value < 0x10000) {
      ans.push_back(static_cast<char>((value >> 12) | 0xE0));
      ans.push_back(static_cast<char>(((value >> 6) & 0x3F) | 0x80));
      ans.push_back(static_cast<char>((value & 0x3F) | 0x80));
    } else {
      ans.push_back(static_cast<char>((value >> 18) | 0xF0));
      ans.push_back(static_cast<char>(((value >> 12) & 0x3F) | 0x80));
      ans.push_back(static_cast<char>(((value >> 6) & 0x3F) | 0x80));
      ans.push_back(static_cast<char>((value & 0x3F) | 0x80));
    }
    return ans;
  }

  static inline std::string to_utf16_dump(uint32_t value) {
    if (value <= 0xFFFF) {
      return std::string("\\u") + to_hex((value & 0xF000) >> 12) +
             to_hex((value & 0x0F00) >> 8) + to_hex((value & 0x00F0) >> 4) +
             to_hex((value & 0x000F));
    } else {
      value -= 0x10000;
      uint32_t high = ((value & 0xFFC00) >> 10) + 0xD800;
      uint32_t low = (value & 0x003FF) + 0xDC00;
      return std::string("\\u") + to_hex((high & 0xF000) >> 12) +
             to_hex((high & 0x0F00) >> 8) + to_hex((high & 0x00F0) >> 4) +
             to_hex((high & 0x000F)) + std::string("\\u") +
             to_hex((low & 0xF000) >> 12) + to_hex((low & 0x0F00) >> 8) +
             to_hex((low & 0x00F0) >> 4) + to_hex((low & 0x000F));
    }
  }
};

class JsonDouble : public mjson::JsonValue {
public:
  JsonDouble() : value_(0) {}
  JsonDouble(double value) : value_(value) {}
  double number_value() const override { return value_; }
  double &number_value() override { return value_; }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  void dump(std::string &) const override;

private:
  double value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_DOUBLE;
};

class JsonInt : public mjson::JsonValue {
public:
  JsonInt() : int_value_(0), double_value_(static_cast<double>(int_value_)) {}
  JsonInt(int value)
      : int_value_(value), double_value_(static_cast<double>(int_value_)) {}
  double number_value() const override { return double_value_; }
  double &number_value() override { return double_value_; }
  int int_value() const override { return int_value_; }
  int &int_value() override { return int_value_; }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  void dump(std::string &) const override;

private:
  int int_value_;
  double double_value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_INT;
};

class JsonBoolean : public mjson::JsonValue {
public:
  JsonBoolean() : value_(false) {}
  JsonBoolean(bool value) : value_(value) {}
  bool bool_value() const override { return value_; }
  bool &bool_value() override { return value_; }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  void dump(std::string &) const override;

private:
  bool value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_BOOL;
};

class JsonString : public mjson::JsonValue {
public:
  JsonString() {}
  JsonString(const std::string &value) : value_(value) {}
  JsonString(const char *value) { value_ = value; }
  const std::string &string_value() const override { return value_; }
  std::string &string_value() override { return value_; }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  void dump(std::string &) const override;

private:
  std::string value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_STRING;
};

class JsonArray : public mjson::JsonValue {
public:
  JsonArray() {}
  JsonArray(const mjson::Json::array &value)
      : value_(value.begin(), value.end()) {}
  const mjson::Json::array &array_value() const override { return value_; }
  mjson::Json::array &array_value() override { return value_; }
  const mjson::Json &operator[](size_t index) const override {
    return value_[index];
  }
  mjson::Json &operator[](size_t index) override { return value_[index]; }
  mjson::Json &ref_element(size_t index) override { return value_[index]; }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  void dump(std::string &) const override;

private:
  mjson::Json::array value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_ARRAY;
};

class JsonObject : public mjson::JsonValue {
public:
  JsonObject() {}
  JsonObject(const mjson::Json::object value)
      : value_(value.begin(), value.end()) {}
  const mjson::Json::object &object_value() const override { return value_; }
  mjson::Json::object &object_value() override { return value_; }
  const mjson::Json &operator[](const std::string &key) const override {
    const auto object_item = value_.find(key);
    if (object_item == value_.end()) {
      return EMPTY_JSON_VALUE;
    }
    return object_item->second;
  }
  mjson::Json &operator[](const std::string &key) override {
    return value_[key];
  }
  mjson::Json &ref_element(const std::string &key) override {
    return value_[key];
  }
  mjson::Json::JsonValueType get_value_type() const override {
    return VALUE_TYPE;
  }
  bool has_key(const std::string &key) const override {
    if (value_.find(key) == value_.end()) {
      return false;
    }
    return true;
  }
  void dump(std::string &) const override;

private:
  mjson::Json::object value_;
  static const mjson::Json::JsonValueType VALUE_TYPE =
      mjson::Json::JsonValueType::JSON_VALUE_OBJECT;
  static const mjson::Json EMPTY_JSON_VALUE;
};
const mjson::Json JsonObject::EMPTY_JSON_VALUE;

// Convert different Json values into string
inline void JsonDouble::dump(std::string &output) const {
  char temp[DOUBLE_TO_STRING_SIZE];
  snprintf(temp, DOUBLE_TO_STRING_SIZE, "%.6f", value_);
  output += temp;
}

inline void JsonInt::dump(std::string &output) const {
  char temp[INT_TO_STRING_SIZE];
  snprintf(temp, INT_TO_STRING_SIZE, "%d", int_value_);
  output += temp;
}

inline void JsonBoolean::dump(std::string &output) const {
  if (value_) {
    output += "true";
  } else {
    output += "false";
  }
}

inline void JsonString::dump(std::string &output) const {
  output += '"';
  for (size_t i = 0; i < value_.size(); ++i) {
    char ch = value_[i];
    if ((ch & 0x80) == 0) {
      if (ch == '\"') {
        output += '\\';
        output += '"';
      } else if (ch == '\\') {
        output += '\\';
        output += '\\';
      } else if (ch == '\b') {
        output += '\\';
        output += 'b';
      } else if (ch == '\f') {
        output += '\\';
        output += 'f';
      } else if (ch == '\n') {
        output += '\\';
        output += 'n';
      } else if (ch == '\r') {
        output += '\\';
        output += 'r';
      } else if (ch == '\t') {
        output += '\\';
        output += 't';
      } else {
        output.push_back(ch);
      }
    } else if ((ch & 0xE0) == 0xC0) {
      assert(i + 1 < value_.size());
      uint32_t v = ((value_[i] & 0x1F) << 6) | ((value_[i + 1] & 0x3F));
      output += JsonUtil::to_utf16_dump(v);
      i += 1;
    } else if ((ch & 0xF0) == 0xE0) {
      assert(i + 2 < value_.size());
      uint32_t v = ((value_[i] & 0x0F) << 12) | ((value_[i + 1] & 0x3F) << 6) |
                   ((value_[i + 2] & 0x3F));
      output += JsonUtil::to_utf16_dump(v);
      i += 2;
    } else if ((ch & 0xF8) == 0xF0) {
      assert(i + 3 < value_.size());
      uint32_t v = ((value_[i] & 0x07) << 18) | ((value_[i + 1] & 0x3F) << 12) |
                   ((value_[i + 2] & 0x3F) << 6) | ((value_[i + 3] & 0x3F));
      output += JsonUtil::to_utf16_dump(v);
      i += 3;
    }
  }
  output += '"';
}

inline void JsonArray::dump(std::string &output) const {
  output += "[";
  bool begin_flag = true;
  for (auto array_item : value_) {
    if (!begin_flag) {
      output += ", ";
    }
    array_item.dump(output);
    begin_flag = false;
  }
  output += "]";
}

inline void JsonObject::dump(std::string &output) const {
  output += "{";
  bool begin_flag = true;
  for (auto object_item : value_) {
    if (!begin_flag) {
      output += ", ";
    }
    std::string ret;
    JsonString(object_item.first).dump(ret);
    output += ret;
    output += " : ";
    object_item.second.dump(output);
    begin_flag = false;
  }
  output += "}";
}

/*
  class JsonParser is used to parse json data types,
  if parse_success() retrun true, you can get_jsonparser_answer(),
  or you will get_failed_index().
*/
class JsonParser {
public:
  JsonParser() {}
  JsonParser(const std::string &obj_string)
      : flag(true), index(0), obj_string(obj_string),
        obj_string_size(obj_string.size()) {
    clean_blank();
    answer = parse();
    if (index + 1 < obj_string_size) {
      index++;
      clean_blank();
      if (index + 1 < obj_string_size) {
        flag = false;
      }
    }
  }

  mjson::Json get_jsonparser_answer() { return answer; }

  bool parse_success() { return flag; }

  size_t get_failed_index() { return index; }

private:
  void clean_blank() {
    while (index < obj_string_size) {
      if (obj_string[index] == ' ' || obj_string[index] == '\n' ||
          obj_string[index] == '\t' || obj_string[index] == '\r') {
        index++;
      } else {
        break;
      }
    }
  }

  bool string_equal(size_t begin, size_t length, std::string reference_string) {
    if (begin + length > obj_string_size)
      return false;
    for (size_t i = begin, j = 0; j < length && i < obj_string_size; i++, j++)
      if (obj_string[i] != reference_string[j]) {
        return false;
      }
    return true;
  }

  bool is_range_left_to_right(char ch, char left, char right) {
    if (ch >= left && ch < right)
      return true;
    return false;
  }

  void solve_digits() {
    while (index + 1 < obj_string_size &&
           is_range_left_to_right(obj_string[index + 1], '0', '9' + 1)) {
      index++;
    }
  }

  mjson::Json parse_null() {
    if (!string_equal(index, NULL_LENGTH, NULL_STRING)) {
      MJSON_FAIL();
    }
    index += NULL_LENGTH - 1;
    return mjson::Json();
  }

  mjson::Json parse_true() {
    if (!string_equal(index, TRUE_LENGTH, TRUE_STRING)) {
      MJSON_FAIL();
    }
    index += TRUE_LENGTH - 1;
    return mjson::Json(true);
  }

  mjson::Json parse_false() {
    if (!string_equal(index, FALSE_LENGTH, FALSE_STRING)) {
      MJSON_FAIL();
    }
    index += FALSE_LENGTH - 1;
    return mjson::Json(false);
  }

  mjson::Json parse_number() {
    auto curent_begin = index;
    bool is_int = true;

    if (obj_string[index] == '0') { // zero

      if (index + 1 < obj_string_size &&
          is_range_left_to_right(obj_string[index + 1], '0', '9' + 1)) {
        MJSON_FAIL();
      }

    } else if (obj_string[index] == '-') {
      IS_ADD_MJSON_FAIL();
      index++;

      if (is_range_left_to_right(obj_string[index], '1', '9' + 1)) {
        solve_digits();
      } else if (obj_string[index] == '0') {
        if (index + 1 < obj_string_size &&
            is_range_left_to_right(obj_string[index + 1], '0', '9' + 1)) {
          MJSON_FAIL();
        }
      } else {
        MJSON_FAIL();
      }

    } else if (is_range_left_to_right(obj_string[index], '1', '9' + 1)) {
      solve_digits();
    } else {
      MJSON_FAIL();
    }

    if (index + 1 < obj_string_size && obj_string[index + 1] == '.') {
      index++;

      IS_ADD_MJSON_FAIL();
      index++;

      if (!is_range_left_to_right(obj_string[index], '0', '9' + 1)) {
        MJSON_FAIL();
      }
      solve_digits();

      is_int = false;
    }

    if (index + 1 < obj_string_size &&
        (obj_string[index + 1] == 'E' || obj_string[index + 1] == 'e')) {
      index++;

      is_int = false;
      IS_ADD_MJSON_FAIL();
      index++;

      if (obj_string[index] == '+' || obj_string[index] == '-') {
        IS_ADD_MJSON_FAIL();
        index++;
      }

      if (!is_range_left_to_right(obj_string[index], '0', '9' + 1)) {
        MJSON_FAIL();
      }
      solve_digits();
    }
    if (is_int) {
      return mjson::Json(std::atoi(obj_string.c_str() + curent_begin));
    }
    return mjson::Json(std::strtod(obj_string.c_str() + curent_begin, nullptr));
  }

  mjson::Json parse_string() {
    std::string temp;
    int32_t cached_codepoint = -1;
    while (true) {
      IS_ADD_MJSON_FAIL();
      index++;

      if (obj_string[index] != '\\' && cached_codepoint != -1) {
        temp += JsonUtil::to_utf8(cached_codepoint);
        cached_codepoint = -1;
      }

      if (obj_string[index] == '"') {
        return mjson::Json(temp);
        break;
      } else if (obj_string[index] == '\\') {
        IS_ADD_MJSON_FAIL();
        index++;

        // parse unicode string
        if (obj_string[index] == 'u') {
          IS_ADD_MJSON_FAIL();
          auto u1 = obj_string[++index];
          if (!JsonUtil::is_hex(u1))
            MJSON_FAIL();
          IS_ADD_MJSON_FAIL();
          auto u2 = obj_string[++index];
          if (!JsonUtil::is_hex(u2))
            MJSON_FAIL();
          IS_ADD_MJSON_FAIL();
          auto u3 = obj_string[++index];
          if (!JsonUtil::is_hex(u3))
            MJSON_FAIL();
          IS_ADD_MJSON_FAIL();
          auto u4 = obj_string[++index];
          if (!JsonUtil::is_hex(u4))
            MJSON_FAIL();

          int32_t codepoint =
              (JsonUtil::from_hex(u1) << 12) | (JsonUtil::from_hex(u2) << 8) |
              (JsonUtil::from_hex(u3) << 4) | JsonUtil::from_hex(u4);
          if (0xD800 <= cached_codepoint && cached_codepoint <= 0xDBFF &&
              0xDC00 <= cached_codepoint && cached_codepoint <= 0xDFFF) {
            temp += JsonUtil::to_utf8(
                (((cached_codepoint - 0xD800) << 10) | (codepoint - 0xDC00)) +
                0x10000);
            cached_codepoint = -1;
          } else {
            temp += JsonUtil::to_utf8(cached_codepoint);
            cached_codepoint = codepoint;
          }
        } else {
          if (cached_codepoint != -1) {
            temp += JsonUtil::to_utf8(cached_codepoint);
            cached_codepoint = -1;
          }

          if (obj_string[index] == '"') {
            temp.push_back('\"');
          } else if (obj_string[index] == '\\') {
            temp.push_back('\\');
          } else if (obj_string[index] == '/') {
            temp.push_back('/');
          } else if (obj_string[index] == 'b') {
            temp.push_back('\b');
          } else if (obj_string[index] == 'f') {
            temp.push_back('\f');
          } else if (obj_string[index] == 'n') {
            temp.push_back('\n');
          } else if (obj_string[index] == 'r') {
            temp.push_back('\r');
          } else if (obj_string[index] == 't') {
            temp.push_back('\t');
          } else {
            MJSON_FAIL();
          }
        }

      } else {
        temp.push_back(obj_string[index]);
      }
    }
  }

  mjson::Json parse_array() {
    mjson::Json::array temp;
    bool begin_flag = true;
    while (true) {
      IS_ADD_MJSON_FAIL();
      index++;

      clean_blank();
      IS_NOW_MJSON_FAIL();

      if (obj_string[index] == ']') {
        return mjson::Json(temp);
      } else if (obj_string[index] == ',' || begin_flag) {
        if (!begin_flag) {
          IS_ADD_MJSON_FAIL();
          index++;
          clean_blank();
          IS_NOW_MJSON_FAIL();
        }

        begin_flag = false;
        temp.push_back(parse());

        IS_MJSON_FAIL();
      } else {
        MJSON_FAIL();
      }
    }
  }

  mjson::Json parse_object() {
    mjson::Json::object temp;
    std::string string_key;
    mjson::Json json_object;
    bool is_exist_string = false;

    while (true) {
      IS_ADD_MJSON_FAIL();
      index++;
      clean_blank();
      IS_NOW_MJSON_FAIL();

      // construct a map of string_key and JsonValue:

      // First must construct string key
      is_exist_string = false;
      if (obj_string[index] == '"') {
        auto json = parse_string();
        string_key = json.string_value();
        is_exist_string = true;
      } else if (obj_string[index] == '}') {
        return mjson::Json(temp);
      } else {
        MJSON_FAIL();
      }

      IS_ADD_MJSON_FAIL();
      index++;
      clean_blank();
      IS_NOW_MJSON_FAIL();

      if (obj_string[index] == ':') {
        if (!is_exist_string || temp.find(string_key) != temp.end()) {
          MJSON_FAIL();
        }

        IS_ADD_MJSON_FAIL();
        index++;
        clean_blank();
        IS_NOW_MJSON_FAIL();

        json_object = parse();
        IS_MJSON_FAIL(); // judge flag is false
        temp[string_key] = json_object;
      } else {
        MJSON_FAIL();
      }

      IS_ADD_MJSON_FAIL();
      index++;
      clean_blank();
      IS_NOW_MJSON_FAIL();

      if (obj_string[index] == '}') {
        return mjson::Json(temp);
      } else if (obj_string[index] != ',') {
        MJSON_FAIL();
      }
    }
  }

  mjson::Json parse() {
    char ch = obj_string[index];
    switch (ch) {
    case 'n': // null
      return parse_null();

    case 'f': // false
      return parse_false();

    case 't': // true
      return parse_true();

    case '"': // string
      return parse_string();

    case '[': // array
      return parse_array();

    case '{': // object
      return parse_object();

    default: // number
      return parse_number();
    }
  }

private:
  bool flag;
  size_t index;
  std::string obj_string;
  size_t obj_string_size;
  mjson::Json answer;
};
} // namespace

// Constructors
inline mjson::Json::Json() : value_ptr_(nullptr) {}

inline mjson::Json::Json(double value) : value_ptr_(new JsonDouble(value)) {}

inline mjson::Json::Json(int value) : value_ptr_(new JsonInt(value)) {}

inline mjson::Json::Json(bool value) : value_ptr_(new JsonBoolean(value)) {}

inline mjson::Json::Json(const char *value)
    : value_ptr_(new JsonString(value)) {}

inline mjson::Json::Json(const std::string &value)
    : value_ptr_(new JsonString(value)) {}

inline mjson::Json::Json(const array &value)
    : value_ptr_(new JsonArray(value)) {}

inline mjson::Json::Json(const object &value)
    : value_ptr_(new JsonObject(value)) {}

inline mjson::Json &mjson::Json::operator=(const mjson::Json &value) {
  switch (value.type()) {
  case Json::JsonValueType::JSON_VALUE_NULL:
    value_ptr_.reset();
    break;
  case Json::JsonValueType::JSON_VALUE_DOUBLE:
    value_ptr_.reset(new JsonDouble(value.number_value()));
    break;
  case Json::JsonValueType::JSON_VALUE_INT:
    value_ptr_.reset(new JsonInt(value.int_value()));
    break;
  case Json::JsonValueType::JSON_VALUE_BOOL:
    value_ptr_.reset(new JsonBoolean(value.bool_value()));
    break;
  case Json::JsonValueType::JSON_VALUE_STRING:
    value_ptr_.reset(new JsonString(value.string_value()));
    break;
  case Json::JsonValueType::JSON_VALUE_ARRAY:
    value_ptr_.reset(new JsonArray(value.array_value()));
    break;
  case Json::JsonValueType::JSON_VALUE_OBJECT:
    value_ptr_.reset(new JsonObject(value.object_value()));
    break;
  default:
    value_ptr_.reset();
    break;
  }
  return *this;
}

// Accessors
inline double mjson::Json::number_value() const {
  if (!is_number()) {
    return ERROR_NUMBER_VALUE;
  }
  return value_ptr_->number_value();
}

inline double &mjson::Json::number_value() {
  if (!is_number()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonDouble>();
    } else {
      static double ret_value = ERROR_NUMBER_VALUE;
      return ret_value;
    }
  }
  return value_ptr_->number_value();
}

inline int mjson::Json::int_value() const {
  if (!is_int()) {
    return ERROR_INT_VALUE;
  }
  return value_ptr_->int_value();
}

inline int &mjson::Json::int_value() {
  if (!is_int()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonInt>();
    } else {
      static int ret_value = ERROR_INT_VALUE;
      return ret_value;
    }
  }
  return value_ptr_->int_value();
}

inline bool mjson::Json::bool_value() const {
  if (!is_bool()) {
    return false;
  }
  return value_ptr_->bool_value();
}

inline bool &mjson::Json::bool_value() {
  if (!is_bool()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonBoolean>();
    } else {
      static bool ret_value = false;
      return ret_value;
    }
  }
  return value_ptr_->bool_value();
}

inline const std::string &mjson::Json::string_value() const {
  if (!is_string()) {
    return EMPTY_JSON_STRING;
  }
  return value_ptr_->string_value();
}

inline std::string &mjson::Json::string_value() {
  if (!is_string()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonString>();
    } else {
      static std::string ret_value = EMPTY_JSON_STRING;
      return ret_value;
    }
  }
  return value_ptr_->string_value();
}

inline const mjson::Json::array &mjson::Json::array_value() const {
  if (!is_array()) {
    return EMPTY_JSON_ARRAY;
  }
  return value_ptr_->array_value();
}

inline mjson::Json::array &mjson::Json::array_value() {
  if (!is_array()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonArray>();
    } else {
      static mjson::Json::array ret_value = EMPTY_JSON_ARRAY;
      return ret_value;
    }
  }
  return value_ptr_->array_value();
}

inline const mjson::Json::object &mjson::Json::object_value() const {
  if (!is_object()) {
    return EMPTY_JSON_OBJECT;
  }
  return value_ptr_->object_value();
}

inline mjson::Json::object &mjson::Json::object_value() {
  if (!is_object()) {
    if (is_null()) {
      value_ptr_ = std::make_shared<JsonObject>();
    } else {
      static mjson::Json::object ret_value = EMPTY_JSON_OBJECT;
      return ret_value;
    }
  }
  return value_ptr_->object_value();
}

inline const mjson::Json &mjson::Json::operator[](size_t index) const {
  if (!is_array()) {
    return EMPTY_JSON_VALUE;
  }
  return (*value_ptr_)[index];
}

inline mjson::Json &mjson::Json::operator[](size_t index) {
  if (!is_array()) {
    static mjson::Json ret_value = EMPTY_JSON_VALUE;
    return ret_value;
  }

  return (*value_ptr_)[index];
}

inline const mjson::Json &mjson::Json::
operator[](const std::string &key) const {
  if (!is_object()) {
    return EMPTY_JSON_VALUE;
  }
  return (*value_ptr_)[key];
}

inline mjson::Json &mjson::Json::operator[](const std::string &key) {
  if (!is_object()) {
    static mjson::Json ret_value = EMPTY_JSON_VALUE;
    return ret_value;
  }
  return (*value_ptr_)[key];
}

inline mjson::Json &mjson::Json::ref_element(const std::string &key) {
  return (*value_ptr_).ref_element(key);
}

inline mjson::Json &mjson::Json::ref_element(size_t index) {
  return (*value_ptr_).ref_element(index);
}

inline bool mjson::Json::has_key(const std::string &key) const {
  if (!is_object()) {
    return false;
  }
  return value_ptr_->has_key(key);
}

// Check different type
inline bool mjson::Json::is_null() const { return (!value_ptr_); }

inline bool mjson::Json::is_int() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type() ==
           mjson::Json::JsonValueType::JSON_VALUE_INT;
  }
  return false;
}

inline bool mjson::Json::is_number() const {
  if (value_ptr_) {
    return (value_ptr_->get_value_type() ==
                mjson::Json::JsonValueType::JSON_VALUE_INT ||
            value_ptr_->get_value_type() ==
                mjson::Json::JsonValueType::JSON_VALUE_DOUBLE);
  }
  return false;
}

inline bool mjson::Json::is_bool() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type() ==
           mjson::Json::JsonValueType::JSON_VALUE_BOOL;
  }
  return false;
}

inline bool mjson::Json::is_string() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type() ==
           mjson::Json::JsonValueType::JSON_VALUE_STRING;
  }
  return false;
}

inline bool mjson::Json::is_array() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type() ==
           mjson::Json::JsonValueType::JSON_VALUE_ARRAY;
  }
  return false;
}

inline bool mjson::Json::is_object() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type() ==
           mjson::Json::JsonValueType::JSON_VALUE_OBJECT;
  }
  return false;
}

inline mjson::Json::JsonValueType mjson::Json::type() const {
  if (value_ptr_) {
    return value_ptr_->get_value_type();
  }
  return mjson::Json::JsonValueType::JSON_VALUE_NULL;
}

// Convert Json into string
inline std::string mjson::Json::dump() const {
  std::string output;
  dump(output);
  return output;
}

inline void mjson::Json::dump(std::string &output) const {
  if (is_null()) {
    output += "null";
  } else if (value_ptr_) {
    value_ptr_->dump(output);
  }
}

inline mjson::Json mjson::Json::parse(const std::string &input,
                                      std::string &error_info) {
  JsonParser temp = JsonParser(input);

  if (!temp.parse_success()) {
    error_info = "parse error at index: ";
    char digit[ERROR_INDEX_DIGIT_LENGTH];
    sprintf(digit, "%zu", temp.get_failed_index());
    error_info += digit;
    return mjson::Json();
  }
  error_info = "";
  return temp.get_jsonparser_answer();
}

