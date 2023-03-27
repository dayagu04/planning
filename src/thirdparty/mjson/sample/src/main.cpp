#include "mjson/mjson.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace mjson;

#define ASSERT_MSG(_cond, fmt...)                                              \
  do {                                                                         \
    if (!(_cond)) {                                                            \
      char buffer[1000];                                                       \
      sprintf(buffer, fmt);                                                    \
      fprintf(stderr, "%s\n", buffer);                                         \
      abort();                                                                 \
    };                                                                         \
  } while (0)

#define ASSERT(_cond) ASSERT_MSG(_cond, "Assert fail %d", __LINE__)

constexpr int INT_ZERO = 0;
constexpr double NUMBER_ZERO = 0;
constexpr double NUMBER_POSITIVE_DECIMAL = 123.456;
constexpr double NUMBER_NEGATIVE_SCIENTIFIC = -789E2;
constexpr double NUMBER_POSITIVE_SCIENTIFIC = 102e-3;
constexpr double NUMBER_POSITIVE_DECIMAL_SCIENTIFIC = 4567.1234e5;
constexpr double NUMBER_NEGATIVE_DECIMAL_SCIENTIFIC = -0.1234E5;

constexpr bool BOOLEAN_TRUE = true;
constexpr bool BOOLEAN_FALSE = false;

const std::string STRING = "`1234567890-=  qwertyuiop[]\\asdfghjkl;'  "
                           "zxcvbnm,./  ~!@#$%^&*()_+  QWERTYUIOP{}  "
                           "ASDFGHJKL:\" ZXCVBNM<>?\"\\/\b\f\n\r\t";
const std::string WORD_STRING = "word string ";
const std::string STRING_EXAMPLE = "ok";
const std::string EMPTY_STRING = "";

const int INT_POSITIVE = 1234567890;
const double NUMBER_POSITIVE = 987654321.0;

const std::string KEY_STRING1 = "key_one";
const std::string KEY_STRING2 = "key_two";
const std::string KEY_STRING3 = "key_three";
const std::string KEY_STRING4 = "key_four";
const std::string KEY_STRING5 = "key_five";
const std::string KEY_STRING6 = "key_six";

Json init() {
  Json::array array_a, array_b, array_c, array_null;
  Json::object obj_a, obj_b, obj_c;

  array_a.push_back(Json());

  array_a.push_back(Json(NUMBER_ZERO));
  array_a.push_back(Json(NUMBER_POSITIVE_DECIMAL));
  array_a.push_back(Json(NUMBER_NEGATIVE_SCIENTIFIC));
  array_a.push_back(Json(NUMBER_POSITIVE_SCIENTIFIC));
  array_a.push_back(Json(NUMBER_POSITIVE_DECIMAL_SCIENTIFIC));
  array_a.push_back(Json(NUMBER_NEGATIVE_DECIMAL_SCIENTIFIC));

  array_a.push_back(Json(BOOLEAN_TRUE));
  array_a.push_back(Json(BOOLEAN_FALSE));

  array_a.push_back(Json(STRING));

  array_a.push_back(Json(array_null));

  array_b.push_back(Json());
  array_b.push_back(Json(BOOLEAN_FALSE));
  array_b.push_back(Json(BOOLEAN_TRUE));
  array_b.push_back(Json(INT_POSITIVE));
  array_b.push_back(Json(WORD_STRING));
  obj_a[STRING_EXAMPLE] = Json();
  array_b.push_back(Json(obj_a));
  array_a.push_back(Json(array_b));

  obj_b[KEY_STRING1] = Json();
  obj_b[KEY_STRING2] = NUMBER_POSITIVE;
  obj_b[KEY_STRING3] = BOOLEAN_FALSE;
  obj_b[KEY_STRING4] = BOOLEAN_TRUE;
  obj_b[KEY_STRING5] = STRING_EXAMPLE;
  array_c.push_back(Json(BOOLEAN_FALSE));
  obj_c[KEY_STRING6] = array_c;
  obj_b[KEY_STRING6] = obj_c;
  array_a.push_back(Json(obj_b));

  return Json(array_a);
}

const auto raw_json = "\
{ \
    \"test_name\": \"line\", \
    \"size\" : [\"480x640\", \"720x1280\", \"1080x1920\"], \
    \"num\": [1, 4, 16, 64, [12, 12]], \
    \"models\" : [ \
        {\"package\": \"raw.tar\", \"config\": \"my.json\"} \
    ], \
    \"model_path_prefix\": \"default\" \
} \
";

void test_ref() {
  // string
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    auto &const_string_ref = const_json_value.string_value();
    ASSERT(json_value.is_null());
    ASSERT(const_string_ref == EMPTY_STRING);

    auto &string_ref = json_value.string_value();
    const auto &string_const_ref = json_value.string_value();
    ASSERT(json_value.is_string());
    ASSERT(json_value.string_value() == EMPTY_STRING);
    string_ref = STRING_EXAMPLE;
    ASSERT(json_value.string_value() == STRING_EXAMPLE);
    ASSERT(json_value.string_value() == string_const_ref);

    auto &int_ref = json_value.int_value();
    ASSERT(json_value.is_string());
    ASSERT(int_ref == INT_ZERO);
  }

  // int ref
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    const auto &const_int_ref = const_json_value.int_value();
    ASSERT(json_value.is_null());
    ASSERT(const_int_ref == INT_ZERO);

    const auto &int_ref_const = json_value.int_value();
    auto &int_ref = json_value.int_value();
    ASSERT(json_value.is_int());
    ASSERT(json_value.int_value() == INT_ZERO);
    int_ref = INT_POSITIVE;
    ASSERT(json_value.int_value() == INT_POSITIVE);
    ASSERT(json_value.int_value() == int_ref_const);

    auto &string_ref = json_value.string_value();
    ASSERT(json_value.is_int());
    ASSERT(string_ref == EMPTY_STRING);
  }

  // number ref
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    const auto &const_number_ref = const_json_value.number_value();
    ASSERT(json_value.is_null());
    ASSERT(const_number_ref == NUMBER_ZERO);

    const auto &number_ref_const = json_value.number_value();
    auto &number_ref = json_value.number_value();
    ASSERT(json_value.is_number());
    ASSERT(json_value.number_value() == NUMBER_ZERO);
    number_ref = NUMBER_POSITIVE;
    ASSERT(json_value.number_value() == NUMBER_POSITIVE);
    ASSERT(json_value.number_value() == number_ref_const);

    auto &string_ref = json_value.string_value();
    ASSERT(json_value.is_number());
    ASSERT(string_ref == EMPTY_STRING);
  }

  // bool ref
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    const auto &const_bool_ref = const_json_value.bool_value();
    ASSERT(json_value.is_null());
    ASSERT(const_bool_ref == 0);

    const auto &bool_ref_const = json_value.bool_value();
    auto &bool_ref = json_value.bool_value();
    ASSERT(json_value.is_bool());
    ASSERT(json_value.bool_value() == BOOLEAN_FALSE);
    bool_ref = BOOLEAN_TRUE;
    ASSERT(json_value.bool_value() == BOOLEAN_TRUE);
    ASSERT(json_value.bool_value() == bool_ref_const);

    auto &string_ref = json_value.string_value();
    ASSERT(json_value.is_bool());
    ASSERT(string_ref == EMPTY_STRING);
  }

  // object ref
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    const auto &const_object_ref = const_json_value.object_value();
    ASSERT(json_value.is_null());
    ASSERT(const_object_ref.empty());

    const auto &object_ref_const = json_value.object_value();
    auto &object_ref = json_value.object_value();
    ASSERT(json_value.is_object());
    ASSERT(json_value.object_value().empty());
    constexpr auto key1 = "key1";
    constexpr auto key2 = "key2";
    object_ref[key1] = Json(STRING_EXAMPLE);
    object_ref[key2] = Json(WORD_STRING);
    ASSERT(json_value.has_key(key1));
    ASSERT(json_value.has_key(key2));
    ASSERT(json_value[key1].string_value() == STRING_EXAMPLE);
    ASSERT(json_value[key2].string_value() == WORD_STRING);
    ASSERT(json_value.object_value() == object_ref_const);

    auto &string_ref = json_value.string_value();
    ASSERT(json_value.is_object());
    ASSERT(string_ref == EMPTY_STRING);
  }

  // array ref
  {
    Json json_value;
    ASSERT(json_value.is_null());
    const auto &const_json_value = json_value;
    const auto &const_array_ref = const_json_value.array_value();
    ASSERT(json_value.is_null());
    ASSERT(const_array_ref.empty());

    const auto &array_ref_const = json_value.array_value();
    auto &array_ref = json_value.array_value();
    ASSERT(json_value.is_array());
    ASSERT(json_value.array_value().empty());
    array_ref.push_back(Json(STRING_EXAMPLE));
    array_ref.push_back(Json(WORD_STRING));
    ASSERT(json_value[0].string_value() == STRING_EXAMPLE);
    ASSERT(json_value[1].string_value() == WORD_STRING);
    ASSERT(json_value.array_value() == array_ref_const);

    auto &string_ref = json_value.string_value();
    ASSERT(json_value.is_array());
    ASSERT(string_ref == EMPTY_STRING);
  }
}

int main(int argc, char *argv[]) {
  { // unicode
    std::string json_value = "{\"value\": \"\\u9B54\\u95E8\\u5854\"}";
    printf("origin %s\n", json_value.c_str());
    std::string error;
    auto json = mjson::Json::parse(json_value, error);
    ASSERT(error == "");
    printf("get value %s\n", json["value"].string_value().c_str());
    printf("dump value %s\n", json.dump().c_str());
  }
  {
    std::string tmp = "{ \"a\": 1, \"b\": 2}";
    std::string error;
    auto json = mjson::Json::parse(tmp, error);
    ASSERT(error == "");
    auto json_copy = json;
    json["a"] = 3;
    ASSERT(json_copy["a"].int_value() == 1);
    ASSERT(json["a"].int_value() == 3);
  }
  {
    std::string tmp = "{ \"PWD\": \"${PWD}\", \"shell\": \"${SHELL}\"}";
    mjson::Reader reader(tmp);
    printf("before expanding: %s\n", reader.raw().dump().c_str());
    auto variable_dict = reader.expand_environment_varialbe();
    printf("after expanding: %s\n", reader.raw().dump().c_str());
    for (auto &pair : variable_dict) {
      printf("%s: %s\n", pair.first.c_str(), pair.second.c_str());
    }
    printf("%s\n", reader.get<std::string>("shell").c_str());
  }
  {
    std::string error;
    auto json = mjson::Json::parse("3", error);
    ASSERT(json.is_int());
    ASSERT(error == "");
    printf("raw json of '3': %s\n", json.dump().c_str());
  }

  {
    std::string data = "{\"value\": [[1, 2, 3], [3, 4, 5]]}";
    mjson::Reader reader(data);
    auto value = reader.get<Matrix<int>>("value");
    auto print_matrix = [](Matrix<int> value) {
      printf("matrix shape: ");
      for (size_t i = 0; i < value.shape.size(); ++i)
        printf("%d ", value.shape[i]);
      printf("\n");
      printf("matrix value: ");
      for (size_t i = 0; i < value.data.size(); ++i)
        printf("%d ", value.data[i]);
      printf("\n");
    };
    printf("matrix origin %s\n", data.c_str());
    print_matrix(value);
  }

  {
    std::string data = "{\"value\": [[1, 2, 3], [3, 4, 5]]}";
    mjson::Reader reader(data);
    auto value = reader.get<std::vector<std::vector<int>>>("value");
    for (size_t i = 0; i < value.size(); ++i) {
      for (size_t j = 0; j < value[i].size(); ++j) {
        printf("%d ", value[i][j]);
      }
      printf("\n");
    }
  }

  {
    std::string error;
    auto json = mjson::Json::parse(raw_json, error);
    printf("before modify %s\n", json["num"][4].dump().c_str());
    json.ref_element("num").ref_element(4).ref_element(1) = mjson::Json(13);
    printf("after modify %s\n", json["num"][4].dump().c_str());
  }

  { // sample of overriding value
    mjson::Reader reader(raw_json, "reader1");
    printf("before overriding %s\n", reader.get<std::string>("num").c_str());
    reader.override_value("num[4][0]", "13");
    reader.override_value("num[4][1]", "\"hello\"");
    reader.override_value("num[3]", "{\"item\": [3]}");
    printf("after overriding %s\n", reader.get<std::string>("num").c_str());
  }

  {
    std::string error;
    auto raw = mjson::Json::parse(raw_json, error);
    ASSERT(error == "");
    mjson::Reader reader1(raw, "reader1");
    mjson::Reader reader2(raw_json, "reader2");
    auto reader1_package = reader1.get<std::string>("models[0].package");
    auto reader2_package = reader2.get<std::string>("models[0].package");
    ASSERT(reader1_package == reader2_package);
    ASSERT(reader1.get<std::string>("num[4][0]") ==
           reader2.get<std::string>("num[4][1]"));
    printf("models[0].package is %s\n", reader1_package.c_str());
    printf("num[4][1] is (string format) %s\n",
           reader2.get<std::string>("num[4][1]").c_str());
    printf("num[4][1] is (number format) %.2lf\n",
           reader2.get<float>("num[4][1]"));
    printf("num[4][1] is (int format) %d\n", reader2.get<int>("num[4][1]"));
  }

  {
    std::string json = "{\"abc\":\"yes\"}";
    std::string error;
    Json value = Json::parse(json, error);
    std::string true_key = "abc";
    std::string false_key = "no";
    ASSERT(value.has_key(true_key));
    ASSERT(false == value.has_key(false_key));
  }

  {
    const char *json = "{\"abc\":\"yes\"}";
    std::string error;
    Json value = Json::parse(json, error);
    std::string true_key = "abc";
    std::string false_key = "no";
    ASSERT(value.has_key(true_key));
    ASSERT(false == value.has_key(false_key));
  }
  {
    auto json = init();
    std::string str = json.dump();
    printf("done!:\n%s\n", str.c_str());
  }
  test_ref();
}
