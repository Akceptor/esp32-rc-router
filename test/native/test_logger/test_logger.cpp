#include <unity.h>
#include <string.h>
#include "logging/logger.h"

void setUp(void) {
  Logger::instance().begin(LogLevel::INFO, false);
  Logger::instance().clear();
}

void tearDown(void) {}

void test_level_filtering_suppresses_debug_at_info(void) {
  Logger::instance().setLevel(LogLevel::INFO);
  Logger::instance().clear();
  LOG_D("TAG", "debug line %d", 1);
  char out[512];
  size_t n = Logger::instance().dump(out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(0, n);
  TEST_ASSERT_EQUAL_STRING("", out);
}

void test_dump_contains_logged_line(void) {
  Logger::instance().setLevel(LogLevel::INFO);
  Logger::instance().clear();
  LOG_I("TAG", "hello %s", "world");
  char out[512];
  size_t n = Logger::instance().dump(out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NOT_NULL(strstr(out, "[INFO][TAG] hello world"));
}

void test_clear_empties_buffer(void) {
  Logger::instance().setLevel(LogLevel::INFO);
  LOG_I("TAG", "some line");
  Logger::instance().clear();
  char out[64];
  size_t n = Logger::instance().dump(out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(0, n);
  TEST_ASSERT_EQUAL_UINT32(0, Logger::instance().droppedLines());
}

void test_ring_wrap_drops_oldest_and_counts(void) {
  Logger::instance().setLevel(LogLevel::INFO);
  Logger::instance().clear();
  for (int i = 0; i < 400; i++) {
    LOG_I("T", "line number %d filler filler filler", i);
  }
  TEST_ASSERT_TRUE(Logger::instance().droppedLines() > 0);
  char out[4096];
  size_t n = Logger::instance().dump(out, sizeof(out));
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_NULL(strstr(out, "line number 0 filler"));
  TEST_ASSERT_NOT_NULL(strstr(out, "line number 399 filler"));
}

void test_format_args_work(void) {
  Logger::instance().setLevel(LogLevel::DEBUG);
  Logger::instance().clear();
  LOG_D("FMT", "int=%d str=%s hex=%02x", 42, "abc", 0xA);
  char out[256];
  Logger::instance().dump(out, sizeof(out));
  TEST_ASSERT_NOT_NULL(strstr(out, "int=42 str=abc hex=0a"));
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_level_filtering_suppresses_debug_at_info);
  RUN_TEST(test_dump_contains_logged_line);
  RUN_TEST(test_clear_empties_buffer);
  RUN_TEST(test_ring_wrap_drops_oldest_and_counts);
  RUN_TEST(test_format_args_work);
  return UNITY_END();
}
