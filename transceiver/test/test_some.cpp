#include <unity.h>
#include <simple_crc.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_run_empty_array() {
    uint8_t* data = nullptr;
    uint8_t crc = calcCheckSum(data, 0); 
    TEST_ASSERT_TRUE(crc == 173);
}

void test_run_test_array() {
    uint8_t data[] = {10, 57, 0, 255, 128, 127, 61, 20, 32, 103};
    uint8_t crc = calcCheckSum(data, 10); 
    TEST_ASSERT_EQUAL_INT(240, crc);
}

void test_run_test_array_with_zero() {
    uint8_t data[] = {0};
    uint8_t crc = calcCheckSum(data, 1); 
    TEST_ASSERT_EQUAL_INT(173, crc);
}

void test_run_test_array_with_two_zero() {
    uint8_t data[] = {0, 0};
    uint8_t crc = calcCheckSum(data, 2); 
    TEST_ASSERT_EQUAL_INT(173, crc);
}


// data read by pin to have stable value
#pragma pack(push, 1)
struct AVG_VALUE {
  uint16_t values[8];
  uint8_t position;
  uint16_t average;
  bool ready;  
};
#pragma pack(pop)

void writeValueToAverage(AVG_VALUE &avg, uint16_t value)
{
  avg.values[avg.position++] = value;
  if (avg.position >= 8) {
    avg.position = 0;
    avg.ready = true;
  }  
  uint32_t sum = avg.values[0];
  uint16_t *ptr = &avg.values[1];
  uint16_t *end_ptr = &avg.values[8]; // illegal pointer for condition
  // sum up
  while (ptr != end_ptr) {
    sum += *ptr;
    ++ptr;
  }
  avg.average = sum >> 3;
}

void test_fill_value_in_average() {
    AVG_VALUE avg;
    avg.position = 0;
    avg.ready = false;
    TEST_ASSERT_EQUAL_INT(avg.position, 0);
    TEST_ASSERT_FALSE(avg.ready);

    writeValueToAverage(avg, 0);
    TEST_ASSERT_EQUAL_INT(avg.position, 1);
    TEST_ASSERT_FALSE(avg.ready);

    writeValueToAverage(avg, 100);
    TEST_ASSERT_EQUAL_INT(avg.position, 2);
    TEST_ASSERT_FALSE(avg.ready);
 
    writeValueToAverage(avg, 200);
    writeValueToAverage(avg, 300);
    writeValueToAverage(avg, 400);
    writeValueToAverage(avg, 500);
    writeValueToAverage(avg, 600);
    TEST_ASSERT_FALSE(avg.ready);
    writeValueToAverage(avg, 700);
    TEST_ASSERT_EQUAL_INT(0, avg.position);
    TEST_ASSERT_TRUE(avg.ready);
    TEST_ASSERT_EQUAL_INT(350, avg.average);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_run_empty_array);
    RUN_TEST(test_run_test_array);
    RUN_TEST(test_run_test_array_with_zero);
    RUN_TEST(test_run_test_array_with_two_zero);
    RUN_TEST(test_fill_value_in_average);
    UNITY_END();
}
