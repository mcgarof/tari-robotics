#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <unity.h>

#include "observability/NonBlockingWriter.h"

class FakeTransport
{
public:
    explicit FakeTransport(std::size_t available)
        : available_(available)
    {
    }

    int availableForWrite() const
    {
        return static_cast<int>(available_);
    }

    std::size_t write(
        const uint8_t* data,
        std::size_t length)
    {
        const std::size_t count =
            std::min(available_, length);

        output_.append(
            reinterpret_cast<const char*>(data),
            count);

        return count;
    }

    void setAvailable(std::size_t available)
    {
        available_ = available;
    }

    const std::string& output() const
    {
        return output_;
    }

private:
    std::size_t available_;
    std::string output_;
};

class PartialWriteTransport
{
public:
    int availableForWrite() const
    {
        return 10;
    }

    std::size_t write(
        const uint8_t* data,
        std::size_t length)
    {
        const std::size_t count =
            std::min<std::size_t>(2, length);

        output_.append(
            reinterpret_cast<const char*>(data),
            count);

        return count;
    }

    const std::string& output() const
    {
        return output_;
    }

private:
    std::string output_;
};

void setUp()
{
}

void tearDown()
{
}

void test_initial_state()
{
    NonBlockingWriter writer;

    TEST_ASSERT_FALSE(writer.busy());
    TEST_ASSERT_FALSE(writer.complete());
}

void test_begin_rejects_invalid_data()
{
    NonBlockingWriter writer;

    TEST_ASSERT_FALSE(writer.begin(nullptr, 10));

    const char data[] = "abc";

    TEST_ASSERT_FALSE(writer.begin(data, 0));
}

void test_write_complete_line()
{
    NonBlockingWriter writer;
    FakeTransport transport{100};

    const char data[] = "hello\n";

    TEST_ASSERT_TRUE(
        writer.begin(data, std::strlen(data)));

    writer.update(transport);

    TEST_ASSERT_EQUAL_STRING(
        data,
        transport.output().c_str());

    TEST_ASSERT_FALSE(writer.busy());
    TEST_ASSERT_TRUE(writer.complete());
}

void test_write_is_incremental()
{
    NonBlockingWriter writer;
    FakeTransport transport{3};

    const char data[] = "abcdefgh";

    TEST_ASSERT_TRUE(
        writer.begin(data, std::strlen(data)));

    writer.update(transport);

    TEST_ASSERT_EQUAL_STRING(
        "abc",
        transport.output().c_str());

    TEST_ASSERT_TRUE(writer.busy());
    TEST_ASSERT_FALSE(writer.complete());

    writer.update(transport);

    TEST_ASSERT_EQUAL_STRING(
        "abcdef",
        transport.output().c_str());

    TEST_ASSERT_TRUE(writer.busy());

    writer.update(transport);

    TEST_ASSERT_EQUAL_STRING(
        "abcdefgh",
        transport.output().c_str());

    TEST_ASSERT_FALSE(writer.busy());
    TEST_ASSERT_TRUE(writer.complete());
}

void test_zero_available_writes_nothing()
{
    NonBlockingWriter writer;
    FakeTransport transport{0};

    const char data[] = "hello";

    TEST_ASSERT_TRUE(
        writer.begin(data, std::strlen(data)));

    writer.update(transport);

    TEST_ASSERT_TRUE(transport.output().empty());
    TEST_ASSERT_TRUE(writer.busy());
    TEST_ASSERT_FALSE(writer.complete());
}

void test_cannot_begin_while_busy()
{
    NonBlockingWriter writer;

    const char first[] = "first";
    const char second[] = "second";

    TEST_ASSERT_TRUE(
        writer.begin(first, std::strlen(first)));

    TEST_ASSERT_FALSE(
        writer.begin(second, std::strlen(second)));
}

void test_reset_allows_new_write()
{
    NonBlockingWriter writer;
    FakeTransport transport{100};

    const char first[] = "first";

    TEST_ASSERT_TRUE(
        writer.begin(first, std::strlen(first)));

    writer.update(transport);

    TEST_ASSERT_TRUE(writer.complete());

    writer.reset();

    TEST_ASSERT_FALSE(writer.busy());
    TEST_ASSERT_FALSE(writer.complete());

    const char second[] = "second";

    TEST_ASSERT_TRUE(
        writer.begin(second, std::strlen(second)));
}

void test_handles_partial_write()
{
    NonBlockingWriter writer;
    PartialWriteTransport transport;

    const char data[] = "abcdef";

    TEST_ASSERT_TRUE(
        writer.begin(data, std::strlen(data)));

    writer.update(transport);
    TEST_ASSERT_EQUAL_STRING("ab", transport.output().c_str());
    TEST_ASSERT_TRUE(writer.busy());

    writer.update(transport);
    TEST_ASSERT_EQUAL_STRING("abcd", transport.output().c_str());
    TEST_ASSERT_TRUE(writer.busy());

    writer.update(transport);
    TEST_ASSERT_EQUAL_STRING("abcdef", transport.output().c_str());

    TEST_ASSERT_FALSE(writer.busy());
    TEST_ASSERT_TRUE(writer.complete());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_initial_state);
    RUN_TEST(test_begin_rejects_invalid_data);
    RUN_TEST(test_write_complete_line);
    RUN_TEST(test_write_is_incremental);
    RUN_TEST(test_zero_available_writes_nothing);
    RUN_TEST(test_cannot_begin_while_busy);
    RUN_TEST(test_reset_allows_new_write);
    RUN_TEST(test_handles_partial_write);

    return UNITY_END();
}
