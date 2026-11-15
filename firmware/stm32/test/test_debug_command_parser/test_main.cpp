#include <unity.h>

#include "observability/DebugCommandParser.h"

void setUp() {}
void tearDown() {}

DebugCommand pushString(
    DebugCommandParser& parser,
    const char* text)
{
    DebugCommand result = DebugCommand::NONE;

    while (*text != '\0')
    {
        const DebugCommand command =
            parser.push(*text++);

        if (command != DebugCommand::NONE)
            result = command;
    }

    return result;
}

void test_dump_fast()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_FAST),
        static_cast<int>(
            pushString(parser, "DUMP FAST\n")));
}

void test_dump_slow()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_SLOW),
        static_cast<int>(
            pushString(parser, "DUMP SLOW\n")));
}

void test_crlf()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_FAST),
        static_cast<int>(
            pushString(parser, "DUMP FAST\r\n")));
}

void test_unknown_command()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::NONE),
        static_cast<int>(
            pushString(parser, "HELLO\n")));
}

void test_command_is_not_returned_before_newline()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::NONE),
        static_cast<int>(
            pushString(parser, "DUMP FAST")));
}

void test_parser_accepts_next_command()
{
    DebugCommandParser parser;

    pushString(parser, "BAD\n");

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_SLOW),
        static_cast<int>(
            pushString(parser, "DUMP SLOW\n")));
}

void test_overflow_discards_entire_line()
{
    DebugCommandParser parser;

    pushString(
        parser,
        "THIS COMMAND IS DEFINITELY LONGER THAN THE BUFFER DUMP FAST\n");

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_FAST),
        static_cast<int>(
            pushString(parser, "DUMP FAST\n")));
}

void test_dump_events()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::DUMP_EVENTS),
        static_cast<int>(
            pushString(parser, "DUMP EVENTS\n")));
}

void test_clear_flight()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::CLEAR_FLIGHT),
        static_cast<int>(
            pushString(parser, "CLEAR FLIGHT\n")));
}

void test_rearm_flight()
{
    DebugCommandParser parser;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(DebugCommand::REARM_FLIGHT),
        static_cast<int>(
            pushString(parser, "REARM FLIGHT\n")));
}

void test_get_params_command()
{
    DebugCommandParser parser;
    DebugCommand command = DebugCommand::NONE;

    const char* input = "GET_PARAMS\n";

    for (const char* p = input; *p != '\0'; ++p)
    {
        command = parser.push(*p);
    }

    TEST_ASSERT_EQUAL(
        static_cast<int>(DebugCommand::GET_PARAMS),
        static_cast<int>(command));
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_dump_fast);
    RUN_TEST(test_dump_slow);
    RUN_TEST(test_crlf);
    RUN_TEST(test_unknown_command);
    RUN_TEST(test_command_is_not_returned_before_newline);
    RUN_TEST(test_parser_accepts_next_command);
    RUN_TEST(test_overflow_discards_entire_line);
    RUN_TEST(test_dump_events);
    RUN_TEST(test_clear_flight);
    RUN_TEST(test_rearm_flight);
    RUN_TEST(test_get_params_command);

    return UNITY_END();
}
