#include <gtest/gtest.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "route3d_web_console/process_manager.hpp"

using namespace std::chrono_literals;

TEST(ProcessManager, StopsDescendantsAfterLauncherExits)
{
  route3d_web_console::ProcessManager manager(20, nullptr);
  std::string error;

  // A non-interactive shell's asynchronous child ignores SIGINT.  This reproduces ROS launch
  // exiting before one of its nodes and verifies that stop() continues with SIGTERM for the
  // entire process group instead of trusting only the launcher status.
  ASSERT_TRUE(manager.start(
      "test", "trap 'exit 0' INT; trap '' TERM; sleep 30 & wait",
      std::map<std::string, std::string>{}, error)) << error;
  std::this_thread::sleep_for(100ms);
  const int process_group = manager.snapshot("test").pid;
  ASSERT_GT(process_group, 0);

  EXPECT_TRUE(manager.stop("test", 250ms, error)) << error;
  EXPECT_FALSE(manager.snapshot("test").running);
  EXPECT_EQ(::kill(-process_group, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(ProcessManager, WritesToManagedProcessStdin)
{
  std::mutex mutex;
  std::string output;
  route3d_web_console::ProcessManager manager(20,
    [&](const std::string &, const std::string & line) {
      std::lock_guard<std::mutex> lock(mutex);
      output += line;
    });
  std::string error;
  ASSERT_TRUE(manager.start("stdin", "IFS= read -r value; echo got:$value", {}, error)) << error;
  ASSERT_TRUE(manager.writeInput("stdin", "hello\n", error)) << error;
  for (int i = 0; i < 50 && manager.snapshot("stdin").running; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_NE(output.find("got:hello"), std::string::npos);
}

TEST(ProcessManager, ClearsLogsWhenNamedProcessRestarts)
{
  route3d_web_console::ProcessManager manager(20, nullptr);
  std::string error;
  ASSERT_TRUE(manager.start("restart", "echo first-run-marker", {}, error)) << error;
  for (int i = 0; i < 50 && manager.snapshot("restart").running; ++i) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_FALSE(manager.snapshot("restart").running);

  ASSERT_TRUE(manager.start("restart", "echo second-run-marker; sleep 30", {}, error)) << error;
  const auto restarted = manager.snapshot("restart");
  for (const auto & line : restarted.log_lines) {
    EXPECT_EQ(line.find("first-run-marker"), std::string::npos);
  }
  EXPECT_TRUE(manager.stop("restart", 250ms, error)) << error;
}
