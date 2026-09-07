#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace route3d_web_console
{

struct ProcessSnapshot
{
  bool running{false};
  int pid{-1};
  int exit_code{0};
  std::deque<std::string> log_lines;
};

class ManagedProcess
{
public:
  using LogCallback = std::function<void(const std::string &, const std::string &)>;

  ManagedProcess(std::string name, std::size_t maximum_log_lines, LogCallback callback);
  ~ManagedProcess();
  ManagedProcess(const ManagedProcess &) = delete;
  ManagedProcess & operator=(const ManagedProcess &) = delete;

  bool start(const std::string & command, const std::map<std::string, std::string> & environment,
    std::string & error);
  bool writeInput(const std::string & data, std::string & error);
  bool stop(std::chrono::milliseconds timeout, std::string & error);
  ProcessSnapshot snapshot() const;

private:
  void readerLoop(int output_fd, int child_pid);
  void appendLog(const std::string & line);

  std::string name_;
  std::size_t maximum_log_lines_;
  LogCallback callback_;
  mutable std::mutex mutex_;
  int pid_{-1};
  int input_fd_{-1};
  int exit_code_{0};
  bool running_{false};
  std::deque<std::string> log_lines_;
  std::thread reader_thread_;
};

class ProcessManager
{
public:
  using LogCallback = ManagedProcess::LogCallback;
  explicit ProcessManager(std::size_t maximum_log_lines, LogCallback callback);
  ~ProcessManager();

  bool start(const std::string & name, const std::string & command,
    const std::map<std::string, std::string> & environment, std::string & error);
  bool writeInput(const std::string & name, const std::string & data, std::string & error);
  bool stop(const std::string & name, std::chrono::milliseconds timeout, std::string & error);
  void stopAll(std::chrono::milliseconds timeout);
  ProcessSnapshot snapshot(const std::string & name) const;

private:
  std::size_t maximum_log_lines_;
  LogCallback callback_;
  mutable std::mutex mutex_;
  std::map<std::string, std::unique_ptr<ManagedProcess>> processes_;
};

}  // namespace route3d_web_console
