#include "route3d_web_console/process_manager.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <sstream>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

namespace route3d_web_console
{

namespace
{

bool processGroupExists(const int leader)
{
  if (leader <= 0) {return false;}
  if (::kill(-leader, 0) == 0) {return true;}
  return errno == EPERM;
}

}  // namespace

ManagedProcess::ManagedProcess(
  std::string name, const std::size_t maximum_log_lines, LogCallback callback)
: name_(std::move(name)), maximum_log_lines_(maximum_log_lines), callback_(std::move(callback)) {}

ManagedProcess::~ManagedProcess()
{
  std::string ignored;
  stop(std::chrono::milliseconds(1500), ignored);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

bool ManagedProcess::start(
  const std::string & command, const std::map<std::string, std::string> & environment,
  std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      error = name_ + " is already running";
      return false;
    }
  }
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  {
    // A ManagedProcess instance is reused when the same named process is restarted.  Its
    // previous log must not survive into the new run: callers use startup markers from this
    // snapshot to decide when an interactive child is ready for input.
    std::lock_guard<std::mutex> lock(mutex_);
    log_lines_.clear();
  }
  int pipe_fds[2];
  int input_fds[2];
  if (::pipe(pipe_fds) != 0) {
    error = std::string("pipe failed: ") + std::strerror(errno);
    return false;
  }
  if (::pipe(input_fds) != 0) {
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    error = std::string("stdin pipe failed: ") + std::strerror(errno);
    return false;
  }
  const pid_t child = ::fork();
  if (child < 0) {
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    ::close(input_fds[0]);
    ::close(input_fds[1]);
    error = std::string("fork failed: ") + std::strerror(errno);
    return false;
  }
  if (child == 0) {
    ::setsid();
    ::dup2(input_fds[0], STDIN_FILENO);
    ::dup2(pipe_fds[1], STDOUT_FILENO);
    ::dup2(pipe_fds[1], STDERR_FILENO);
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    ::close(input_fds[0]);
    ::close(input_fds[1]);
    // The web server and ROS node have many sockets open. A forked launcher must not inherit
    // them across exec; otherwise a stopped recording/localization child can keep the HTTP
    // listen socket alive and make a later web-console restart appear successful while the
    // old endpoint still owns the port.
#ifdef SYS_close_range
    ::syscall(SYS_close_range, 3U, ~0U, 0U);
#else
    for (int fd = 3; fd < 1024; ++fd) {::close(fd);}
#endif
    for (const auto & entry : environment) {
      ::setenv(entry.first.c_str(), entry.second.c_str(), 1);
    }
    ::execl("/bin/bash", "bash", "-lc", command.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }
  ::close(pipe_fds[1]);
  ::close(input_fds[0]);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pid_ = child;
    input_fd_ = input_fds[1];
    running_ = true;
    exit_code_ = 0;
  }
  appendLog("$ " + command);
  reader_thread_ = std::thread(&ManagedProcess::readerLoop, this, pipe_fds[0], child);
  return true;
}

bool ManagedProcess::writeInput(const std::string & data, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || input_fd_ < 0) {
    error = name_ + " is not running or has no input pipe";
    return false;
  }
  std::size_t offset = 0;
  while (offset < data.size()) {
    const auto written = ::write(input_fd_, data.data() + offset, data.size() - offset);
    if (written < 0) {
      if (errno == EINTR) {continue;}
      error = std::string("write stdin failed: ") + std::strerror(errno);
      return false;
    }
    offset += static_cast<std::size_t>(written);
  }
  return true;
}

bool ManagedProcess::stop(
  const std::chrono::milliseconds timeout, std::string & error)
{
  int child = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return true;
    }
    child = pid_;
    if (input_fd_ >= 0) {
      ::close(input_fd_);
      input_fd_ = -1;
    }
  }
  if (::kill(-child, SIGINT) != 0 && errno != ESRCH) {
    error = std::string("SIGINT failed: ") + std::strerror(errno);
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    // The launcher shell can exit before the ROS processes it spawned.  Checking only
    // running_ would then report a successful stop while leaving those descendants alive.
    if (!processGroupExists(child)) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (processGroupExists(child)) {::kill(-child, SIGTERM);}
  const auto terminate_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < terminate_deadline) {
    if (!processGroupExists(child)) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  bool forced = false;
  if (processGroupExists(child)) {
    if (::kill(-child, SIGKILL) != 0 && errno != ESRCH) {
      error = std::string("SIGKILL failed: ") + std::strerror(errno);
    } else {
      forced = true;
    }
  }
  const auto kill_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (processGroupExists(child) && std::chrono::steady_clock::now() < kill_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (processGroupExists(child) && error.empty()) {
    error = name_ + " process group is still alive after SIGKILL";
  } else if (forced) {
    appendLog("[process group required SIGKILL and was stopped]");
  }
  if (reader_thread_.joinable() && reader_thread_.get_id() != std::this_thread::get_id()) {
    reader_thread_.join();
  }
  return error.empty();
}

ProcessSnapshot ManagedProcess::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ProcessSnapshot{running_, pid_, exit_code_, log_lines_};
}

void ManagedProcess::readerLoop(const int output_fd, const int child_pid)
{
  FILE * stream = ::fdopen(output_fd, "r");
  if (stream != nullptr) {
    char * line = nullptr;
    std::size_t capacity = 0;
    while (::getline(&line, &capacity, stream) >= 0) {
      std::string value(line);
      while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) {
        value.pop_back();
      }
      appendLog(value);
    }
    std::free(line);
    ::fclose(stream);
  } else {
    ::close(output_fd);
  }
  int status = 0;
  while (::waitpid(child_pid, &status, 0) < 0 && errno == EINTR) {}
  int code = -1;
  if (WIFEXITED(status)) {
    code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    code = 128 + WTERMSIG(status);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    exit_code_ = code;
    running_ = false;
    pid_ = -1;
    if (input_fd_ >= 0) {
      ::close(input_fd_);
      input_fd_ = -1;
    }
  }
  appendLog("[process exited with code " + std::to_string(code) + "]");
}

void ManagedProcess::appendLog(const std::string & line)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    log_lines_.push_back(line);
    while (log_lines_.size() > maximum_log_lines_) {
      log_lines_.pop_front();
    }
  }
  if (callback_) {
    callback_(name_, line);
  }
}

ProcessManager::ProcessManager(const std::size_t maximum_log_lines, LogCallback callback)
: maximum_log_lines_(maximum_log_lines), callback_(std::move(callback)) {}

ProcessManager::~ProcessManager() {stopAll(std::chrono::milliseconds(1500));}

bool ProcessManager::start(
  const std::string & name, const std::string & command,
  const std::map<std::string, std::string> & environment, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & process = processes_[name];
  if (!process) {
    process = std::make_unique<ManagedProcess>(name, maximum_log_lines_, callback_);
  }
  return process->start(command, environment, error);
}

bool ProcessManager::stop(
  const std::string & name, const std::chrono::milliseconds timeout, std::string & error)
{
  ManagedProcess * process = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = processes_.find(name);
    if (found == processes_.end()) {
      return true;
    }
    process = found->second.get();
  }
  return process->stop(timeout, error);
}

bool ProcessManager::writeInput(
  const std::string & name, const std::string & data, std::string & error)
{
  ManagedProcess * process = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = processes_.find(name);
    if (found == processes_.end()) {
      error = "unknown process: " + name;
      return false;
    }
    process = found->second.get();
  }
  return process->writeInput(data, error);
}

void ProcessManager::stopAll(const std::chrono::milliseconds timeout)
{
  std::vector<ManagedProcess *> values;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & entry : processes_) {
      values.push_back(entry.second.get());
    }
  }
  for (auto * process : values) {
    std::string ignored;
    process->stop(timeout, ignored);
  }
}

ProcessSnapshot ProcessManager::snapshot(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = processes_.find(name);
  return found == processes_.end() ? ProcessSnapshot{} : found->second->snapshot();
}

}  // namespace route3d_web_console
