#pragma once
// C++ standard library header file
#include <assert.h>
#include <string>

// Third-party library headers
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/persistence.hpp>
#include <opencv4/opencv2/opencv.hpp>

class FileReader {
 public:
  explicit FileReader(const std::string& file_path);

  ~FileReader();

  template <typename T>
  T Read(const std::string& key) {
    if (!is_open_ || !fs_.isOpened()) {
      throw std::runtime_error("FileReader is not open");
    }

    const char* cstr = key.c_str();
    if (fs_[cstr].empty()) {
      throw std::runtime_error("Key: \"" + key + "\" not found in the file.");
    }
    T value;
    fs_[cstr] >> value;
    return value;
  }

 private:
  const std::string file_path_;
  cv::FileStorage fs_;
  bool is_open_ = false;
};

class FileWriter {
 public:
  explicit FileWriter(const std::string& file_path);

  ~FileWriter();

 private:
  cv::FileStorage fs_;
  std::string file_path_;
  bool is_open_ = false;
};
