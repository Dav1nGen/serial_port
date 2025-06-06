#pragma once
// C++ standard library header file
#include <assert.h>
#include <iostream>
#include <string>
#include <utility>

// Third-party library headers
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/persistence.hpp>
#include <opencv4/opencv2/opencv.hpp>

class FileReader {
 public:
  explicit FileReader(const std::string& file_path)
      : file_path_(file_path), is_open_(false) {
    cv::FileStorage file_storage(file_path_, cv::FileStorage::READ);

    if (!file_storage.isOpened()) {
      std::string error_msg = "File \"" + file_path + "\" open failed.";
      std::cerr << error_msg << std::endl;
      throw std::runtime_error(error_msg);
    }

    fs_ = std::move(file_storage);
    is_open_ = true;
  }

  ~FileReader() {
    if (is_open_ && fs_.isOpened()) {
      fs_.release();
    }
  }

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
  bool is_open_;
};

class FileWriter {
 public:
  explicit FileWriter(const std::string& file_path)
      : file_path_(file_path), is_open_(false) {
    cv::FileStorage file_storage(file_path_, cv::FileStorage::WRITE);

    if (!file_storage.isOpened()) {
      std::string error_msg = "File \"" + file_path + "\" open failed.";
      std::cerr << error_msg << std::endl;
      throw std::runtime_error(error_msg);
    }

    fs_ = std::move(file_storage);
    is_open_ = true;
  }

  ~FileWriter() {
    if (is_open_ && fs_.isOpened()) {
      fs_.release();
    }
  }

 private:
  cv::FileStorage fs_;
  std::string file_path_;
  bool is_open_;
};
