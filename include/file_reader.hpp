#pragma once

// C++ standard library header file
#include <assert.h>
#include <iostream>
#include <string>

// Third-party library headers
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/persistence.hpp>
#include <opencv4/opencv2/opencv.hpp>

class FileReader {
 public:
  explicit FileReader(std::string file_path) : file_path_(file_path) {
    cv::FileStorage file_storage(file_path_, cv::FileStorage::READ);

    if (!file_storage.isOpened()) {
      std::string error_msg = "File \"" + file_path + "\" open failed.";
      std::cerr << error_msg << std::endl;
      throw std::runtime_error(error_msg);
    }

    this->fs_ = file_storage;
  }
  ~FileReader() { this->fs_.release(); }

  template <typename T>
  T Read(std::string key) {
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
};

class FileWriter {
 public:
  explicit FileWriter(std::string file_path) : file_path_(file_path) {
    cv::FileStorage file_storage(file_path_, cv::FileStorage::WRITE);

    if (!file_storage.isOpened()) {
      std::string error_msg = "File \"" + file_path + "\" open failed.";
      std::cerr << error_msg << std::endl;
      throw std::runtime_error(error_msg);
    }

    this->fs = file_storage;
  }
  ~FileWriter() { this->fs.release(); }
  cv::FileStorage fs;

 private:
  std::string file_path_;
};
