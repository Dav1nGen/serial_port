#include "file_reader.hpp"

FileReader::FileReader(const std::string& file_path) : file_path_(file_path) {
  cv::FileStorage file_storage(file_path_, cv::FileStorage::READ);

  if (!file_storage.isOpened()) {
    std::string error_msg = "File \"" + file_path + "\" open failed.";
    std::cerr << error_msg << "\n";
    throw std::runtime_error(error_msg);
  }

  fs_ = file_storage;
  is_open_ = true;
}

FileReader::~FileReader() {
  if (is_open_ && fs_.isOpened()) {
    fs_.release();
  }
}

FileWriter::FileWriter(const std::string& file_path) : file_path_(file_path) {
  cv::FileStorage file_storage(file_path_, cv::FileStorage::WRITE);

  if (!file_storage.isOpened()) {
    std::string error_msg = "File \"" + file_path + "\" open failed.";
    std::cerr << error_msg << "\n";
    throw std::runtime_error(error_msg);
  }

  fs_ = file_storage;
  is_open_ = true;
}

FileWriter::~FileWriter() {
  if (is_open_ && fs_.isOpened()) {
    fs_.release();
  }
}
