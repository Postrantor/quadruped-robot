#ifndef __ERRORCLASS_H
#define __ERRORCLASS_H

#include <string>
#include <exception>
#include <stdexcept>
#include <sstream>
#include <cstring>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, __LINE__, (message))

class IOException : public std::exception {
  // disable copy constructors
  IOException& operator=(const IOException&);

  std::string file_;
  int line_;
  std::string e_what_;
  int errno_;

public:
  explicit IOException(std::string file, int line, int errnum)
      : file_(file), line_(line), errno_(errnum) {
    std::stringstream ss;
    char* error_str = strerror(errnum);
    ss << "IO Exception (" << errno_ << "): " << error_str;
    ss << ", file " << file_ << ", line " << line_ << ".";
    e_what_ = ss.str();
  }

  explicit IOException(std::string file, int line, const char* description)
      : file_(file), line_(line), errno_(0) {
    std::stringstream ss;
    ss << "IO Exception: " << description;
    ss << ", file " << file_ << ", line " << line_ << ".";
    e_what_ = ss.str();
  }

  virtual ~IOException() throw() {}

  IOException(const IOException& other)
      : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

  int getErrorNumber() const { return errno_; }

  virtual const char* what() const throw() { return e_what_.c_str(); }
};

#endif  // ERRORCLASS_H
