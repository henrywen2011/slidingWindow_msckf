#ifndef BASE_EXCEPTION_H
#define BASE_EXCEPTION_H

#include <exception>

class BaseException : public std::exception {
public:
    BaseException();
    virtual const char* what() const noexcept = 0;
};

#endif //BASE_EXCEPTION_H
