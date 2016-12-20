#ifndef IMPOSSIBLE_EXCEPTION_H
#define IMPOSSIBLE_EXCEPTION_H

#include <string>

#include "exceptions/base_exception.h"

class ImpossibleException : public BaseException {
public:
    ImpossibleException(const std::string& msg);

    virtual const char* what() const noexcept;

    ~ImpossibleException();

private:
    std::string msg_;
};

#endif //IMPOSSIBLE_EXCEPTION_H
