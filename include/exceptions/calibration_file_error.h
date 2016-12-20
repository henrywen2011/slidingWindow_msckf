#ifndef CALIBRATION_FILE_ERROR_H
#define CALIBRATION_FILE_ERROR_H

#include <string>

#include "exceptions/base_exception.h"

class CalibrationFileError : public BaseException {
public:
    CalibrationFileError(const std::string& msg);
    CalibrationFileError(int line_number, const std::string& msg);

    virtual const char* what() const noexcept;

    virtual ~CalibrationFileError();

private:
    std::string msg_;
};

#endif //CALIBRATION_FILE_ERROR_H
