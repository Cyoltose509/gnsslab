#pragma once

#include <iostream>
#include <utility>
#include <string>
#include <exception>

#include "Log.h"

class BaseException : public std::exception {
public:
    explicit BaseException(std::string msg, const bool doLog = true) : message(std::move(msg)) {
        if (doLog)
            LOG_ERROR << BaseException::what();
    }

    ~BaseException() noexcept override = default;

    [[nodiscard]] const char *what() const noexcept override {
        return message.c_str();
    }

protected:
    std::string message;
};

// ---- 子类自动继承 loggable=true ----
class StringException : public BaseException {
public:
    explicit StringException(std::string msg) : BaseException(std::move(msg)) {
    }
};

class InvalidRequest : public BaseException {
public:
    explicit InvalidRequest(std::string msg) : BaseException(std::move(msg)) {
    }
};

class FFStreamError : public BaseException {
public:
    explicit FFStreamError(const std::string &msg) : BaseException(msg) {
    }
};

// EndOfFile 是流程控制，不记 ERROR
class EndOfFile : public BaseException {
public:
    explicit EndOfFile(const std::string &msg) : BaseException(msg, false) {
    }
};

class FileMissingException : public BaseException {
public:
    explicit FileMissingException(const std::string &msg) : BaseException(msg) {
    }
};

class ConfigException : public BaseException {
public:
    explicit ConfigException(std::string msg) : BaseException(std::move(msg)) {
    }
};

class GeometryException : public BaseException {
public:
    explicit GeometryException(std::string msg) : BaseException(std::move(msg)) {
    }
};

class TypeIDNotFound : public BaseException {
public:
    explicit TypeIDNotFound(const std::string &msg) : BaseException(msg) {
    }
};

class SatIDNotFound : public BaseException {
public:
    explicit SatIDNotFound(const std::string &msg) : BaseException(msg) {
    }
};

class NumberOfSatsMismatch : public BaseException {
public:
    explicit NumberOfSatsMismatch(const std::string &msg) : BaseException(msg) {
    }
};

class NumberOfTypesMismatch : public BaseException {
public:
    explicit NumberOfTypesMismatch(const std::string &msg) : BaseException(msg) {
    }
};

class SVNumException : public BaseException {
public:
    explicit SVNumException(const std::string &msg) : BaseException(msg) {
    }
};

class InvalidSolver : public BaseException {
public:
    explicit InvalidSolver(const std::string &msg) : BaseException(msg) {
    }
};

class SyncException : public BaseException {
public:
    explicit SyncException(const std::string &msg) : BaseException(msg) {
    }
};
