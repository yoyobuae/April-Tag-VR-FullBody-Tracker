// Allows for public apis to take unicode strings without dealing with wxString.
// Main purpose is to call wxString::FromUTF8Unchecked when passed to a wxString arg in gui code,
// as by default, it will use the current locale.

#pragma once

#include <opencv2/core/persistence.hpp>

#include <string>
#include <string_view>

class wxString;

class U8String
{
public:
    U8String() = default;
    U8String(const char* const _str) : str(_str) {}
    U8String(std::string _str) : str(std::move(_str)) {}
    operator wxString() const;

    U8String& operator+=(const U8String& rhs)
    {
        str += rhs.str;
        return *this;
    }
    U8String& operator+=(const std::string& rhs)
    {
        str += rhs;
        return *this;
    }
    friend U8String operator+(const U8String& lhs, const U8String& rhs) { return lhs.str + rhs.str; }
    friend U8String operator+(const U8String& lhs, const std::string& rhs) { return lhs.str + rhs; }
    friend U8String operator+(const std::string& lhs, const U8String& rhs) { return lhs + rhs.str; }

    friend void Write(cv::FileStorage& fs, const U8String& val) { fs << val.str; }
    friend void Read(const cv::FileNode& fn, U8String& val) { fn >> val.str; }

private:
    std::string str;
};

class U8StringView
{
public:
    constexpr U8StringView(std::string_view _str) : str(std::move(_str)) {}
    constexpr U8StringView(const char* const _str) : str(_str) {}
    operator wxString() const;

private:
    std::string_view str;
};
