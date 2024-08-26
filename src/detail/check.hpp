#pragma once
#include <sstream>

namespace ob_lidar_driver
{
    inline void LogFatal(const char* file, int line, const std::string& message)
    {
        std::cerr << "Fatal error at " << file << ":" << line << ": " << message << std::endl;
        std::abort();
    }
}

template <typename T1, typename T2>
void CheckOp(const char* expr, const char* file, int line, T1 val1, T2 val2, bool result)
{
    if (!result)
    {
        std::ostringstream os;
        os << "Check failed: " << expr << " (" << val1 << " vs. " << val2 << ")";
        ob_lidar_driver::LogFatal(file, line, os.str());
    }
}

// Macros for checking conditions and comparing values
#define CHECK(condition) \
(!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition) : (void)0)

#define CHECK_OP(opname, op, val1, val2) \
CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2))

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

namespace ob_lidar_driver
{
    // Overload for raw pointers
    template <typename T>
    T* CheckNotNull(T* ptr, const char* file, int line)
    {
        if (ptr == nullptr)
        {
            std::ostringstream os;
            os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
            ob_lidar_driver::LogFatal(file, line, os.str());
        }
        return ptr;
    }

    // Template for smart pointers like std::shared_ptr, std::unique_ptr
    template <typename T>
    T& CheckNotNull(T& ptr, const char* file, int line)
    {
        if (ptr == nullptr)
        {
            std::ostringstream os;
            os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
            LogFatal(file, line, os.str());
        }
        return ptr;
    }
}

#if defined(CHECK_NOTNULL)
#undef CHECK_NOTNULL
#endif
#define CHECK_NOTNULL(val) CheckNotNull(val, __FILE__, __LINE__)
