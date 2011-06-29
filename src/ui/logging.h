/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _LOGGING_H

#include <cstdarg>

#define LOGGING_FUNCTION(funcname, level) \
    void funcname(const char* format, ...) { \
        va_list arglist;                     \
        if (m_args.verbosity < level)          \
            return;                          \
        va_start(arglist, format);           \
        vfprintf(stderr, format, arglist);   \
        fprintf(stderr, "\n");               \
        va_end(arglist);                     \
    }

#endif       // _LOGGING_H
