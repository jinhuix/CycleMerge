#pragma once

#ifndef CONST_H
#define CONST_H

#ifdef __cplusplus
extern "C"
{
#endif

#define NORMAL "\x1B[0m"
// #define BLACK "\x1B[30m"
#define RED "\x1B[31m"
#define GREEN "\x1B[32m"
#define YELLOW "\x1B[33m"
#define BLUE "\x1B[34m"
#define MAGENTA "\x1B[35m"
#define CYAN "\x1B[36m"
    // #define WHITE "\x1B[37m"

#define VVVERBOSE_LEVEL 3
#define VVERBOSE_LEVEL 4
#define VERBOSE_LEVEL 5
#define DEBUG_LEVEL 6
#define INFO_LEVEL 7
#define WARN_LEVEL 8
#define SEVERE_LEVEL 9

#ifdef __cplusplus
}
#endif

#endif
