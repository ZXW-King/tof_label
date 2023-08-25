//////////////////////////////////////////////////////////////////////
///  @file     type_check.h
///  @brief    check variable type
///  @author   sunhao
///  @date     2021.12.22
//////////////////////////////////////////////////////////////////////
#ifndef DETECTOR_TYPE_CHECK_H
#define DETECTOR_TYPE_CHECK_H

#include <type_traits>

// check T has the Write member function
template<typename T>
struct HasWrite
{
    // if hase Write, call this func, return void
    template<typename U>
    static void check(decltype(&U::Write));

    // else call this func, return int
    template<typename U>
    static int check(...);

    // if get void, it has the Write func
    enum
    {
        value = std::is_void<decltype(check<T>(0))>::value
    };
};

// check T has the Read member function
template<typename T>
struct HasRead
{
    // if hase Write, call this func, return void
    template<typename U>
    static void check(decltype(&U::Read));

    // else call this func, return int
    template<typename U>
    static int check(...);

    // if get void, it has the Write func
    enum
    {
        value = std::is_void<decltype(check<T>(0))>::value
    };
};

#endif //DETECTOR_TYPE_CHECK_H