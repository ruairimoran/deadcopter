#include "config.h"

#ifdef DEBUG_MODE

#define debug SERIAL_OBJECT

#define debug_begin(baud) debug.begin(baud);

#define debug_println(string) debug.println(string)

#define debug_print(string) debug.print(string)
#define debug_print2(arg1, arg2) debug_print(arg1), debug_print(arg2)
#define debug_print4(arg1, arg2, arg3, arg4) debug_print2(arg1, arg2), debug_print2(arg3, arg4)
#define debug_print8(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) debug_print4(arg1, arg2, arg3, arg4), \
  debug_print4(arg5, arg6, arg7, arg8)

#define debug_print16(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16) \
  debug_print8(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8), \
  debug_print8(arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16)

// recursive printing
#define P ()
#define E(...) E4(E4(E4(E4(__VA_ARGS__))))
#define E4(...) E3(E3(E3(E3(__VA_ARGS__))))
#define E3(...) E2(E2(E2(E2(__VA_ARGS__))))
#define E2(...) E1(E1(E1(E1(__VA_ARGS__))))
#define E1(...) __VA_ARGS__
#define FOR_EACH(macro, ...) __VA_OPT__(E(FOR_EACH_HELPER(macro, __VA_ARGS__)))
#define FOR_EACH_HELPER(macro, a1, ...) macro(a1) __VA_OPT__(FOR_EACH_AGAIN P(macro, __VA_ARGS__))
#define FOR_EACH_AGAIN() FOR_EACH_HELPER


#define debug_print_all(args...) FOR_EACH(debug_print, args)

#else

#define debug 1
#define debug_begin(baud) 1
#define debug_println(string) 1
#define debug_print(string) 1


#define debug_print2(arg1, arg2) 1
#define debug_print4(arg1, arg2, arg3, arg4) 1
#define debug_print8(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) 1
#define debug_print16(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16) 1


#define debug_print_all(args...)

#endif
