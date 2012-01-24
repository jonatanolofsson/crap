# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef PREPROCESSOR_TUPLE_EAT_HPP
# define PREPROCESSOR_TUPLE_EAT_HPP
#
# include "config.hpp"
#
# /* PP_TUPLE_EAT */
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_MWCC()
#    define PP_TUPLE_EAT(size) PP_TUPLE_EAT_I(size)
# else
#    define PP_TUPLE_EAT(size) PP_TUPLE_EAT_OO((size))
#    define PP_TUPLE_EAT_OO(par) PP_TUPLE_EAT_I ## par
# endif
#
# define PP_TUPLE_EAT_I(size) PP_TUPLE_EAT_ ## size
#
# define PP_TUPLE_EAT_0()
# define PP_TUPLE_EAT_1(a)
# define PP_TUPLE_EAT_2(a, b)
# define PP_TUPLE_EAT_3(a, b, c)
# define PP_TUPLE_EAT_4(a, b, c, d)
# define PP_TUPLE_EAT_5(a, b, c, d, e)
# define PP_TUPLE_EAT_6(a, b, c, d, e, f)
# define PP_TUPLE_EAT_7(a, b, c, d, e, f, g)
# define PP_TUPLE_EAT_8(a, b, c, d, e, f, g, h)
# define PP_TUPLE_EAT_9(a, b, c, d, e, f, g, h, i)
# define PP_TUPLE_EAT_10(a, b, c, d, e, f, g, h, i, j)
# define PP_TUPLE_EAT_11(a, b, c, d, e, f, g, h, i, j, k)
# define PP_TUPLE_EAT_12(a, b, c, d, e, f, g, h, i, j, k, l)
# define PP_TUPLE_EAT_13(a, b, c, d, e, f, g, h, i, j, k, l, m)
# define PP_TUPLE_EAT_14(a, b, c, d, e, f, g, h, i, j, k, l, m, n)
# define PP_TUPLE_EAT_15(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o)
# define PP_TUPLE_EAT_16(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p)
# define PP_TUPLE_EAT_17(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q)
# define PP_TUPLE_EAT_18(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r)
# define PP_TUPLE_EAT_19(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s)
# define PP_TUPLE_EAT_20(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t)
# define PP_TUPLE_EAT_21(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u)
# define PP_TUPLE_EAT_22(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v)
# define PP_TUPLE_EAT_23(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w)
# define PP_TUPLE_EAT_24(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x)
# define PP_TUPLE_EAT_25(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y)
#
# endif
