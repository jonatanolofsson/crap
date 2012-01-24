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
# ifndef PREPROCESSOR_CAT_HPP
# define PREPROCESSOR_CAT_HPP
#
# include "config.hpp"
#
# /* PP_CAT */
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_MWCC()
#    define PP_CAT(a, b) PP_CAT_I(a, b)
# else
#    define PP_CAT(a, b) PP_CAT_OO((a, b))
#    define PP_CAT_OO(par) PP_CAT_I ## par
# endif
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_MSVC()
#    define PP_CAT_I(a, b) a ## b
# else
#    define PP_CAT_I(a, b) PP_CAT_II(a ## b)
#    define PP_CAT_II(res) res
# endif
#
# endif
