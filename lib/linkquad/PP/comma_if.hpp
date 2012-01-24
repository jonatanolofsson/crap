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
#ifndef PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
#define PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
#
#include "config.hpp"
#include "if.hpp"
#include "empty.hpp"
#include "comma.hpp"
#
# /* PP_COMMA_IF */
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_EDG()
#    define PP_COMMA_IF(cond) PP_IF(cond, PP_COMMA, PP_EMPTY)()
# else
#    define PP_COMMA_IF(cond) PP_COMMA_IF_I(cond)
#    define PP_COMMA_IF_I(cond) PP_IF(cond, PP_COMMA, PP_EMPTY)()
# endif
#
# endif