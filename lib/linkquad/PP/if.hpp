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
# ifndef PREPROCESSOR_CONTROL_IF_HPP
# define PREPROCESSOR_CONTROL_IF_HPP
#
# include "config.hpp"
# include "iif.hpp"
# include "bool.hpp"
#
# /* PP_IF */
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_EDG()
#    define PP_IF(cond, t, f) PP_IIF(PP_BOOL(cond), t, f)
# else
#    define PP_IF(cond, t, f) PP_IF_I(cond, t, f)
#    define PP_IF_I(cond, t, f) PP_IIF(PP_BOOL(cond), t, f)
# endif
#
# endif
