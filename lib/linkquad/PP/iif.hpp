# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef PREPROCESSOR_CONTROL_IIF_HPP
# define PREPROCESSOR_CONTROL_IIF_HPP
#
# include "config.hpp"
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_MWCC()
#    define PP_IIF(bit, t, f) PP_IIF_I(bit, t, f)
# else
#    define PP_IIF(bit, t, f) PP_IIF_OO((bit, t, f))
#    define PP_IIF_OO(par) PP_IIF_I ## par
# endif
#
# if ~PP_CONFIG_FLAGS() & PP_CONFIG_MSVC()
#    define PP_IIF_I(bit, t, f) PP_IIF_ ## bit(t, f)
# else
#    define PP_IIF_I(bit, t, f) PP_IIF_II(PP_IIF_ ## bit(t, f))
#    define PP_IIF_II(id) id
# endif
#
# define PP_IIF_0(t, f) f
# define PP_IIF_1(t, f) t
#
# endif
