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
# ifndef PREPROCESSOR_DEBUG_ERROR_HPP
# define PREPROCESSOR_DEBUG_ERROR_HPP
#
# include "cat.hpp"
# include "config.hpp"
#
# /* PP_ERROR */
#
# if PP_CONFIG_ERRORS
#    define PP_ERROR(code) PP_CAT(PP_ERROR_, code)
# endif
#
# define PP_ERROR_0x0000 PP_ERROR(0x0000, PP_INDEX_OUT_OF_BOUNDS)
# define PP_ERROR_0x0001 PP_ERROR(0x0001, PP_WHILE_OVERFLOW)
# define PP_ERROR_0x0002 PP_ERROR(0x0002, PP_FOR_OVERFLOW)
# define PP_ERROR_0x0003 PP_ERROR(0x0003, PP_REPEAT_OVERFLOW)
# define PP_ERROR_0x0004 PP_ERROR(0x0004, PP_LIST_FOLD_OVERFLOW)
# define PP_ERROR_0x0005 PP_ERROR(0x0005, PP_SEQ_FOLD_OVERFLOW)
# define PP_ERROR_0x0006 PP_ERROR(0x0006, PP_ARITHMETIC_OVERFLOW)
# define PP_ERROR_0x0007 PP_ERROR(0x0007, PP_DIVISION_BY_ZERO)
#
# endif
