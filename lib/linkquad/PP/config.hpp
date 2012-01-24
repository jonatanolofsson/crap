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
# ifndef PREPROCESSOR_CONFIG_CONFIG_HPP
# define PREPROCESSOR_CONFIG_CONFIG_HPP
#
# /* PP_CONFIG_FLAGS */
#
# define PP_CONFIG_STRICT() 0x0001
# define PP_CONFIG_IDEAL() 0x0002
#
# define PP_CONFIG_MSVC() 0x0004
# define PP_CONFIG_MWCC() 0x0008
# define PP_CONFIG_BCC() 0x0010
# define PP_CONFIG_EDG() 0x0020
# define PP_CONFIG_DMC() 0x0040
#
# ifndef PP_CONFIG_FLAGS
#    if defined(__GCCXML__)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_STRICT())
#    elif defined(__WAVE__)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_STRICT())
#    elif defined(__MWERKS__) && __MWERKS__ >= 0x3200
#        define PP_CONFIG_FLAGS() (PP_CONFIG_STRICT())
#    elif defined(__EDG__) || defined(__EDG_VERSION__)
#        if defined(_MSC_VER) && __EDG_VERSION__ >= 308
#            define PP_CONFIG_FLAGS() (PP_CONFIG_MSVC())
#        else
#            define PP_CONFIG_FLAGS() (PP_CONFIG_EDG() | PP_CONFIG_STRICT())
#        endif
#    elif defined(__MWERKS__)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_MWCC())
#    elif defined(__DMC__)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_DMC())
#    elif defined(__BORLANDC__) && __BORLANDC__ >= 0x581
#        define PP_CONFIG_FLAGS() (PP_CONFIG_STRICT())
#    elif defined(__BORLANDC__) || defined(__IBMC__) || defined(__IBMCPP__) || defined(__SUNPRO_CC)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_BCC())
#    elif defined(_MSC_VER)
#        define PP_CONFIG_FLAGS() (PP_CONFIG_MSVC())
#    else
#        define PP_CONFIG_FLAGS() (PP_CONFIG_STRICT())
#    endif
# endif
#
# /* PP_CONFIG_EXTENDED_LINE_INFO */
#
# ifndef PP_CONFIG_EXTENDED_LINE_INFO
#    define PP_CONFIG_EXTENDED_LINE_INFO 0
# endif
#
# /* PP_CONFIG_ERRORS */
#
# ifndef PP_CONFIG_ERRORS
#    ifdef NDEBUG
#        define PP_CONFIG_ERRORS 0
#    else
#        define PP_CONFIG_ERRORS 1
#    endif
# endif
#
# endif
