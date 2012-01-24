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
# include "config.hpp"
#
# if PP_CONFIG_FLAGS() & PP_CONFIG_DMC()
#     include "dmc/auto_rec.hpp"
# else
#
# ifndef PREPROCESSOR_DETAIL_AUTO_REC_HPP
# define PREPROCESSOR_DETAIL_AUTO_REC_HPP
#
# include "iif.hpp"
#
# /* PP_AUTO_REC */
#
# define PP_AUTO_REC(pred, n) PP_NODE_ENTRY_ ## n(pred)
#
# define PP_NODE_ENTRY_256(p) PP_NODE_128(p)(p)(p)(p)(p)(p)(p)(p)
# define PP_NODE_ENTRY_128(p) PP_NODE_64(p)(p)(p)(p)(p)(p)(p)
# define PP_NODE_ENTRY_64(p) PP_NODE_32(p)(p)(p)(p)(p)(p)
# define PP_NODE_ENTRY_32(p) PP_NODE_16(p)(p)(p)(p)(p)
# define PP_NODE_ENTRY_16(p) PP_NODE_8(p)(p)(p)(p)
# define PP_NODE_ENTRY_8(p) PP_NODE_4(p)(p)(p)
# define PP_NODE_ENTRY_4(p) PP_NODE_2(p)(p)
# define PP_NODE_ENTRY_2(p) PP_NODE_1(p)
#
# define PP_NODE_128(p) PP_IIF(p(128), PP_NODE_64, PP_NODE_192)
#    define PP_NODE_64(p) PP_IIF(p(64), PP_NODE_32, PP_NODE_96)
#        define PP_NODE_32(p) PP_IIF(p(32), PP_NODE_16, PP_NODE_48)
#            define PP_NODE_16(p) PP_IIF(p(16), PP_NODE_8, PP_NODE_24)
#                define PP_NODE_8(p) PP_IIF(p(8), PP_NODE_4, PP_NODE_12)
#                    define PP_NODE_4(p) PP_IIF(p(4), PP_NODE_2, PP_NODE_6)
#                        define PP_NODE_2(p) PP_IIF(p(2), PP_NODE_1, PP_NODE_3)
#                            define PP_NODE_1(p) PP_IIF(p(1), 1, 2)
#                            define PP_NODE_3(p) PP_IIF(p(3), 3, 4)
#                        define PP_NODE_6(p) PP_IIF(p(6), PP_NODE_5, PP_NODE_7)
#                            define PP_NODE_5(p) PP_IIF(p(5), 5, 6)
#                            define PP_NODE_7(p) PP_IIF(p(7), 7, 8)
#                    define PP_NODE_12(p) PP_IIF(p(12), PP_NODE_10, PP_NODE_14)
#                        define PP_NODE_10(p) PP_IIF(p(10), PP_NODE_9, PP_NODE_11)
#                            define PP_NODE_9(p) PP_IIF(p(9), 9, 10)
#                            define PP_NODE_11(p) PP_IIF(p(11), 11, 12)
#                        define PP_NODE_14(p) PP_IIF(p(14), PP_NODE_13, PP_NODE_15)
#                            define PP_NODE_13(p) PP_IIF(p(13), 13, 14)
#                            define PP_NODE_15(p) PP_IIF(p(15), 15, 16)
#                define PP_NODE_24(p) PP_IIF(p(24), PP_NODE_20, PP_NODE_28)
#                    define PP_NODE_20(p) PP_IIF(p(20), PP_NODE_18, PP_NODE_22)
#                        define PP_NODE_18(p) PP_IIF(p(18), PP_NODE_17, PP_NODE_19)
#                            define PP_NODE_17(p) PP_IIF(p(17), 17, 18)
#                            define PP_NODE_19(p) PP_IIF(p(19), 19, 20)
#                        define PP_NODE_22(p) PP_IIF(p(22), PP_NODE_21, PP_NODE_23)
#                            define PP_NODE_21(p) PP_IIF(p(21), 21, 22)
#                            define PP_NODE_23(p) PP_IIF(p(23), 23, 24)
#                    define PP_NODE_28(p) PP_IIF(p(28), PP_NODE_26, PP_NODE_30)
#                        define PP_NODE_26(p) PP_IIF(p(26), PP_NODE_25, PP_NODE_27)
#                            define PP_NODE_25(p) PP_IIF(p(25), 25, 26)
#                            define PP_NODE_27(p) PP_IIF(p(27), 27, 28)
#                        define PP_NODE_30(p) PP_IIF(p(30), PP_NODE_29, PP_NODE_31)
#                            define PP_NODE_29(p) PP_IIF(p(29), 29, 30)
#                            define PP_NODE_31(p) PP_IIF(p(31), 31, 32)
#            define PP_NODE_48(p) PP_IIF(p(48), PP_NODE_40, PP_NODE_56)
#                define PP_NODE_40(p) PP_IIF(p(40), PP_NODE_36, PP_NODE_44)
#                    define PP_NODE_36(p) PP_IIF(p(36), PP_NODE_34, PP_NODE_38)
#                        define PP_NODE_34(p) PP_IIF(p(34), PP_NODE_33, PP_NODE_35)
#                            define PP_NODE_33(p) PP_IIF(p(33), 33, 34)
#                            define PP_NODE_35(p) PP_IIF(p(35), 35, 36)
#                        define PP_NODE_38(p) PP_IIF(p(38), PP_NODE_37, PP_NODE_39)
#                            define PP_NODE_37(p) PP_IIF(p(37), 37, 38)
#                            define PP_NODE_39(p) PP_IIF(p(39), 39, 40)
#                    define PP_NODE_44(p) PP_IIF(p(44), PP_NODE_42, PP_NODE_46)
#                        define PP_NODE_42(p) PP_IIF(p(42), PP_NODE_41, PP_NODE_43)
#                            define PP_NODE_41(p) PP_IIF(p(41), 41, 42)
#                            define PP_NODE_43(p) PP_IIF(p(43), 43, 44)
#                        define PP_NODE_46(p) PP_IIF(p(46), PP_NODE_45, PP_NODE_47)
#                            define PP_NODE_45(p) PP_IIF(p(45), 45, 46)
#                            define PP_NODE_47(p) PP_IIF(p(47), 47, 48)
#                define PP_NODE_56(p) PP_IIF(p(56), PP_NODE_52, PP_NODE_60)
#                    define PP_NODE_52(p) PP_IIF(p(52), PP_NODE_50, PP_NODE_54)
#                        define PP_NODE_50(p) PP_IIF(p(50), PP_NODE_49, PP_NODE_51)
#                            define PP_NODE_49(p) PP_IIF(p(49), 49, 50)
#                            define PP_NODE_51(p) PP_IIF(p(51), 51, 52)
#                        define PP_NODE_54(p) PP_IIF(p(54), PP_NODE_53, PP_NODE_55)
#                            define PP_NODE_53(p) PP_IIF(p(53), 53, 54)
#                            define PP_NODE_55(p) PP_IIF(p(55), 55, 56)
#                    define PP_NODE_60(p) PP_IIF(p(60), PP_NODE_58, PP_NODE_62)
#                        define PP_NODE_58(p) PP_IIF(p(58), PP_NODE_57, PP_NODE_59)
#                            define PP_NODE_57(p) PP_IIF(p(57), 57, 58)
#                            define PP_NODE_59(p) PP_IIF(p(59), 59, 60)
#                        define PP_NODE_62(p) PP_IIF(p(62), PP_NODE_61, PP_NODE_63)
#                            define PP_NODE_61(p) PP_IIF(p(61), 61, 62)
#                            define PP_NODE_63(p) PP_IIF(p(63), 63, 64)
#        define PP_NODE_96(p) PP_IIF(p(96), PP_NODE_80, PP_NODE_112)
#            define PP_NODE_80(p) PP_IIF(p(80), PP_NODE_72, PP_NODE_88)
#                define PP_NODE_72(p) PP_IIF(p(72), PP_NODE_68, PP_NODE_76)
#                    define PP_NODE_68(p) PP_IIF(p(68), PP_NODE_66, PP_NODE_70)
#                        define PP_NODE_66(p) PP_IIF(p(66), PP_NODE_65, PP_NODE_67)
#                            define PP_NODE_65(p) PP_IIF(p(65), 65, 66)
#                            define PP_NODE_67(p) PP_IIF(p(67), 67, 68)
#                        define PP_NODE_70(p) PP_IIF(p(70), PP_NODE_69, PP_NODE_71)
#                            define PP_NODE_69(p) PP_IIF(p(69), 69, 70)
#                            define PP_NODE_71(p) PP_IIF(p(71), 71, 72)
#                    define PP_NODE_76(p) PP_IIF(p(76), PP_NODE_74, PP_NODE_78)
#                        define PP_NODE_74(p) PP_IIF(p(74), PP_NODE_73, PP_NODE_75)
#                            define PP_NODE_73(p) PP_IIF(p(73), 73, 74)
#                            define PP_NODE_75(p) PP_IIF(p(75), 75, 76)
#                        define PP_NODE_78(p) PP_IIF(p(78), PP_NODE_77, PP_NODE_79)
#                            define PP_NODE_77(p) PP_IIF(p(77), 77, 78)
#                            define PP_NODE_79(p) PP_IIF(p(79), 79, 80)
#                define PP_NODE_88(p) PP_IIF(p(88), PP_NODE_84, PP_NODE_92)
#                    define PP_NODE_84(p) PP_IIF(p(84), PP_NODE_82, PP_NODE_86)
#                        define PP_NODE_82(p) PP_IIF(p(82), PP_NODE_81, PP_NODE_83)
#                            define PP_NODE_81(p) PP_IIF(p(81), 81, 82)
#                            define PP_NODE_83(p) PP_IIF(p(83), 83, 84)
#                        define PP_NODE_86(p) PP_IIF(p(86), PP_NODE_85, PP_NODE_87)
#                            define PP_NODE_85(p) PP_IIF(p(85), 85, 86)
#                            define PP_NODE_87(p) PP_IIF(p(87), 87, 88)
#                    define PP_NODE_92(p) PP_IIF(p(92), PP_NODE_90, PP_NODE_94)
#                        define PP_NODE_90(p) PP_IIF(p(90), PP_NODE_89, PP_NODE_91)
#                            define PP_NODE_89(p) PP_IIF(p(89), 89, 90)
#                            define PP_NODE_91(p) PP_IIF(p(91), 91, 92)
#                        define PP_NODE_94(p) PP_IIF(p(94), PP_NODE_93, PP_NODE_95)
#                            define PP_NODE_93(p) PP_IIF(p(93), 93, 94)
#                            define PP_NODE_95(p) PP_IIF(p(95), 95, 96)
#            define PP_NODE_112(p) PP_IIF(p(112), PP_NODE_104, PP_NODE_120)
#                define PP_NODE_104(p) PP_IIF(p(104), PP_NODE_100, PP_NODE_108)
#                    define PP_NODE_100(p) PP_IIF(p(100), PP_NODE_98, PP_NODE_102)
#                        define PP_NODE_98(p) PP_IIF(p(98), PP_NODE_97, PP_NODE_99)
#                            define PP_NODE_97(p) PP_IIF(p(97), 97, 98)
#                            define PP_NODE_99(p) PP_IIF(p(99), 99, 100)
#                        define PP_NODE_102(p) PP_IIF(p(102), PP_NODE_101, PP_NODE_103)
#                            define PP_NODE_101(p) PP_IIF(p(101), 101, 102)
#                            define PP_NODE_103(p) PP_IIF(p(103), 103, 104)
#                    define PP_NODE_108(p) PP_IIF(p(108), PP_NODE_106, PP_NODE_110)
#                        define PP_NODE_106(p) PP_IIF(p(106), PP_NODE_105, PP_NODE_107)
#                            define PP_NODE_105(p) PP_IIF(p(105), 105, 106)
#                            define PP_NODE_107(p) PP_IIF(p(107), 107, 108)
#                        define PP_NODE_110(p) PP_IIF(p(110), PP_NODE_109, PP_NODE_111)
#                            define PP_NODE_109(p) PP_IIF(p(109), 109, 110)
#                            define PP_NODE_111(p) PP_IIF(p(111), 111, 112)
#                define PP_NODE_120(p) PP_IIF(p(120), PP_NODE_116, PP_NODE_124)
#                    define PP_NODE_116(p) PP_IIF(p(116), PP_NODE_114, PP_NODE_118)
#                        define PP_NODE_114(p) PP_IIF(p(114), PP_NODE_113, PP_NODE_115)
#                            define PP_NODE_113(p) PP_IIF(p(113), 113, 114)
#                            define PP_NODE_115(p) PP_IIF(p(115), 115, 116)
#                        define PP_NODE_118(p) PP_IIF(p(118), PP_NODE_117, PP_NODE_119)
#                            define PP_NODE_117(p) PP_IIF(p(117), 117, 118)
#                            define PP_NODE_119(p) PP_IIF(p(119), 119, 120)
#                    define PP_NODE_124(p) PP_IIF(p(124), PP_NODE_122, PP_NODE_126)
#                        define PP_NODE_122(p) PP_IIF(p(122), PP_NODE_121, PP_NODE_123)
#                            define PP_NODE_121(p) PP_IIF(p(121), 121, 122)
#                            define PP_NODE_123(p) PP_IIF(p(123), 123, 124)
#                        define PP_NODE_126(p) PP_IIF(p(126), PP_NODE_125, PP_NODE_127)
#                            define PP_NODE_125(p) PP_IIF(p(125), 125, 126)
#                            define PP_NODE_127(p) PP_IIF(p(127), 127, 128)
#    define PP_NODE_192(p) PP_IIF(p(192), PP_NODE_160, PP_NODE_224)
#        define PP_NODE_160(p) PP_IIF(p(160), PP_NODE_144, PP_NODE_176)
#            define PP_NODE_144(p) PP_IIF(p(144), PP_NODE_136, PP_NODE_152)
#                define PP_NODE_136(p) PP_IIF(p(136), PP_NODE_132, PP_NODE_140)
#                    define PP_NODE_132(p) PP_IIF(p(132), PP_NODE_130, PP_NODE_134)
#                        define PP_NODE_130(p) PP_IIF(p(130), PP_NODE_129, PP_NODE_131)
#                            define PP_NODE_129(p) PP_IIF(p(129), 129, 130)
#                            define PP_NODE_131(p) PP_IIF(p(131), 131, 132)
#                        define PP_NODE_134(p) PP_IIF(p(134), PP_NODE_133, PP_NODE_135)
#                            define PP_NODE_133(p) PP_IIF(p(133), 133, 134)
#                            define PP_NODE_135(p) PP_IIF(p(135), 135, 136)
#                    define PP_NODE_140(p) PP_IIF(p(140), PP_NODE_138, PP_NODE_142)
#                        define PP_NODE_138(p) PP_IIF(p(138), PP_NODE_137, PP_NODE_139)
#                            define PP_NODE_137(p) PP_IIF(p(137), 137, 138)
#                            define PP_NODE_139(p) PP_IIF(p(139), 139, 140)
#                        define PP_NODE_142(p) PP_IIF(p(142), PP_NODE_141, PP_NODE_143)
#                            define PP_NODE_141(p) PP_IIF(p(141), 141, 142)
#                            define PP_NODE_143(p) PP_IIF(p(143), 143, 144)
#                define PP_NODE_152(p) PP_IIF(p(152), PP_NODE_148, PP_NODE_156)
#                    define PP_NODE_148(p) PP_IIF(p(148), PP_NODE_146, PP_NODE_150)
#                        define PP_NODE_146(p) PP_IIF(p(146), PP_NODE_145, PP_NODE_147)
#                            define PP_NODE_145(p) PP_IIF(p(145), 145, 146)
#                            define PP_NODE_147(p) PP_IIF(p(147), 147, 148)
#                        define PP_NODE_150(p) PP_IIF(p(150), PP_NODE_149, PP_NODE_151)
#                            define PP_NODE_149(p) PP_IIF(p(149), 149, 150)
#                            define PP_NODE_151(p) PP_IIF(p(151), 151, 152)
#                    define PP_NODE_156(p) PP_IIF(p(156), PP_NODE_154, PP_NODE_158)
#                        define PP_NODE_154(p) PP_IIF(p(154), PP_NODE_153, PP_NODE_155)
#                            define PP_NODE_153(p) PP_IIF(p(153), 153, 154)
#                            define PP_NODE_155(p) PP_IIF(p(155), 155, 156)
#                        define PP_NODE_158(p) PP_IIF(p(158), PP_NODE_157, PP_NODE_159)
#                            define PP_NODE_157(p) PP_IIF(p(157), 157, 158)
#                            define PP_NODE_159(p) PP_IIF(p(159), 159, 160)
#            define PP_NODE_176(p) PP_IIF(p(176), PP_NODE_168, PP_NODE_184)
#                define PP_NODE_168(p) PP_IIF(p(168), PP_NODE_164, PP_NODE_172)
#                    define PP_NODE_164(p) PP_IIF(p(164), PP_NODE_162, PP_NODE_166)
#                        define PP_NODE_162(p) PP_IIF(p(162), PP_NODE_161, PP_NODE_163)
#                            define PP_NODE_161(p) PP_IIF(p(161), 161, 162)
#                            define PP_NODE_163(p) PP_IIF(p(163), 163, 164)
#                        define PP_NODE_166(p) PP_IIF(p(166), PP_NODE_165, PP_NODE_167)
#                            define PP_NODE_165(p) PP_IIF(p(165), 165, 166)
#                            define PP_NODE_167(p) PP_IIF(p(167), 167, 168)
#                    define PP_NODE_172(p) PP_IIF(p(172), PP_NODE_170, PP_NODE_174)
#                        define PP_NODE_170(p) PP_IIF(p(170), PP_NODE_169, PP_NODE_171)
#                            define PP_NODE_169(p) PP_IIF(p(169), 169, 170)
#                            define PP_NODE_171(p) PP_IIF(p(171), 171, 172)
#                        define PP_NODE_174(p) PP_IIF(p(174), PP_NODE_173, PP_NODE_175)
#                            define PP_NODE_173(p) PP_IIF(p(173), 173, 174)
#                            define PP_NODE_175(p) PP_IIF(p(175), 175, 176)
#                define PP_NODE_184(p) PP_IIF(p(184), PP_NODE_180, PP_NODE_188)
#                    define PP_NODE_180(p) PP_IIF(p(180), PP_NODE_178, PP_NODE_182)
#                        define PP_NODE_178(p) PP_IIF(p(178), PP_NODE_177, PP_NODE_179)
#                            define PP_NODE_177(p) PP_IIF(p(177), 177, 178)
#                            define PP_NODE_179(p) PP_IIF(p(179), 179, 180)
#                        define PP_NODE_182(p) PP_IIF(p(182), PP_NODE_181, PP_NODE_183)
#                            define PP_NODE_181(p) PP_IIF(p(181), 181, 182)
#                            define PP_NODE_183(p) PP_IIF(p(183), 183, 184)
#                    define PP_NODE_188(p) PP_IIF(p(188), PP_NODE_186, PP_NODE_190)
#                        define PP_NODE_186(p) PP_IIF(p(186), PP_NODE_185, PP_NODE_187)
#                            define PP_NODE_185(p) PP_IIF(p(185), 185, 186)
#                            define PP_NODE_187(p) PP_IIF(p(187), 187, 188)
#                        define PP_NODE_190(p) PP_IIF(p(190), PP_NODE_189, PP_NODE_191)
#                            define PP_NODE_189(p) PP_IIF(p(189), 189, 190)
#                            define PP_NODE_191(p) PP_IIF(p(191), 191, 192)
#        define PP_NODE_224(p) PP_IIF(p(224), PP_NODE_208, PP_NODE_240)
#            define PP_NODE_208(p) PP_IIF(p(208), PP_NODE_200, PP_NODE_216)
#                define PP_NODE_200(p) PP_IIF(p(200), PP_NODE_196, PP_NODE_204)
#                    define PP_NODE_196(p) PP_IIF(p(196), PP_NODE_194, PP_NODE_198)
#                        define PP_NODE_194(p) PP_IIF(p(194), PP_NODE_193, PP_NODE_195)
#                            define PP_NODE_193(p) PP_IIF(p(193), 193, 194)
#                            define PP_NODE_195(p) PP_IIF(p(195), 195, 196)
#                        define PP_NODE_198(p) PP_IIF(p(198), PP_NODE_197, PP_NODE_199)
#                            define PP_NODE_197(p) PP_IIF(p(197), 197, 198)
#                            define PP_NODE_199(p) PP_IIF(p(199), 199, 200)
#                    define PP_NODE_204(p) PP_IIF(p(204), PP_NODE_202, PP_NODE_206)
#                        define PP_NODE_202(p) PP_IIF(p(202), PP_NODE_201, PP_NODE_203)
#                            define PP_NODE_201(p) PP_IIF(p(201), 201, 202)
#                            define PP_NODE_203(p) PP_IIF(p(203), 203, 204)
#                        define PP_NODE_206(p) PP_IIF(p(206), PP_NODE_205, PP_NODE_207)
#                            define PP_NODE_205(p) PP_IIF(p(205), 205, 206)
#                            define PP_NODE_207(p) PP_IIF(p(207), 207, 208)
#                define PP_NODE_216(p) PP_IIF(p(216), PP_NODE_212, PP_NODE_220)
#                    define PP_NODE_212(p) PP_IIF(p(212), PP_NODE_210, PP_NODE_214)
#                        define PP_NODE_210(p) PP_IIF(p(210), PP_NODE_209, PP_NODE_211)
#                            define PP_NODE_209(p) PP_IIF(p(209), 209, 210)
#                            define PP_NODE_211(p) PP_IIF(p(211), 211, 212)
#                        define PP_NODE_214(p) PP_IIF(p(214), PP_NODE_213, PP_NODE_215)
#                            define PP_NODE_213(p) PP_IIF(p(213), 213, 214)
#                            define PP_NODE_215(p) PP_IIF(p(215), 215, 216)
#                    define PP_NODE_220(p) PP_IIF(p(220), PP_NODE_218, PP_NODE_222)
#                        define PP_NODE_218(p) PP_IIF(p(218), PP_NODE_217, PP_NODE_219)
#                            define PP_NODE_217(p) PP_IIF(p(217), 217, 218)
#                            define PP_NODE_219(p) PP_IIF(p(219), 219, 220)
#                        define PP_NODE_222(p) PP_IIF(p(222), PP_NODE_221, PP_NODE_223)
#                            define PP_NODE_221(p) PP_IIF(p(221), 221, 222)
#                            define PP_NODE_223(p) PP_IIF(p(223), 223, 224)
#            define PP_NODE_240(p) PP_IIF(p(240), PP_NODE_232, PP_NODE_248)
#                define PP_NODE_232(p) PP_IIF(p(232), PP_NODE_228, PP_NODE_236)
#                    define PP_NODE_228(p) PP_IIF(p(228), PP_NODE_226, PP_NODE_230)
#                        define PP_NODE_226(p) PP_IIF(p(226), PP_NODE_225, PP_NODE_227)
#                            define PP_NODE_225(p) PP_IIF(p(225), 225, 226)
#                            define PP_NODE_227(p) PP_IIF(p(227), 227, 228)
#                        define PP_NODE_230(p) PP_IIF(p(230), PP_NODE_229, PP_NODE_231)
#                            define PP_NODE_229(p) PP_IIF(p(229), 229, 230)
#                            define PP_NODE_231(p) PP_IIF(p(231), 231, 232)
#                    define PP_NODE_236(p) PP_IIF(p(236), PP_NODE_234, PP_NODE_238)
#                        define PP_NODE_234(p) PP_IIF(p(234), PP_NODE_233, PP_NODE_235)
#                            define PP_NODE_233(p) PP_IIF(p(233), 233, 234)
#                            define PP_NODE_235(p) PP_IIF(p(235), 235, 236)
#                        define PP_NODE_238(p) PP_IIF(p(238), PP_NODE_237, PP_NODE_239)
#                            define PP_NODE_237(p) PP_IIF(p(237), 237, 238)
#                            define PP_NODE_239(p) PP_IIF(p(239), 239, 240)
#                define PP_NODE_248(p) PP_IIF(p(248), PP_NODE_244, PP_NODE_252)
#                    define PP_NODE_244(p) PP_IIF(p(244), PP_NODE_242, PP_NODE_246)
#                        define PP_NODE_242(p) PP_IIF(p(242), PP_NODE_241, PP_NODE_243)
#                            define PP_NODE_241(p) PP_IIF(p(241), 241, 242)
#                            define PP_NODE_243(p) PP_IIF(p(243), 243, 244)
#                        define PP_NODE_246(p) PP_IIF(p(246), PP_NODE_245, PP_NODE_247)
#                            define PP_NODE_245(p) PP_IIF(p(245), 245, 246)
#                            define PP_NODE_247(p) PP_IIF(p(247), 247, 248)
#                    define PP_NODE_252(p) PP_IIF(p(252), PP_NODE_250, PP_NODE_254)
#                        define PP_NODE_250(p) PP_IIF(p(250), PP_NODE_249, PP_NODE_251)
#                            define PP_NODE_249(p) PP_IIF(p(249), 249, 250)
#                            define PP_NODE_251(p) PP_IIF(p(251), 251, 252)
#                        define PP_NODE_254(p) PP_IIF(p(254), PP_NODE_253, PP_NODE_255)
#                            define PP_NODE_253(p) PP_IIF(p(253), 253, 254)
#                            define PP_NODE_255(p) PP_IIF(p(255), 255, 256)
#
# endif
# endif
