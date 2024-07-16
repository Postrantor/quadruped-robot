##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



INTRODUCTION
============

qpOASES is an open-source C++ implementation of the recently proposed online active set strategy, which was inspired by important observations from the field of parametric quadratic programming (QP). It has several theoretical features that make it particularly suited for model predictive control (MPC) applications. Further numerical modifications have made qpOASES a reliable QP solver, even when tackling semi-definite, ill-posed or degenerated QP problems. Moreover, several interfaces to third-party software like ​Matlab or ​Simulink are provided that make qpOASES easy-to-use even for users without knowledge of C/C++.

> qpOASES 是最近提出的在线主动集策略的开源 C++ 实现。在线主动集策略的开源 C++ 实现。该策略的灵感来自参数二次编程 (QP) 领域的重要观察结果。它有几个理论特性，使其特别适用于模型预测控制（MPC控制 (MPC) 应用。进一步的数值修改使qpOASES 已成为一个可靠的 QP 求解器，即使在处理半有限问题、问题严重或退化的 QP 问题时也是如此。问题。此外，qpOASES 还为第三方软件（如 Matlab 或 Simulink）提供了多个接口。接口，即使不懂 C/C++ 的用户也能轻松使用 qpOASES。不懂 C/C++ 的用户也能轻松使用。

GETTING STARTED
===============

1. For installation, usage and additional information on this software package 
   see the qpOASES User's Manual located at doc/manual.pdf or check its
   source code documentation!


2. The file LICENSE.txt contains a copy of the GNU Lesser General Public 
   License (v2.1). Please read it carefully before using qpOASES!


3. The whole software package can be obtained from 

       http://www.qpOASES.org/ or
	   https://projects.coin-or.org/qpOASES/

   On this webpage you will also find further support such as a list of 
   questions posed by other users.



CONTACT THE AUTHORS
===================

If you have got questions, remarks or comments on qpOASES, it is strongly 
encouraged to report them by creating a new ticket at the qpOASES webpage.
In case you do not want to disclose your feedback to the public, you may
send an e-mail to

        support@qpOASES.org

Finally, you may contact one of the main authors directly:

        Hans Joachim Ferreau, joachim.ferreau@ch.abb.com
        Andreas Potschka,     potschka@iwr.uni-heidelberg.de
        Christian Kirches,    christian.kirches@iwr.uni-heidelberg.de

Also bug reports, source code enhancements or success stories are most welcome!



##
##	end of file
##
