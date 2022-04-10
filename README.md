# learn_control

#### 介绍
基于《飞机的性能、稳定性、动力学与控制》，班度.N.帕玛迪；基于《自动控制原理》，胡寿松；

**做控制**是一个很大的概念，控制律的设计方法可能大多是基于PID或其他控制理论的，但是学会对于一个6自由度的模型（在这里即飞机，常规布局），从他的性能、稳定性操纵性、6DOF自由度方程，动导数方程，闭环响应等去做一个完整的闭环控制需要一个完整的动力学+建模+控制理论的知识结构；

我会说，我是学飞控的，不是做控制的（来自LBY学长）；我还不会飞控开发，我现在只是在学控制律的设计（其实是上学期没学好），简简单单而又十分麻烦的从一个6自由度的飞机模型开始，接触从0到1控制律的设计；

DWS学长说，飞行力学控制和自动控制原理是我们飞控安身立命的东西；虽然我以后想去学robotics，想去学RCS（add control systems）这类的课程，但我觉得学长说的很有道理；我的专业是飞控，这两门课过不去，实在不好意思说自己是学飞控的；

#### 计划
* MINI1600飞机控制律的设计
* **总的设计步骤和方法（重要）**--[Design Steps](DesignSteps.md)
* 示例代码--[Code Demo](control_law_design/sources/CodeDemo.md)
* 我的设计报告markdown--[my Design](control_law_design/myDesign.md)
* 飞机的性能、稳定性、动力学与控制PSDC[笔记](Airplane_PSDC.md)

#### 参考
* [PILAB仓库关于飞控](https://gitee.com/hazyparker/research_flight_controller)
* ZSChen的代码和思路
* welcome to fork and pull request

#### 建议

* 建议把markdown下下来看，这个网站有的LaTex解码不出来，矩阵看的很难受；
* 使用Typora查看和编辑markdown文件，[Typora — a markdown editor, markdown reader.](https://www.typora.io/)
* Typora用法，[Typora 完全使用详解](https://sspai.com/post/54912/)
* 可以给电脑安装一个`Git`，[Git - Downloads (git-scm.com)](https://git-scm.com/downloads)；然后`git clone  https://gitee.com/hazyparker/learn_control.git`，每次更新之后直接`git pull`；Git的配置不放了；
* 建议`fork`+`git clone`+`git pull`
* **如果大量引用，请附`参考`，毕竟不是一个人的成果**；

#### 联系

* Email_hazyparker: 2559272883@qq.com
* Gitee ZSChen：[zschen879- Gitee.com](https://gitee.com/zschen879)
* Gitee A_XJ：[xuchengbei - Gitee.com](https://gitee.com/xuchengbei)
