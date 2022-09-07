# Games101HW

__Author: Zhan Zhan__
__Last update: 22/09/01__



This is my homework for Games101.

Assignment 0 is updated. 22/05/20

Assignment 1 is updated. 22/06/23

Assignment 2 is updated. 22/07/14

Assignment 3 is updated. 22/07/31

Assignment 4 is updated. 22/08/25

Assignment 5 is updated. 22/08/30

Assignment 6 is updated. 22/09/01

Assignment 7 is updated. 22/09/07

作业2，3的框架中znear和zfar的值为正数，作业中将其修改为负数和ppt中矩阵n,f对应,相应的会修改depth_buf初始值和比较方法

框架在深度插值上有一定问题，可以参考下面这篇文章：https://zhuanlan.zhihu.com/p/509902950

作业4可以点击右键删掉最后一个点，控制点的上限设置为20个

作业5代码调用： Render => castRay => trace, cast         trace => object->intersect

作业6中： Bounds3::IntersectP 为了实现对Vector3f调用swap，在Vector.h中增加了对float&  operator\[](int index)的实现

作业7中： 在Vector.h中增加了对Vector3f类 ==操作符重载的实现，浮点数判断大小的范围在函数中my_epsilon设定；另外相比于作业6，修改了Bound3.hpp 中Bounds3::IntersectP返回值的判断，因为本次作业中墙体厚度为0，包围盒的进入和出去的时间可能是相等的，需要避坑。get_random_float函数添加static关键字可以提高速度

Metion: 

CMakeLists.txt is changed for my computer



