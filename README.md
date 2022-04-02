# 综合运用多个中断实现功能示例

## 任务

假定要实现的功能为：让两位数码管从00开始显示，显示到学号后两位+20：如我的学号是67，则需要从00跳到67+20=87.每秒变一次数字。到学号后两位+20停止变动数字。如我要实现数码管依次显示01、02、。。09、10、11、。。。86、87。

## 请同学们思考10分钟

从以下问题开始：

1. 硬件上需要什么器件、怎么连接
1. 用到单片机的哪些功能
1. 功能怎么实现、程序如何编写

## 硬件上

只用到数码管、电阻、单片机。

## 功能分析

1. 让数码管显示正确的数字
    1. 需要用一个定时器快速切换并正确显示两位数字，每几十毫秒一次，可暂定30ms。具体逻辑为：
        1. 进入中断
        1. 如果当前显示个位则关个位显示，开十位显示十位数码。如果当前显示十位则关十位显示，开个位显示个位数码。
        1. 结束中断
1. 变换数字
    1. 需要用一个定时器用于数字的变动的显示，每秒一次。具体逻辑为：
        1. 进入中断
        1. 检查是否到终点，没到则让显示数字+1
        1. 结束中断

1. "让显示数字+1",为我们带来了新实现点，分析如何实现：
    1. 必须结合“让数码管显示正确的数字”进行，因此“让数码管显示正确的数字”功能点，需要以某种形式让我们指定数字
        * 用数组进行转换
        * 定义函数提取个位和十位
        * define和变量的区别
    2. 这个指定显示的数字，同时又要被“变换数字”功能改变，所以要在两个中断函数中