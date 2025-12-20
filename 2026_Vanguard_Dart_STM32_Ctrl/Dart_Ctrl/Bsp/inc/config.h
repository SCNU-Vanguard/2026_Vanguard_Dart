/**********************************************************
 * 文件名：config.h
 * 用途：存放一些关于整个飞镖系统工程参数的宏定义
 * 创建时间：12.15 2025
 * 创建人：邓金水
 * 数据全部经过测试得来
 * todo：等待准确的数据
 *********************************************************/
#ifndef __CONFIG_H_
#define __CONFIG_H_

// 换弹结构相关
#define ConveyorBeltLength 20673
#define SeperationAngle 375
#define FirstServoLoc -7547
#define SecondServoLoc -13200
#define ThirdServoLoc -18749
// 第三个舵机（距离7547）
// 第二个舵机（距离13200）<与第三个相距5653>
// 第一个舵机（距离18749）<与第二个相距5549>

// 同步带相关

// 扳机射程相关
#define MG996R_store 2500   // 发射扳机待机状态。
#define MG996R_shoot 1000   // 发射扳机发射状态
#define MG996R_initial 2500 // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG996R_extend 1750  // 飞镖支架伸出状态
#define MG996R_shrink 900   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点
#define MG995_initial 2500  // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG995_extend 2000   // 飞镖支架伸出状态
#define MG995_shrink 1500   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点

// 云台转轴相关

#endif
