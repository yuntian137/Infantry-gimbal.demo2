# Infantry-gimbal.demo2-作为demo1的修改版
实际上只是实习期最后一次平衡步云台调试代码，注释很详细，看不懂注释可以多看，在freertos.c里存放了几乎所有的任务函数，基本实现模块化，看懂一块就能懂很多块。
主要功能为，基于stm32f405rgt6-HAL-CUBEMX的实现了通过DR16控制M2006拨盘电机连发与单发模式转换，pitch轴yaw轴GM6020电机转动、
两个摩擦轮为单环pid，拨盘为串级pid（需要注意拨盘电机的角度换算与pitch,yaw轴电机不同，是将错就错调出来的参数），yaw轴为正常的串级pid，pitch轴实现了陀螺仪反馈角度并进行互补滤波，且加上了前馈控制。
注意一个小点，弹舱盖的开关由于舵机摆放角度不正，故而参数有所调整，请根据实际情况调整。
能帮到你就点个星星叭！