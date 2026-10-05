#!/bin/bash
# 终极底层方案：直接写串口文件，无任何依赖
SERIAL_PORT="/dev/ttyUSB0"

# 1. 极简重置串口（只保留必须的，避开报错参数）
sudo stty -F $SERIAL_PORT 9600 cs8 -cstopb -parenb raw -echo
sudo chmod 777 $SERIAL_PORT

# 2. 等STM32完全启动（4秒，确保不复位）
echo "⏳ 等待LeArm初始化..."
sleep 4

# 3. 直接往串口文件写指令（和电脑串口助手一模一样的字节）
# 底座左转：s,6,875,2000\r\n
echo -n -e "s,6,875,2000\r\n" | sudo tee $SERIAL_PORT > /dev/null
echo "✅ 发送指令：s,6,875,2000"
sleep 3

# 爪子张开：s,1,200,2000\r\n
echo -n -e "s,1,200,2000\r\n" | sudo tee $SERIAL_PORT > /dev/null
echo "✅ 发送指令：s,1,200,2000"
sleep 3

echo "🎉 执行完成！"

# 兜底：如果还不动，执行这行（手动强制写）
echo -e "\n❌ 若仍不动，终端手动运行："
echo "sudo echo -n -e 's,6,875,2000\\r\\n' > $SERIAL_PORT"