import serial
import time
import os
import sys

# 针对树莓派 5 和 CH340 芯片修改后的正确端口
USB_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def send_gcode(file_path):
    # 1. 检查 G-code 文件是否存在
    if not os.path.exists(file_path):
        print(f"❌ 错误: 找不到文件 {file_path}")
        return

    try:
        print(f"正在连接到绘图机: {USB_PORT}...")
        # 2. 建立串口连接
        s = serial.Serial(USB_PORT, BAUD_RATE, timeout=1)
        
        # 3. 唤醒 Grbl 固件 (发送空格/换行并等待初始化)
        s.write(b"\r\n\r\n")
        time.sleep(2)   # 给机器一点反应时间
        s.reset_input_buffer()

        print(f"🚀 开始发送指令: {file_path}")
        
        with open(file_path, 'r') as f:
            for line in f:
                l = line.strip()
                # 跳过空行或注释
                if not l or l.startswith(';'):
                    continue
                
                # 发送指令给机器
                s.write((l + '\n').encode('utf-8'))
                
                # 等待机器回复 'ok' 才发送下一行 (防止缓冲区溢出)
                while True:
                    grbl_out = s.readline().decode('utf-8').strip()
                    if grbl_out == 'ok':
                        break
                    elif 'error' in grbl_out.lower():
                        print(f"  ⚠️ 机器报错: {grbl_out}")
                        break

        print("\n✨ 绘图任务完成！")
        s.close()
        
    except serial.SerialException as e:
        print(f"❌ 串口错误: {e}")
        print("💡 请检查: 1.绘图机是否开机 2.USB线是否插紧 3.是否运行了 sudo chmod 666 /dev/ttyUSB0")
    except Exception as e:
        print(f"❌ 发生未知错误: {e}")

if __name__ == "__main__":
    # 核心：优先接收来自 main.py 传过来的具体文件名
    if len(sys.argv) > 1:
        gcode_file = sys.argv[1]
    else:
        # 如果手动运行 python3 send.py，则默认画最近生成的 draw.ngc
        gcode_file = "output/draw.ngc"
    
    send_gcode(gcode_file)
