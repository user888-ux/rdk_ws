import argparse
import threading
import time
from flask import Flask, jsonify
import serial

app = Flask(__name__)

# 解析命令行参数（可选串口路径）
parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', default='/dev/ttyS1', help='串口设备路径')
args = parser.parse_args()

# 打开串口（注意：这里同时用于发送和接收）
ser = serial.Serial(args.port, 115200, timeout=1)

# 用于存储最新收到的数据（可改为队列）
latest_data = None
data_lock = threading.Lock()

def serial_listener():
    """后台线程：持续读取串口数据并存储"""
    global latest_data
    while True:
        try:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                # 将字节数据转换为字符串（根据实际协议调整）
                text = data.decode('ascii', errors='replace')
                with data_lock:
                    latest_data = text
                # 也可以打印到控制台便于调试
                print(f"收到: {text}")
            time.sleep(0.01)
        except Exception as e:
            print(f"串口读取异常: {e}")
            break

# 启动监听线程（非守护线程，确保应用退出时能清理）
thread = threading.Thread(target=serial_listener, daemon=True)
thread.start()

@app.route('/laser/on')
def laser_on():
    ser.write(b'1')
    return jsonify(status='success', message='laser on')

@app.route('/laser/off')
def laser_off():
    ser.write(b'0')
    return jsonify(status='success', message='laser off')

@app.route('/laser/data')
def get_laser_data():
    """获取最近收到的串口数据"""
    with data_lock:
        return jsonify(data=latest_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
