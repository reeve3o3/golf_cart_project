# 安裝配置指南 - UWB導航系統

## 系統需求

### 硬體需求
- **Raspberry Pi 4** (建議4GB RAM以上)
- **UWB模組** DW1000 (支援UART通訊)
- **馬達驅動板** (支援GPIO控制)
- **Micro SD卡** (32GB以上，Class 10)
- **電源供應** (5V 3A USB-C + UWB模組5V/1A)

### UWB模組規格確認
```
型號: DW1000
尺寸: 80×55mm
供電需求: DC5V/1A
通訊接口: TTL3.3V UART
波特率: 115200, 8N1
工作頻段: 6.5GHz
測距範圍: 80m(穩定) / 150m(極限)
角度精度: ±5°
距離精度: ±10cm
更新頻率: 100Hz
```

### 軟體需求
- **作業系統**: Raspberry Pi OS (Bullseye或更新版本)
- **Python**: 3.7以上版本
- **依賴套件**: pyserial, numpy, RPi.GPIO

## 硬體安裝

### 1. UWB模組連接 (DW1000)

#### 電源與信號連接
```
Raspberry Pi 4        DW1000
GPIO 14 (TXD)    ←→   RX (TTL3.3V)
GPIO 15 (RXD)    ←→   TX (TTL3.3V)
5V Power         ←→   VCC (DC5V/1A)
Ground           ←→   GND
```

**注意事項**:
- UWB模組需要5V/1A穩定供電
- TTL3.3V信號電平與Raspberry Pi兼容
- 確認波特率設定為115200, 8N1
- 工作溫度範圍: -20℃至65℃

#### 接線圖
```
   Pi 4 GPIO Layout (相關腳位)
   ┌─────────────────────────┐
   │ 2[5V]     4[5V]        │
   │ 6[GND]    8[TXD]       │
   │10[RXD]   12[GPIO18]    │
   │14[]      16[GPIO23]    │
   │18[GPIO24] 20[GND]      │
   │22[GPIO25] 24[]         │
   └─────────────────────────┘
```

### 2. 馬達控制連接

#### GPIO腳位分配
```python
FORWARD_PIN = 18  # GPIO 18 - 前進控制
LEFT_PIN = 23     # GPIO 23 - 左轉控制
RIGHT_PIN = 24    # GPIO 24 - 右轉控制  
STOP_PIN = 25     # GPIO 25 - 停止控制
```

#### 連接馬達驅動板
```
GPIO 18  ──→  馬達驅動板前進輸入
GPIO 23  ──→  馬達驅動板左轉輸入
GPIO 24  ──→  馬達驅動板右轉輸入
GPIO 25  ──→  馬達驅動板停止輸入
GND      ──→  馬達驅動板地線
```

## 系統配置

### 1. 啟用UART接口

#### 方法一：使用raspi-config
```bash
sudo raspi-config
```
選擇路徑：
```
Interfacing Options → Serial Port
→ "Would you like a login shell accessible over serial?" → No
→ "Would you like the serial port hardware enabled?" → Yes
```

#### 方法二：手動編輯配置文件
```bash
# 編輯boot配置
sudo nano /boot/config.txt

# 添加以下行
enable_uart=1
dtoverlay=disable-bt
```

#### 方法三：檢查UART狀態
```bash
# 檢查UART設備
ls -l /dev/ttyS0
ls -l /dev/serial*

# 檢查UART配置
dmesg | grep tty
```

### 2. 禁用串口控制台

#### 編輯cmdline.txt
```bash
sudo nano /boot/cmdline.txt

# 移除以下內容（如果存在）
console=serial0,115200
```

#### 編輯getty服務
```bash
# 禁用串口getty服務
sudo systemctl disable serial-getty@ttyS0.service
```

## 軟體安裝

### 1. 更新系統
```bash
# 更新套件列表
sudo apt update
sudo apt upgrade -y

# 安裝基本工具
sudo apt install -y git python3-pip python3-dev build-essential
```

### 2. 安裝Python依賴
```bash
# 安裝必要的Python套件
pip3 install --upgrade pip
pip3 install pyserial numpy RPi.GPIO

# 驗證安裝
python3 -c "import serial, numpy, RPi.GPIO; print('所有套件安裝成功')"
```

### 3. 系統配置優化

#### 禁用不需要的服務
```bash
# 禁用藍牙（釋放UART資源）
sudo systemctl disable bluetooth
sudo systemctl disable hciuart

# 禁用音頻（節省資源）
sudo systemctl disable alsa-state
```

#### 優化系統性能
```bash
# 編輯config.txt優化性能
sudo nano /boot/config.txt

# 添加以下配置
# GPU記憶體分割（最小化）
gpu_mem=16

# 啟用硬體時鐘
dtparam=i2c_arm=on
dtparam=spi=on

# 禁用音頻
dtparam=audio=off
```

## 項目部署

### 1. 創建項目目錄
```bash
# 創建項目目錄
mkdir -p ~/uwb_navigation
cd ~/uwb_navigation

# 創建必要的子目錄
mkdir logs data config
```

### 2. 部署程式碼
```bash
# 將程式碼複製到項目目錄
cp /path/to/source_code.py ~/uwb_navigation/

# 設置執行權限
chmod +x ~/uwb_navigation/source_code.py
```

### 3. 創建配置文件
```bash
# 創建配置文件
cat > ~/uwb_navigation/config/uwb_config.ini << EOF
[UWB]
port = /dev/ttyS0
baud_rate = 115200
timeout = 1.0

[GPIO]
forward_pin = 18
left_pin = 23
right_pin = 24
stop_pin = 25

[Control]
angle_threshold_normal = 12
angle_threshold_caution = 10
distance_stop = 1.5
distance_caution = 2.5

[Logging]
level = INFO
log_file = logs/uwb_navigation.log
EOF
```

### 4. 創建啟動腳本
```bash
# 創建啟動腳本
cat > ~/uwb_navigation/start_uwb.sh << 'EOF'
#!/bin/bash

# UWB導航系統啟動腳本
cd ~/uwb_navigation

# 檢查硬體連接
echo "檢查硬體連接..."
if [ ! -e /dev/ttyS0 ]; then
    echo "錯誤: UWB設備未連接 (/dev/ttyS0)"
    exit 1
fi

# 檢查GPIO權限
if [ ! -r /dev/gpiomem ]; then
    echo "錯誤: GPIO權限不足"
    exit 1
fi

# 創建日誌目錄
mkdir -p logs

# 啟動程式
echo "啟動UWB導航系統..."
python3 source_code.py 2>&1 | tee logs/uwb_navigation.log

EOF

# 設置執行權限
chmod +x ~/uwb_navigation/start_uwb.sh
```

### 5. 創建系統服務（可選）
```bash
# 創建systemd服務文件
sudo tee /etc/systemd/system/uwb-navigation.service > /dev/null << EOF
[Unit]
Description=UWB Navigation System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/uwb_navigation
ExecStart=/usr/bin/python3 /home/pi/uwb_navigation/source_code.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# 啟用服務
sudo systemctl daemon-reload
sudo systemctl enable uwb-navigation.service
```

## 測試驗證

### 1. 硬體連接測試

#### 測試UART連接
```bash
# 測試UART設備
python3 -c "
import serial
try:
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    print('UART連接成功')
    ser.close()
except Exception as e:
    print(f'UART連接失敗: {e}')
"
```

#### 測試GPIO控制
```bash
# 創建GPIO測試腳本
cat > test_gpio.py << 'EOF'
import RPi.GPIO as GPIO
import time

# GPIO設置
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pins = [18, 23, 24, 25]
pin_names = ['前進', '左轉', '右轉', '停止']

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

print("GPIO測試開始...")
for i, pin in enumerate(pins):
    print(f"測試{pin_names[i]}腳位 (GPIO {pin})")
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.5)

GPIO.cleanup()
print("GPIO測試完成")
EOF

python3 test_gpio.py
```

### 2. UWB數據測試

#### 創建UWB測試腳本
```bash
cat > test_uwb.py << 'EOF'
import serial
import time

def test_uwb_connection():
    try:
        ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        print("UWB連接成功，等待數據...")
        
        start_time = time.time()
        data_count = 0
        
        while time.time() - start_time < 10:  # 測試10秒
            if ser.in_waiting >= 31:
                data = ser.read(31)
                if data[0] == 0x2A and data[30] == 0x23:
                    data_count += 1
                    if data_count <= 5:  # 只顯示前5筆數據
                        print(f"收到有效數據包 #{data_count}: {data.hex()}")
        
        ser.close()
        print(f"測試完成，10秒內收到 {data_count} 筆有效數據")
        
        if data_count > 0:
            print("✓ UWB連接正常")
            return True
        else:
            print("✗ 未收到有效UWB數據")
            return False
            
    except Exception as e:
        print(f"UWB測試失敗: {e}")
        return False

if __name__ == "__main__":
    test_uwb_connection()
EOF

python3 test_uwb.py
```

### 3. 系統整合測試

#### 運行完整系統測試
```bash
# 創建系統測試腳本
cat > system_test.py << 'EOF'
import time
import threading
from source_code import UWBReader, MotorController

def system_integration_test():
    print("開始系統整合測試...")
    
    # 初始化組件
    uwb_reader = UWBReader('/dev/ttyS0', 115200)
    motor_controller = MotorController()
    
    # 啟動UWB讀取
    if not uwb_reader.start_reading():
        print("✗ UWB讀取器啟動失敗")
        return False
    
    print("✓ UWB讀取器啟動成功")
    
    # 測試數據接收
    test_duration = 5
    start_time = time.time()
    data_received = 0
    
    while time.time() - start_time < test_duration:
        if uwb_reader.is_data_fresh(timeout=1.0):
            data = uwb_reader.get_latest_data()
            if data:
                data_received += 1
                print(f"數據 #{data_received}: 角度={data['angle']:.2f}°, 距離={data['distance']:.2f}m")
        time.sleep(0.1)
    
    # 清理資源
    uwb_reader.stop_reading()
    motor_controller.cleanup()
    
    if data_received > 0:
        print(f"✓ 系統測試通過，接收到 {data_received} 筆數據")
        return True
    else:
        print("✗ 系統測試失敗，未接收到數據")
        return False

if __name__ == "__main__":
    system_integration_test()
EOF

python3 system_test.py
```

## 故障排除

### 1. 常見問題及解決方案

#### 問題：UART設備不存在
```bash
# 症狀
ls: cannot access '/dev/ttyS0': No such file or directory

# 解決方案
# 1. 檢查UART是否啟用
sudo raspi-config  # 確認Serial Port已啟用

# 2. 檢查設備樹配置
grep -i uart /boot/config.txt

# 3. 重新啟動系統
sudo reboot
```

#### 問題：權限被拒絕
```bash
# 症狀
PermissionError: [Errno 13] Permission denied: '/dev/ttyS0'

# 解決方案
# 1. 添加用戶到正確的群組
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER

# 2. 重新登入或重啟
sudo reboot

# 3. 手動設置權限（臨時）
sudo chmod 666 /dev/ttyS0
sudo chmod 666 /dev/gpiomem
```

#### 問題：GPIO控制失效
```bash
# 症狀
RuntimeError: No access to /dev/gpiomem

# 解決方案
# 1. 檢查GPIO群組
groups $USER | grep gpio

# 2. 安裝GPIO庫
sudo apt install python3-rpi.gpio

# 3. 測試GPIO訪問
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO正常')"
```

### 2. 調試工具

#### 串口監控工具
```bash
# 安裝minicom
sudo apt install minicom

# 監控串口數據
sudo minicom -D /dev/ttyS0 -b 115200
```

#### 系統監控
```bash
# CPU和記憶體使用情況
top
htop

# GPIO狀態查看
gpio readall

# 串口狀態查看
dmesg | grep tty
lsof | grep ttyS0
```

#### 日誌分析
```bash
# 查看系統日誌
sudo journalctl -u uwb-navigation.service -f

# 查看內核消息
dmesg | tail -20

# 查看應用日誌
tail -f ~/uwb_navigation/logs/uwb_navigation.log
```

### 3. 性能調優

#### 系統優化
```bash
# 增加GPU記憶體分割
echo "gpu_mem=16" | sudo tee -a /boot/config.txt

# 禁用不需要的服務
sudo systemctl disable bluetooth
sudo systemctl disable wifi-country
sudo systemctl disable triggerhappy

# 設置CPU調度器
echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

#### 應用層優化
```python
# 在程式中添加性能監控
import psutil
import time

def monitor_performance():
    cpu_percent = psutil.cpu_percent(interval=1)
    memory_info = psutil.virtual_memory()
    
    if cpu_percent > 80:
        logger.warning(f"CPU使用率過高: {cpu_percent}%")
    
    if memory_info.percent > 80:
        logger.warning(f"記憶體使用率過高: {memory_info.percent}%")
```

## 維護指南

### 1. 定期維護任務

#### 每週維護
```bash
# 清理日誌文件
find ~/uwb_navigation/logs -name "*.log" -mtime +7 -delete

# 檢查系統狀態
systemctl status uwb-navigation.service

# 監控磁碟空間
df -h /
```

#### 每月維護
```bash
# 更新系統
sudo apt update && sudo apt upgrade

# 檢查硬體連接
python3 test_uwb.py
python3 test_gpio.py

# 備份配置文件
tar -czf uwb_backup_$(date +%Y%m%d).tar.gz ~/uwb_navigation/config/
```

### 2. 監控指標

#### 關鍵性能指標
- CPU使用率 < 50%
- 記憶體使用 < 512MB
- UWB數據接收率 > 90%
- 系統響應時間 < 50ms

#### 告警設置
```python
# 在程式中添加監控告警
def check_system_health():
    cpu_usage = psutil.cpu_percent()
    memory_usage = psutil.virtual_memory().percent
    
    if cpu_usage > 80:
        logger.error(f"CPU過載: {cpu_usage}%")
    
    if memory_usage > 80:
        logger.error(f"記憶體不足: {memory_usage}%")
```

---

*安裝配置指南版本: 1.0*  
*最後更新: 2025年7月29日*
