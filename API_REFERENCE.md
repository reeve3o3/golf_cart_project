# API參考文檔 - UWB導航系統

## 概述

本文檔詳細說明了UWB導航系統中各個類別的API接口、參數說明和使用範例。

## 類別API參考

### 1. UWBReader

UWB數據讀取器，負責與UWB-X2-AOA-N(GW)模組的UART通訊。

#### UWB模組規格
- **型號**: UWB-X2-AOA-N(GW)
- **通訊接口**: TTL3.3V UART  
- **波特率**: 115200, 8N1
- **更新頻率**: 100Hz
- **角度精度**: ±5°
- **測距精度**: ±10cm
- **有效範圍**: 80m(穩定) / 150m(極限)

#### 構造函數
```python
UWBReader(port: str = '/dev/ttyS0', baud_rate: int = 115200)
```

**參數：**
- `port`: UART設備路徑，預設 '/dev/ttyS0'
- `baud_rate`: 通訊波特率，預設 115200

**範例：**
```python
uwb_reader = UWBReader('/dev/ttyS0', 115200)
```

#### 方法

##### `start_reading() -> bool`
啟動UWB數據讀取線程。

**返回值：**
- `bool`: 成功啟動返回True，否則False

**範例：**
```python
if uwb_reader.start_reading():
    print("UWB讀取器啟動成功")
else:
    print("UWB讀取器啟動失敗")
```

##### `stop_reading() -> None`
停止UWB數據讀取線程。

**範例：**
```python
uwb_reader.stop_reading()
```

##### `get_latest_data() -> dict | None`
獲取最新的UWB數據（線程安全）。

**返回值：**
```python
{
    'address': str,      # 設備地址（十六進制字符串）
    'angle': float,      # 濾波後的角度值（度）
    'raw_angle': float,  # 原始角度值（度）
    'distance': float,   # 距離值（米）
    'timestamp': float   # 時間戳（Unix時間）
}
```

**範例：**
```python
data = uwb_reader.get_latest_data()
if data:
    print(f"角度: {data['angle']:.2f}°, 距離: {data['distance']:.2f}m")
```

##### `is_data_fresh(timeout: float = 1.0) -> bool`
檢查數據是否在指定時間內更新。

**參數：**
- `timeout`: 超時時間（秒），預設1.0秒

**返回值：**
- `bool`: 數據新鮮返回True，超時返回False

**範例：**
```python
if uwb_reader.is_data_fresh(timeout=0.5):
    data = uwb_reader.get_latest_data()
    # 處理新鮮數據
```

##### `process_valid_data(data: bytes) -> dict | None`
處理有效的UWB數據包（內部方法）。

**參數：**
- `data`: 31字節的UWB數據包

**返回值：**
- `dict | None`: 解析成功返回數據字典，失敗返回None

---

### 2. AdaptiveKalmanFilter

自適應卡爾曼濾波器，專門處理角度數據濾波。

#### 構造函數
```python
AdaptiveKalmanFilter(initial_angle: float = 0.0)
```

**參數：**
- `initial_angle`: 初始角度值（度），預設0.0

**範例：**
```python
kalman_filter = AdaptiveKalmanFilter(initial_angle=0.0)
```

#### 方法

##### `predict(dt: float) -> None`
執行濾波器預測步驟。

**參數：**
- `dt`: 時間間隔（秒）

**範例：**
```python
kalman_filter.predict(0.01)  # 10ms時間間隔
```

##### `update(measured_angle: float) -> float`
執行濾波器更新步驟並返回濾波後角度。

**參數：**
- `measured_angle`: 測量的角度值（度）

**返回值：**
- `float`: 濾波後的角度值（度）

**範例：**
```python
filtered_angle = kalman_filter.update(raw_angle)
```

##### `normalize_angle(angle: float) -> float`
將角度正規化到[-90, 90]範圍。

**參數：**
- `angle`: 輸入角度（度）

**返回值：**
- `float`: 正規化後的角度（度）

##### `detect_angle_jump(measured_angle: float) -> bool`
檢測角度突變。

**參數：**
- `measured_angle`: 測量角度（度）

**返回值：**
- `bool`: 檢測到突變返回True

---

### 3. PredictiveController

預測控制器，提供運動預測和控制優化功能。

#### 構造函數
```python
PredictiveController()
```

#### 方法

##### `add_measurement(angle: float, distance: float, timestamp: float) -> None`
添加新的測量數據。

**參數：**
- `angle`: 角度值（度）
- `distance`: 距離值（米）
- `timestamp`: 時間戳（秒）

**範例：**
```python
controller.add_measurement(15.5, 2.3, time.time())
```

##### `calculate_angular_velocity() -> float`
計算當前角速度。

**返回值：**
- `float`: 角速度（度/秒）

**範例：**
```python
angular_velocity = controller.calculate_angular_velocity()
print(f"角速度: {angular_velocity:.2f}°/s")
```

##### `calculate_angular_acceleration() -> float`
計算當前角加速度。

**返回值：**
- `float`: 角加速度（度/秒²）

##### `predict_future_angle(prediction_time: float = None) -> float`
預測未來角度。

**參數：**
- `prediction_time`: 預測時間（秒），預設使用估計延遲

**返回值：**
- `float`: 預測的未來角度（度）

**範例：**
```python
future_angle = controller.predict_future_angle(0.1)  # 預測0.1秒後的角度
```

##### `detect_convergence() -> bool`
檢測是否正在收斂到目標。

**返回值：**
- `bool`: 正在收斂返回True

##### `calculate_control_factor(current_angle: float, distance: float) -> float`
計算智能控制因子。

**參數：**
- `current_angle`: 當前角度（度）
- `distance`: 當前距離（米）

**返回值：**
- `float`: 控制因子（0.1-1.0）

**範例：**
```python
control_factor = controller.calculate_control_factor(angle, distance)
print(f"控制因子: {control_factor:.3f}")
```

---

### 4. AdaptiveSpeedController

自適應速度控制器，整合預測控制實現智能速度調節。

#### 構造函數
```python
AdaptiveSpeedController()
```

#### 屬性

##### `base_forward_levels: dict`
基礎前進速度級別。
```python
{
    'slow': 0.08,    # 慢速間隔（秒）
    'medium': 0.05,  # 中速間隔（秒）
    'fast': 0.02     # 快速間隔（秒）
}
```

##### `base_turn_levels: dict`
基礎轉向速度級別。
```python
{
    'fine': 0.15,       # 精細調整間隔（秒）
    'normal': 0.08,     # 正常轉向間隔（秒）
    'aggressive': 0.05  # 激進轉向間隔（秒）
}
```

#### 方法

##### `update_measurement(angle: float, distance: float, timestamp: float) -> None`
更新測量數據到預測控制器。

**參數：**
- `angle`: 角度值（度）
- `distance`: 距離值（米）
- `timestamp`: 時間戳（秒）

##### `get_forward_interval(distance: float, control_factor: float = 1.0) -> float | None`
獲取前進控制間隔。

**參數：**
- `distance`: 當前距離（米）
- `control_factor`: 控制因子，預設1.0

**返回值：**
- `float | None`: 控制間隔（秒），距離過近返回None

**範例：**
```python
interval = speed_controller.get_forward_interval(2.5, 0.8)
if interval:
    print(f"前進間隔: {interval:.3f}秒")
```

##### `get_turn_interval(angle: float, distance: float) -> float`
獲取轉向控制間隔。

**參數：**
- `angle`: 角度絕對值（度）
- `distance`: 當前距離（米）

**返回值：**
- `float`: 轉向控制間隔（秒）

**範例：**
```python
turn_interval = speed_controller.get_turn_interval(abs(angle), distance)
print(f"轉向間隔: {turn_interval:.3f}秒")
```

---

### 5. MotorController

馬達控制器，基於GPIO實現馬達控制邏輯。

#### 構造函數
```python
MotorController()
```

#### GPIO腳位配置
```python
FORWARD_PIN = 18  # 前進控制腳位
LEFT_PIN = 23     # 左轉控制腳位
RIGHT_PIN = 24    # 右轉控制腳位
STOP_PIN = 25     # 停止控制腳位
```

#### 方法

##### `setup_gpio() -> None`
初始化GPIO設定。

##### `trigger_button(pin: int, duration: float = 0.1) -> None`
觸發指定GPIO按鈕。

**參數：**
- `pin`: GPIO腳位號
- `duration`: 觸發持續時間（秒），預設0.1

##### `forward_pulse(duration: float = 0.05) -> None`
執行前進脈衝。

**參數：**
- `duration`: 脈衝持續時間（秒），預設0.05

**範例：**
```python
motor_controller.forward_pulse(0.05)
```

##### `left_pulse(duration: float = 0.05) -> None`
執行左轉脈衝。

**參數：**
- `duration`: 脈衝持續時間（秒），預設0.05

##### `right_pulse(duration: float = 0.05) -> None`
執行右轉脈衝。

**參數：**
- `duration`: 脈衝持續時間（秒），預設0.05

##### `stop() -> None`
執行停止命令。

**範例：**
```python
motor_controller.stop()
```

##### `control_motors(angle: float, distance: float, timestamp: float) -> None`
根據角度和距離控制馬達（主控制邏輯）。

**參數：**
- `angle`: 目標角度（度）
- `distance`: 目標距離（米）
- `timestamp`: 當前時間戳（秒）

**控制邏輯：**
- `distance ≤ 1.5m`: 立即停止
- `1.5m < distance < 2.5m`: 謹慎模式（角度閾值±10°）
- `distance ≥ 2.5m`: 正常模式（角度閾值±12°）

**範例：**
```python
motor_controller.control_motors(15.5, 2.3, time.time())
```

##### `cleanup() -> None`
清理GPIO資源。

**範例：**
```python
try:
    # 主程式邏輯
    pass
finally:
    motor_controller.cleanup()
```

---

## 數據結構

### UWB數據包格式

```python
class UWBDataPacket:
    """
    UWB數據包結構（31字節）
    """
    start_marker: bytes     # Byte 0: 0x2A
    reserved: bytes         # Byte 1-2: 保留
    address_high: int       # Byte 3: 地址高位
    address_low: int        # Byte 4: 地址低位
    raw_angle: int         # Byte 5: 原始角度
    angle_sign_1: int      # Byte 6: 角度符號1
    angle_sign_2: int      # Byte 7: 角度符號2
    angle_sign_3: int      # Byte 8: 角度符號3
    distance_byte1: int    # Byte 9: 距離位元組1
    distance_byte2: int    # Byte 10: 距離位元組2
    distance_byte3: int    # Byte 11: 距離位元組3
    distance_byte4: int    # Byte 12: 距離位元組4
    reserved2: bytes       # Byte 13-29: 保留
    end_marker: bytes      # Byte 30: 0x23
```

### 濾波數據結構

```python
class FilteredData:
    """
    濾波後的數據結構
    """
    address: str           # 設備地址（十六進制）
    angle: float          # 濾波後角度（度）
    raw_angle: float      # 原始角度（度）
    distance: float       # 距離（米）
    timestamp: float      # 時間戳（Unix時間）
    
    # 可選的調試信息
    angular_velocity: float     # 角速度（度/秒）
    angular_acceleration: float # 角加速度（度/秒²）
    prediction: float          # 預測角度（度）
    control_factor: float      # 控制因子
```

---

## 錯誤處理

### 異常類型

#### `UWBConnectionError`
UWB連接相關錯誤。
```python
class UWBConnectionError(Exception):
    """UWB連接錯誤"""
    pass
```

#### `GPIOError`
GPIO操作相關錯誤。
```python
class GPIOError(Exception):
    """GPIO操作錯誤"""
    pass
```

#### `FilterError`
濾波處理相關錯誤。
```python
class FilterError(Exception):
    """濾波處理錯誤"""
    pass
```

### 錯誤處理範例

```python
try:
    uwb_reader = UWBReader('/dev/ttyS0', 115200)
    if not uwb_reader.start_reading():
        raise UWBConnectionError("無法啟動UWB讀取器")
        
except UWBConnectionError as e:
    logger.error(f"UWB連接錯誤: {e}")
    # 錯誤恢復邏輯
    
except GPIOError as e:
    logger.error(f"GPIO錯誤: {e}")
    # GPIO清理和重初始化
    
except Exception as e:
    logger.error(f"未預期錯誤: {e}")
    # 通用錯誤處理
    
finally:
    # 資源清理
    if 'motor_controller' in locals():
        motor_controller.cleanup()
```

---

## 配置參數

### 系統配置

```python
class SystemConfig:
    """系統配置參數"""
    
    # UWB配置
    UWB_PORT = '/dev/ttyS0'
    UWB_BAUDRATE = 115200
    UWB_TIMEOUT = 1.0
    
    # GPIO配置
    GPIO_FORWARD = 18
    GPIO_LEFT = 23
    GPIO_RIGHT = 24
    GPIO_STOP = 25
    
    # 控制參數
    ANGLE_THRESHOLD_NORMAL = 12.0    # 正常模式角度閾值
    ANGLE_THRESHOLD_CAUTION = 10.0   # 謹慎模式角度閾值
    DISTANCE_STOP = 1.5              # 停止距離
    DISTANCE_CAUTION = 2.5           # 謹慎距離
    
    # 濾波參數
    KALMAN_PROCESS_NOISE = 0.1       # 卡爾曼濾波過程噪聲
    KALMAN_OBSERVATION_NOISE = 1.0   # 卡爾曼濾波觀測噪聲
    ANGLE_JUMP_THRESHOLD = 15.0      # 角度突變檢測閾值
    
    # 預測控制參數
    MAX_PREDICTION_TIME = 0.5        # 最大預測時間
    CONVERGENCE_THRESHOLD = 2.0      # 收斂檢測閾值
    ESTIMATED_DELAY = 0.2            # 估計系統延遲
```

### 動態參數調整

```python
def update_config(config_dict: dict) -> None:
    """
    動態更新配置參數
    
    Args:
        config_dict: 配置參數字典
    """
    for key, value in config_dict.items():
        if hasattr(SystemConfig, key):
            setattr(SystemConfig, key, value)
            logger.info(f"更新配置: {key} = {value}")
        else:
            logger.warning(f"未知配置參數: {key}")

# 使用範例
update_config({
    'ANGLE_THRESHOLD_NORMAL': 15.0,
    'DISTANCE_STOP': 1.2
})
```

---

## 使用範例

### 基本使用

```python
import time
import logging
from source_code import UWBReader, MotorController

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    """基本使用範例"""
    
    # 初始化組件
    uwb_reader = UWBReader('/dev/ttyS0', 115200)
    motor_controller = MotorController()
    
    try:
        # 啟動UWB讀取
        if not uwb_reader.start_reading():
            logger.error("UWB讀取器啟動失敗")
            return
        
        logger.info("系統啟動成功")
        
        # 主控制循環
        while True:
            if uwb_reader.is_data_fresh(timeout=1.0):
                data = uwb_reader.get_latest_data()
                if data:
                    # 控制馬達
                    motor_controller.control_motors(
                        data['angle'], 
                        data['distance'], 
                        data['timestamp']
                    )
            else:
                # 數據超時，停止馬達
                motor_controller.stop()
                logger.warning("UWB數據超時")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("用戶中斷程式")
    except Exception as e:
        logger.error(f"運行錯誤: {e}")
    finally:
        # 清理資源
        uwb_reader.stop_reading()
        motor_controller.cleanup()
        logger.info("程式結束")

if __name__ == "__main__":
    main()
```

### 高級使用（自定義參數）

```python
from source_code import *

def advanced_usage():
    """高級使用範例"""
    
    # 自定義卡爾曼濾波器
    kalman_filter = AdaptiveKalmanFilter(initial_angle=0.0)
    kalman_filter.Q = np.array([[0.05, 0.0], [0.0, 0.3]])  # 調整過程噪聲
    kalman_filter.R_base = np.array([[0.5]])  # 調整觀測噪聲
    
    # 自定義預測控制器
    predictor = PredictiveController()
    predictor.max_prediction_time = 0.3  # 調整最大預測時間
    predictor.convergence_threshold = 1.5  # 調整收斂閾值
    
    # 自定義速度控制器
    speed_controller = AdaptiveSpeedController()
    speed_controller.base_turn_levels['fine'] = 0.12  # 調整精細轉向間隔
    
    logger.info("高級配置完成")

def data_analysis_example():
    """數據分析範例"""
    
    uwb_reader = UWBReader()
    data_history = []
    
    # 收集數據
    if uwb_reader.start_reading():
        for _ in range(100):  # 收集100筆數據
            if uwb_reader.is_data_fresh():
                data = uwb_reader.get_latest_data()
                if data:
                    data_history.append(data)
            time.sleep(0.1)
    
    # 分析數據
    if data_history:
        angles = [d['angle'] for d in data_history]
        distances = [d['distance'] for d in data_history]
        
        import statistics
        print(f"角度統計 - 平均: {statistics.mean(angles):.2f}°, "
              f"標準差: {statistics.stdev(angles):.2f}°")
        print(f"距離統計 - 平均: {statistics.mean(distances):.2f}m, "
              f"標準差: {statistics.stdev(distances):.2f}m")
    
    uwb_reader.stop_reading()
```

---

*API參考文檔版本: 1.0*  
*最後更新: 2025年7月29日*
