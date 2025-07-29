# 項目檔案總覽

## 檔案結構

```
高球車程式碼整理/
├── source code.txt          # 主程式碼檔案
├── README.md               # 項目說明文件
├── TECHNICAL_DOCS.md       # 技術文檔
├── INSTALLATION_GUIDE.md   # 安裝配置指南
├── API_REFERENCE.md        # API參考文檔
└── PROJECT_OVERVIEW.md     # 本檔案 - 項目總覽
```

## 文檔說明

### 1. README.md
**主要說明文件** - 包含系統概述、硬體架構、軟體架構等基本信息
- 系統硬體需求和GPIO配置
- UWB數據格式和通訊協議
- 控制算法概述
- 安全機制說明
- 性能參數和技術規格

### 2. TECHNICAL_DOCS.md  
**技術深度分析** - 詳細的程式碼架構和數學模型分析
- 各類別的詳細分析和方法說明
- 卡爾曼濾波器和預測控制的數學模型
- 性能分析和計算複雜度
- 優化建議和測試策略
- 故障診斷流程圖

### 3. INSTALLATION_GUIDE.md
**安裝部署指南** - 從硬體連接到軟體配置的完整指南
- 詳細的硬體連接說明和GPIO配置
- Raspberry Pi系統配置步驟
- 軟體依賴安裝和項目部署
- 測試驗證和故障排除
- 系統監控和維護指南

### 4. API_REFERENCE.md
**API接口文檔** - 完整的API參考和使用範例
- 所有類別的構造函數和方法說明
- 參數類型、返回值和使用範例
- 錯誤處理和異常類型
- 配置參數和動態調整
- 基本和高級使用範例

## 程式碼架構摘要

### 主要類別
1. **UWBReader** - UWB數據接收和UART通訊
2. **AdaptiveKalmanFilter** - 自適應卡爾曼濾波器
3. **PredictiveController** - 預測控制和延遲補償
4. **AdaptiveSpeedController** - 自適應速度控制
5. **MotorController** - GPIO馬達控制

### 核心功能
- **智能濾波**: 使用自適應卡爾曼濾波處理UWB角度數據
- **預測控制**: 補償系統延遲，防止過度轉向
- **分層安全**: 多層次的安全檢查和故障保護
- **動態調整**: 根據距離和角度動態調整控制參數

## 快速開始

### 硬體準備
1. Raspberry Pi 4 + SD卡(32GB+)
2. DW1000模組連接到UART接口(/dev/ttyS0)
   - 規格: 6.5GHz, 100Hz更新, ±5°角度精度, ±10cm距離精度
   - 供電: 5V/1A, TTL3.3V UART輸出
3. 馬達驅動板連接到GPIO(18,23,24,25)

### 軟體安裝
```bash
# 1. 啟用UART
sudo raspi-config  # Interfacing Options -> Serial

# 2. 安裝依賴
pip3 install pyserial numpy RPi.GPIO

# 3. 運行程式
python3 source_code.py
```

### 基本測試
```bash
# 測試UWB連接
python3 -c "import serial; ser=serial.Serial('/dev/ttyS0',115200); print('UWB OK')"

# 測試GPIO
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO OK')"
```


---

*最後更新: 2025年7月*
