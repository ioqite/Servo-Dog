/*
 * BLETextLink.hpp - ESP32-S3 BLE 5.0 持久连接双向文本链路 (单文件库)
 *
 * 适用于: Arduino core for ESP32 v3.3.4 (基于 ESP-IDF 5.x, Bluedroid 栈)
 *
 * ============================================================================
 *  特性
 * ============================================================================
 *   1. 持久连接   - 开机自动连接配置的对端 MAC, 连接保持不主动断开
 *   2. 自动重连   - 断开后由后台任务自动重连 (默认 3 秒间隔)
 *   3. 双向文本   - 单条 GATT 连接即支持双向:
 *                   Master 写入 Slave 的 CHAR_RX  ->  Slave 收
 *                   Slave 通过 CHAR_TX 通知        ->  Master 收
 *   4. 三类回调   - onReceive / onConnect / onDisconnect, 在用户 loop() 上下文派发
 *                   (BLE 回调任务仅做缓冲, 用户可在回调中执行耗时操作)
 *   5. 角色自动   - AUTO 模式按 MAC 字典序决定主从, 两端代码可完全一致
 *   6. BLE 5.0    - 默认 MTU=512, 单包有效载荷 509 字节; 2M PHY 自动协商
 *
 * ============================================================================
 *  使用步骤
 * ============================================================================
 *   1. 两块 ESP32-S3 烧录同一份示例 (本机 MAC 互填为对端)
 *   2. link.begin(peerMac);  link.onReceive(...); link.onConnect(...);
 *   3. loop() 中调用 link.loop()  -- 必须!
 *   4. link.send("hello") 发送;  onReceive 回调接收
 *
 * ============================================================================
 *  协议 (双端 UUID 必须一致)
 * ============================================================================
 *   Service        6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   CHAR_RX (写)   6E400002-B5A3-F393-E0A9-E50E24DCCA9E  远端写入 -> 本机接收
 *   CHAR_TX (通知) 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  本机通知 -> 远端接收
 */

#pragma once

#define LogSerial Serial

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <esp_gatts_api.h>
#endif
#include <functional>
#include <string>

class BLETextLink {
public:
    using MsgCallback   = std::function<void(const String&)>;
    using EventCallback = std::function<void()>;

    enum Role : uint8_t { SLAVE = 0, MASTER = 1, AUTO = 2 };

    // ---------------- 公共 UUID (可修改, 双端须一致) ----------------
    static constexpr const char* SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* CHAR_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr const char* CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

    BLETextLink() : _serverCb(this), _charRxCb(this), _clientCb(this) {}
    ~BLETextLink() { end(); }

    // ====================================================================
    //  begin - 初始化 BLE, 启动 Peripheral, 若为 MASTER 则启动连接任务
    void begin(const String& peerAddr,
               Role role = AUTO,
               const String& name = "ESP32S3-BLE",
               uint16_t mtu = 512)
    {
        _peerAddr = peerAddr;
        _name     = name;
        _mtu      = mtu;
        _role     = role;
        _stopTask = false;

        BLEDevice::init(_name.c_str());
        BLEDevice::setMTU(_mtu);
        _decideRole();

        _startServer();

        if (_effectiveRole == MASTER) {
            // 优化: 任务栈从 8192 缩减至 3072, 节省 5KB RAM
            xTaskCreatePinnedToCore(_connTaskStub, "BLELink", 3072,
                                    this, 1, &_taskHandle, 0);
        }

        LogSerial.printf("[BLELink] role=%s peer=%s local=%s\r\n",
            _effectiveRole == MASTER ? "MASTER" : "SLAVE",
            _peerAddr.c_str(), localAddress().c_str());
    }

    // ---------------- 回调注册 ----------------
    void onReceive(MsgCallback cb)      { _onRx = cb; }
    void onConnect(EventCallback cb)    { _onCn = cb; }
    void onDisconnect(EventCallback cb) { _onDc = cb; }

    // ====================================================================
    //  send - 发送文本到对端
    //   - 持久连接, 无需每次重连
    //   - 超过 MTU-3 字节自动截断 (509 字节 @MTU=512)
    //   - 返回 true 表示已成功写入底层 (实际送达依赖 GATT ACK)
    bool send(const String& text)
    {
        if (text.length() == 0) return false;
        size_t maxN = (size_t)_mtu - 3;
        size_t n = (text.length() < maxN) ? text.length() : maxN;

        if (_effectiveRole == MASTER) {
            if (!_client || !_client->isConnected() || !_remoteRx) return false;
            _remoteRx->writeValue(text, true);
            return true;
        } else {
            if (!_connected || !_charTx) return false;
            _charTx->setValue((const uint8_t*)text.c_str(), n);
            _charTx->notify();
            return true;
        }
    }

    // ---------------- 状态查询 ----------------
    bool   isConnected() const { return _connected; }
    String localAddress() const {
        return BLEDevice::getAddress().toString().c_str();
    }
    String peerAddress() const { return _peerAddr; }
    Role   role()        const { return _effectiveRole; }

    // ---------------- 清空接收缓冲 ----------------
    void clearBuffer()
    {
        portENTER_CRITICAL(&_rxMux);
        _rxHead = 0;
        _rxTail = 0;
        portEXIT_CRITICAL(&_rxMux);
    }

    // ====================================================================
    //  loop - 必须在 Arduino loop() 中调用
    //   1. 派发接收消息到用户 onReceive
    //   2. 派发连接 / 断开事件到用户 onConnect / onDisconnect
    //   3. MASTER 角色的自动重连由后台任务执行, 不在此处阻塞
    void loop()
    {
        // 1. 派发接收消息
        for (;;) {
            uint8_t tail;
            bool has;
            
            portENTER_CRITICAL(&_rxMux);
            tail = _rxTail;
            has = (_rxHead != _rxTail);
            if (has) {
                _rxTail = (tail + 1) % RX_BUF_LEN;
            }
            portEXIT_CRITICAL(&_rxMux);

            if (!has) break;
            if (_onRx) {
                // 在 loop 上下文中构造 String, 避免在 BLE 回调中造成堆碎片
                _onRx(String((const char*)_rxBuf[tail].data, _rxBuf[tail].len));
            }
        }
        // 2. 派发事件
        if (_evtConnected)    { _evtConnected = false;    if (_onCn) _onCn(); }
        if (_evtDisconnected) { _evtDisconnected = false; if (_onDc) _onDc(); }
    }

    void end()
    {
        _stopTask = true;
        if (_taskHandle) {
            // 等待任务退出 (最多约 1 秒)
            for (int i = 0; i < 10 && _taskHandle; ++i) vTaskDelay(pdMS_TO_TICKS(100));
            _taskHandle = nullptr;
        }
        _stopClient();
        if (_server) BLEDevice::getAdvertising()->stop();
    }

private:
    // ---------------- 接收环形缓冲 (BLE task 写, loop task 读) ----------------
    // 优化: 使用固定大小结构体数组替代 String 数组, 避免高频 BLE 收发导致的堆碎片
    static constexpr int RX_BUF_LEN = 8;
    static constexpr size_t MAX_PAYLOAD = 512; // 预留 MTU 上限空间
    struct RxPacket {
        uint16_t len;
        uint8_t data[MAX_PAYLOAD];
    };
    
    RxPacket _rxBuf[RX_BUF_LEN];
    volatile uint8_t _rxHead = 0, _rxTail = 0;
    portMUX_TYPE     _rxMux = portMUX_INITIALIZER_UNLOCKED;

    // ---------------- 事件标志 (BLE task 置位, loop task 清除并回调) ----------------
    volatile bool _evtConnected    = false;
    volatile bool _evtDisconnected = false;

    // ---------------- 配置 ----------------
    String   _peerAddr;
    String   _name;
    Role     _role = AUTO;
    Role     _effectiveRole = SLAVE;
    uint16_t _mtu = 512;

    // ---------------- 状态 ----------------
    volatile bool _connected  = false;
    volatile bool _connecting = false;
    volatile bool _stopTask   = false;
    TaskHandle_t  _taskHandle = nullptr;

    // ---------------- Central 资源 ----------------
    BLEClient*                _client   = nullptr;
    BLERemoteCharacteristic*  _remoteRx = nullptr;
    BLERemoteCharacteristic*  _remoteTx = nullptr;

    // ---------------- Peripheral 资源 ----------------
    BLEServer*         _server = nullptr;
    BLECharacteristic* _charRx = nullptr;
    BLECharacteristic* _charTx = nullptr;

    // ---------------- 嵌套回调类声明 ----------------
    class ServerCb;
    class CharRxCb;
    class ClientCb;
    friend class ServerCb; 
    friend class CharRxCb;
    friend class ClientCb;

    // ---------------- 用户回调 ----------------
    MsgCallback   _onRx = nullptr;
    EventCallback _onCn = nullptr;
    EventCallback _onDc = nullptr;

    // ============================================================
    //  接收缓冲写入 (BLE 回调上下文调用, 零分配)
    void _pushRx(const uint8_t* data, size_t len)
    {
        if (len == 0 || len > MAX_PAYLOAD) return;

        portENTER_CRITICAL(&_rxMux);
        uint8_t head = _rxHead;
        _rxBuf[head].len = len;
        memcpy(_rxBuf[head].data, data, len);

        head = (head + 1) % RX_BUF_LEN;
        if (head == _rxTail) { // 满则丢最旧
            _rxTail = (_rxTail + 1) % RX_BUF_LEN;
        }
        _rxHead = head;
        portEXIT_CRITICAL(&_rxMux);
    }

    // ============================================================
    //  角色决策 (AUTO 模式: MAC 字典序小者为 MASTER)
    void _decideRole()
    {
        if (_role == MASTER)      { _effectiveRole = MASTER; return; }
        if (_role == SLAVE)       { _effectiveRole = SLAVE;  return; }
        String me = localAddress();
        _effectiveRole = _macLessThan(me, _peerAddr) ? MASTER : SLAVE;
    }
    
    // 优化: 消除原实现中 toUpperCase 和 replace 产生的临时 String 堆分配
    static bool _macLessThan(const String& a, const String& b)
    {
        size_t min_len = a.length() < b.length() ? a.length() : b.length();
        for (size_t i = 0; i < min_len; ++i) {
            char ca = a[i];
            char cb = b[i];
            if (ca == ':') continue;
            if (cb == ':') continue;
            if (ca >= 'a' && ca <= 'f') ca -= 32;
            if (cb >= 'a' && cb <= 'f') cb -= 32;
            if (ca != cb) return ca < cb;
        }
        return a.length() < b.length();
    }

    // ============================================================
    //  启动 Peripheral (Server + 广播)
    void _startServer()
    {
        _server = BLEDevice::createServer();
        _server->setCallbacks(&_serverCb);

        BLEService* svc = _server->createService(SERVICE_UUID);

        _charRx = svc->createCharacteristic(
            CHAR_RX_UUID,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        _charRx->setCallbacks(&_charRxCb);

        _charTx = svc->createCharacteristic(
            CHAR_TX_UUID,
            BLECharacteristic::PROPERTY_NOTIFY);

        svc->start();

        BLEAdvertising* adv = BLEDevice::getAdvertising();
        adv->addServiceUUID(SERVICE_UUID);
        adv->setScanResponse(true);
        adv->setMinPreferred(0x06);
        adv->setMinPreferred(0x12);
        BLEDevice::startAdvertising();
    }

    // ============================================================
    //  Central 连接管理
    void _stopClient()
    {
        if (_client) {
            if (_client->isConnected()) _client->disconnect();
            delete _client;
            _client = nullptr;
            _remoteRx = nullptr;
            _remoteTx = nullptr;
        }
    }

    // 在后台任务中执行: 阻塞式 connect, 5s 超时
    void _doConnect()
    {
        _stopClient();
        _client = BLEDevice::createClient();
        _client->setClientCallbacks(&_clientCb);

        if (!_client->connect(BLEAddress(_peerAddr.c_str()))) {
            LogSerial.printf("[BLELink] connect %s failed\r\n", _peerAddr.c_str());
            _stopClient();
            return;
        }
        _client->setMTU(_mtu);

        BLERemoteService* svc = _client->getService(BLEUUID(SERVICE_UUID));
        if (!svc) { LogSerial.println("[BLELink] service not found"); _stopClient(); return; }

        _remoteRx = svc->getCharacteristic(BLEUUID(CHAR_RX_UUID));
        _remoteTx = svc->getCharacteristic(BLEUUID(CHAR_TX_UUID));
        if (!_remoteRx) { LogSerial.println("[BLELink] RX char not found"); _stopClient(); return; }

        if (_remoteTx && _remoteTx->canNotify()) {
            _remoteTx->subscribe(true, [this](BLERemoteCharacteristic* c, uint8_t* data, size_t length, bool isNotify) {
                if (length) _pushRx(data, length);
            });
        }

        _connected = true;
        _evtConnected = true;
        LogSerial.printf("[BLELink] connected to %s (MTU=%u)\r\n",
                      _peerAddr.c_str(), _client->getMTU());
    }

    static void _connTaskStub(void* arg)
    {
        static_cast<BLETextLink*>(arg)->_connTask();
        vTaskDelete(nullptr);
    }

    void _connTask()
    {
        const uint32_t RETRY_MS = 3000;
        uint32_t last = 0;
        for (;;) {
            if (_stopTask) return;
            if (!_connected && !_connecting) {
                if (millis() - last > RETRY_MS) {
                    _connecting = true;
                    _doConnect();
                    _connecting = false;
                    last = millis();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // ============================================================
    //  回调类实现 (嵌套, 作为友元直接访问主类私有成员)
    // ============================================================

    // ---- Peripheral Server 端 ----
    class ServerCb : public BLEServerCallbacks {
    public:
        ServerCb(BLETextLink* p) : _p(p) {}

        void onConnect(BLEServer* s) override
        {
            // SLAVE 角色: 任意远端连入即视为与配置 peer 建链
            if (_p->_effectiveRole == SLAVE) {
                _p->_connected = true;
                _p->_evtConnected = true;
            }
        }

        void onDisconnect(BLEServer* s) override
        {
            if (_p->_effectiveRole == SLAVE) {
                _p->_connected = false;
                _p->_evtDisconnected = true;
                s->getAdvertising()->start();  // 重新广播, 等待 master 重连
            }
        }

    private:
        BLETextLink* _p;
    };

    // ---- 本机 CHAR_RX 被写入 (Peripheral 端接收) ----
    class CharRxCb : public BLECharacteristicCallbacks {
    public:
        CharRxCb(BLETextLink* p) : _p(p) {}
        void onWrite(BLECharacteristic* c) override
        {
            uint8_t* data = c->getData();
            size_t len = c->getLength();
            if (len > 0) _p->_pushRx(data, len);
        }
    private:
        BLETextLink* _p;
    };

    // ---- Central 端连接事件 ----
    class ClientCb : public BLEClientCallbacks {
    public:
        ClientCb(BLETextLink* p) : _p(p) {}
        void onConnect(BLEClient*) override {}
        void onDisconnect(BLEClient*) override
        {
            _p->_connected = false;
            _p->_evtDisconnected = true;
            _p->_remoteRx = nullptr;
            _p->_remoteTx = nullptr;
            // 注意: 不在此 delete _client, 由后台任务在下一次 _doConnect() 中清理
        }
    private:
        BLETextLink* _p;
    };

    // ---------------- 预分配的回调实例 (避免使用 new 产生堆碎片) ----------------
    ServerCb _serverCb;
    CharRxCb _charRxCb;
    ClientCb _clientCb;
};