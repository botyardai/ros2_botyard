# 灵巧手设备身份信息：连接成功后由硬件层发布一次（transient_local），供控制器层组装 Service 响应。
# 当前是否已连接
bool connected
# 产品系列：0=S1, 1=S2（对应 driver dexhand_driver::HandSeries）
uint8 series
# 整机版本号原始串，如 "[S1.2]2026.3.1-rel-a3c2e4d"；探测失败为空
string version
