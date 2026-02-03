import asyncio
from bleak import BleakClient

hand_mac = "10:B4:1D:E9:3B:B6"

# 掉线时的回调函数
def on_disconnect(client):
    print("⚠️ 设备已断开连接！")

async def run_ble():
    while True:  # 外层死循环实现重连机制
        print(f"尝试连接 {hand_mac}...")
        try:
            async with BleakClient(hand_mac, disconnected_callback=on_disconnect) as client:
                print("✅ 已连接")
                print("\n--- 服务列表 ---")
                for service in client.services:
                    print(f"Service: {service.uuid}")
                    for char in service.characteristics:
                        print(f"  └─ Characteristic: {char.uuid} (Props: {char.properties})")
                # 在这里保持运行
                while client.is_connected:
                    await asyncio.sleep(1) # 维持心跳或执行任务
                    
        except Exception as e:
            print(f"连接失败或出错: {e}")
            print("5秒后尝试重连...")
            await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(run_ble())