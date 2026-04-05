#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


async def wait_until_connected(drone):
    print("Bağlantı bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlandı.")
            return


async def wait_until_ready(drone):
    print("Local position bekleniyor...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Hazır.")
            return


async def goto(drone, north, east, down, yaw_deg=0.0, wait_sec=6):
    print(f"Setpoint → N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_ready(drone)

    print("İlk setpoint gönderiliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -1.5, 0.0)
    )

    print("Arm...")
    await drone.action.arm()

    print("Offboard start...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard başlatılamadı: {e._result.result}")
        await drone.action.disarm()
        return

    try:
        print("Safe scan başlıyor...")

        # 1) Kalkış / hover
        await goto(drone, 0.0, 0.0, -1.5, 0.0, 6)

        # 2) Küçük güvenli tarama alanı
        # Girişe yakın, kısa sweep
        await goto(drone, 1.5, 0.0, -1.5, 0.0, 7)
        await goto(drone, 1.5, 1.0, -1.5, 0.0, 6)
        await goto(drone, 0.5, 1.0, -1.5, 0.0, 7)
        await goto(drone, 0.5, 2.0, -1.5, 0.0, 6)
        await goto(drone, 1.8, 2.0, -1.5, 0.0, 7)

        # 3) Kısa hover
        print("Hover...")
        await asyncio.sleep(5)

        # 4) Başlangıca yakın dönüş
        await goto(drone, 0.5, 0.5, -1.5, 0.0, 6)

    finally:
        print("Offboard stop...")
        try:
            await drone.offboard.stop()
        except Exception as e:
            print(f"Offboard stop uyarısı: {e}")

        print("Landing...")
        await drone.action.land()
        await asyncio.sleep(10)

        print("Safe scan tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
