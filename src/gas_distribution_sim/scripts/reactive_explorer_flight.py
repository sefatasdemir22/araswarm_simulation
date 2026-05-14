#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import math
import threading

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ALTITUDE = -1.5  # NED: negative means up
BASE_YAW = 90.0  # +Y mağara yönü
LEFT_CHECK_YAW = 45.0
RIGHT_CHECK_YAW = 135.0
YAW_TOWARDS_CAVE = BASE_YAW
STEP_Y = 1.0
MAX_Y = 10.0
SWEEP_X = 0.4
MAX_STEPS = 10
SECTOR_SIZE = 5
FORWARD_AVG_CLEARANCE_M = 3.5
FORWARD_MIN_CLEARANCE_M = 2.0
ESCAPE_MIN_CLEARANCE_M = 1.3
AVOIDANCE_STEP_M = 0.6
ESCAPE_FOLLOW_STEPS = 2
MAX_CONSECUTIVE_OBSTACLES = 3
FRONT_SCAN_TOPIC = '/drone/front_scan'

APPROACH_Y_LEVELS = [2.0, 4.0, 6.0, 8.0]
SCAN_Y_LEVELS = [9.0, 10.0]


class FrontScanMonitor(Node):
    def __init__(self):
        super().__init__('front_scan_monitor')
        self.latest_stats = None
        self.create_subscription(LaserScan, FRONT_SCAN_TOPIC, self.scan_callback, 10)
        self.get_logger().info(f'Ön laser scan topic dinleniyor: {FRONT_SCAN_TOPIC}')

    def scan_callback(self, msg):
        ranges = self._normalize_ranges(msg.ranges, msg.range_max)
        if len(ranges) < SECTOR_SIZE * 3:
            self.get_logger().warning(
                f'Yetersiz scan verisi: {len(ranges)} ray geldi, en az {SECTOR_SIZE * 3} gerekli.'
            )
            return

        left = ranges[:SECTOR_SIZE]
        center_start = (len(ranges) - SECTOR_SIZE) // 2
        center = ranges[center_start:center_start + SECTOR_SIZE]
        right = ranges[-SECTOR_SIZE:]

        left_min, left_avg = self._sector_stats(left)
        center_min, center_avg = self._sector_stats(center)
        right_min, right_avg = self._sector_stats(right)
        decision = self._make_decision(left_min, left_avg, center_min, center_avg, right_min, right_avg)

        self.latest_stats = {
            'left_min': left_min,
            'left_avg': left_avg,
            'center_min': center_min,
            'center_avg': center_avg,
            'right_min': right_min,
            'right_avg': right_avg,
            'decision': decision,
        }

    def is_front_clear(self):
        if self.latest_stats is None:
            print("UYARI: Henüz front_scan verisi yok, hareket kontrollü devam ediyor.")
            return True
        return self.latest_stats['decision'] == 'forward'

    def selected_escape_direction(self):
        if self.latest_stats is None:
            return None
        if self.latest_stats['decision'] == 'left':
            return 'left'
        if self.latest_stats['decision'] == 'right':
            return 'right'
        return None

    def describe_scan(self):
        if self.latest_stats is None:
            return "front_scan=veri_yok"

        return (
            f"Sol min/avg: {self.latest_stats['left_min']:.2f} / {self.latest_stats['left_avg']:.2f} m | "
            f"Orta min/avg: {self.latest_stats['center_min']:.2f} / {self.latest_stats['center_avg']:.2f} m | "
            f"Sağ min/avg: {self.latest_stats['right_min']:.2f} / {self.latest_stats['right_avg']:.2f} m | "
            f"Karar: {self.decision_text()}"
        )

    def log_scan_status(self, current_yaw, escape_follow_remaining):
        print(
            f"Forward avg/min clearance: "
            f"{FORWARD_AVG_CLEARANCE_M:.1f} / {FORWARD_MIN_CLEARANCE_M:.1f} m | "
            f"Escape min clearance: {ESCAPE_MIN_CLEARANCE_M:.1f} m"
        )
        print(self.describe_scan())
        print(f"current_yaw={current_yaw:.1f}, escape_follow_remaining={escape_follow_remaining}")

    def decision_text(self):
        if self.latest_stats is None:
            return "VERİ YOK"
        decision = self.latest_stats['decision']
        if decision == 'forward':
            return "İLERİ AÇIK"
        if decision == 'left':
            return "SOLA KAÇ"
        if decision == 'right':
            return "SAĞA KAÇ"
        return "DUR / GERİ DÖN"

    @staticmethod
    def _normalize_ranges(raw_ranges, range_max):
        normalized = []
        for value in raw_ranges:
            if math.isinf(value):
                normalized.append(float(range_max))
            elif math.isnan(value):
                normalized.append(0.0)
            else:
                normalized.append(float(value))
        return normalized

    @staticmethod
    def _sector_stats(values):
        if not values:
            return 0.0, 0.0
        return min(values), sum(values) / len(values)

    @staticmethod
    def _make_decision(left_min, left_avg, center_min, center_avg, right_min, right_avg):
        center_clear = (
            center_avg > FORWARD_AVG_CLEARANCE_M and
            center_min > FORWARD_MIN_CLEARANCE_M
        )
        if center_clear:
            return 'forward'

        left_clear = left_min > ESCAPE_MIN_CLEARANCE_M
        right_clear = right_min > ESCAPE_MIN_CLEARANCE_M
        if left_clear and left_avg > right_avg:
            return 'left'
        if right_clear and right_avg > left_avg:
            return 'right'
        return 'stop'


def start_front_scan_monitor():
    rclpy.init(args=None)
    monitor = FrontScanMonitor()
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()
    return monitor


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


async def goto(drone, north, east, down=ALTITUDE, yaw_deg=YAW_TOWARDS_CAVE, wait_sec=5):
    print(f"Setpoint -> N:{north:.2f} E:{east:.2f} D:{down:.2f} Yaw:{yaw_deg:.1f}")
    await drone.offboard.set_position_ned(
        PositionNedYaw(north, east, down, yaw_deg)
    )
    await asyncio.sleep(wait_sec)


def should_continue_forward(current_y):
    return current_y <= MAX_Y


def should_scan_here(current_y):
    return current_y in SCAN_Y_LEVELS


def projected_step(current_x, current_y, yaw_deg, distance_m):
    if yaw_deg == LEFT_CHECK_YAW:
        return current_x + distance_m, current_y + distance_m
    if yaw_deg == RIGHT_CHECK_YAW:
        return current_x - distance_m, current_y + distance_m
    return current_x, current_y + distance_m


async def attempt_escape(
    drone,
    scan_monitor,
    current_x,
    current_y,
    last_escape_direction,
    current_yaw,
    escape_follow_remaining
):
    scan_monitor.log_scan_status(current_yaw, escape_follow_remaining)
    selected = scan_monitor.selected_escape_direction()
    if selected is None:
        print("LaserScan sektör kararı güvenli kaçış yönü bulamadı. Güvenli geri dönüş moduna geçiliyor.")
        return False, current_x, current_y, last_escape_direction, BASE_YAW, 0

    selected_label = "sol" if selected == 'left' else "sağ"
    print(f"Seçilen kaçış yönü: {selected_label}")

    return await move_escape(drone, current_x, current_y, selected)


async def move_escape(drone, current_x, current_y, direction):
    yaw = LEFT_CHECK_YAW if direction == 'left' else RIGHT_CHECK_YAW
    label = "sola" if direction == 'left' else "sağa"
    escape_x, escape_y = projected_step(current_x, current_y, yaw, AVOIDANCE_STEP_M)

    print(f"KAÇIŞ BAŞLADI: {'sol' if direction == 'left' else 'sağ'}")
    print(f"{label.capitalize()} küçük kaçış yapılıyor: x={escape_x:.2f}, y={escape_y:.2f}")
    await goto(drone, escape_x, escape_y, ALTITUDE, yaw, 2)
    print(f"KAÇIŞ TAMAMLANDI. Sonraki {ESCAPE_FOLLOW_STEPS} ileri adım yaw={yaw:.1f} ile sürecek.")

    return True, escape_x, escape_y, direction, yaw, ESCAPE_FOLLOW_STEPS


async def refresh_base_yaw_if_possible(
    drone,
    scan_monitor,
    current_x,
    current_y,
    current_yaw
):
    if current_yaw == BASE_YAW:
        return current_yaw, 0

    print("Kaçış takip adımları tamamlandı. BASE_YAW yönü kontrol ediliyor...")
    await goto(drone, current_x, current_y, ALTITUDE, BASE_YAW, 1)
    scan_monitor.log_scan_status(BASE_YAW, 0)
    if scan_monitor.latest_stats is None:
        print("BASE_YAW kontrolünde front_scan verisi yok. Ana yöne dönülüyor.")
        return BASE_YAW, 0

    if scan_monitor.is_front_clear():
        print("BASE_YAW yönü açık. Ana mağara yönüne dönülüyor.")
        return BASE_YAW, 0

    print("BASE_YAW hâlâ kapalı. Mevcut kaçış yaw yönünde devam edilecek.")
    await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 1)
    return current_yaw, ESCAPE_FOLLOW_STEPS


async def guarded_forward(
    drone,
    scan_monitor,
    current_x,
    current_y,
    target_y,
    last_escape_direction,
    consecutive_obstacle_count,
    current_yaw,
    escape_follow_remaining
):
    if escape_follow_remaining <= 0:
        current_yaw, escape_follow_remaining = await refresh_base_yaw_if_possible(
            drone, scan_monitor, current_x, current_y, current_yaw
        )

    scan_monitor.log_scan_status(current_yaw, escape_follow_remaining)
    if scan_monitor.is_front_clear():
        print(
            "Ön sektör açık. "
            f"Yaw {current_yaw:.1f} ile ileri gidiliyor."
        )
        if current_yaw == BASE_YAW:
            next_x, next_y = current_x, target_y
        else:
            next_x, next_y = projected_step(current_x, current_y, current_yaw, STEP_Y)
            if next_y > target_y:
                next_y = target_y

        await goto(drone, next_x, next_y, ALTITUDE, current_yaw, 3)
        if escape_follow_remaining > 0:
            escape_follow_remaining -= 1
            print(f"Kaçış yaw takip kalan adım: {escape_follow_remaining}")
        return True, next_x, next_y, last_escape_direction, 0, current_yaw, escape_follow_remaining

    print("ENGEL ALGILANDI. Orta sektör güvenli değil, ileri hareket durduruldu.")
    print("Hover/stabil bekleme yapılıyor...")
    await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 1)
    consecutive_obstacle_count += 1
    if consecutive_obstacle_count >= MAX_CONSECUTIVE_OBSTACLES:
        print("Üst üste 3 engel denemesi başarısız. Güvenli geri dönüş moduna geçiliyor.")
        return (
            False,
            current_x,
            current_y,
            last_escape_direction,
            consecutive_obstacle_count,
            current_yaw,
            escape_follow_remaining
        )

    ok, next_x, next_y, next_escape_direction, next_yaw, next_follow_remaining = await attempt_escape(
        drone,
        scan_monitor,
        current_x,
        current_y,
        last_escape_direction,
        current_yaw,
        escape_follow_remaining
    )
    if not ok:
        return (
            False,
            next_x,
            next_y,
            next_escape_direction,
            consecutive_obstacle_count,
            next_yaw,
            next_follow_remaining
        )

    return True, next_x, next_y, next_escape_direction, 0, next_yaw, next_follow_remaining


async def soft_land(drone):
    print("Yumuşak iniş hazırlığı...")

    await goto(drone, 0.0, 0.5, -1.2, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.9, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.6, YAW_TOWARDS_CAVE, 4)
    await goto(drone, 0.0, 0.5, -0.4, YAW_TOWARDS_CAVE, 4)

    print("Offboard stop...")
    try:
        await drone.offboard.stop()
    except Exception as e:
        print(f"Offboard stop uyarısı: {e}")

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(20)


async def main():
    scan_monitor = start_front_scan_monitor()
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_until_connected(drone)
    await wait_until_ready(drone)

    print("İlk setpoint gönderiliyor...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, ALTITUDE, YAW_TOWARDS_CAVE)
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

    path = [(0.0, 0.0)]
    current_x = 0.0
    current_y = 0.0
    last_escape_direction = None
    consecutive_obstacle_count = 0
    current_yaw = BASE_YAW
    escape_follow_remaining = 0

    try:
        print("Reactive explorer deneme uçuşu başlıyor...")
        await goto(drone, 0.0, 0.0, ALTITUDE, BASE_YAW, 6)

        print("FAZ 1: Mağara girişine düz +Y hattından yaklaşma...")
        step_count = 0
        abort_exploration = False
        for target_y in APPROACH_Y_LEVELS:
            if not should_continue_forward(target_y):
                print("İleri keşif sınırı görüldü, yaklaşma durduruluyor.")
                break

            step_count += 1
            if step_count > MAX_STEPS:
                print("Maksimum adım sayısına ulaşıldı.")
                break

            print(f"Yaklaşma noktası: y={target_y:.1f}")
            (
                ok,
                current_x,
                current_y,
                last_escape_direction,
                consecutive_obstacle_count,
                current_yaw,
                escape_follow_remaining
            ) = await guarded_forward(
                drone,
                scan_monitor,
                current_x,
                current_y,
                target_y,
                last_escape_direction,
                consecutive_obstacle_count,
                current_yaw,
                escape_follow_remaining
            )
            if not ok:
                abort_exploration = True
                break
            path.append((current_x, current_y))

        if not abort_exploration:
            print("FAZ 2: Mağara içinde kısa X sağ-sol taraması...")

        for target_y in SCAN_Y_LEVELS:
            if abort_exploration:
                break

            if path[-1][1] != target_y:
                (
                    ok,
                    current_x,
                    current_y,
                    last_escape_direction,
                    consecutive_obstacle_count,
                    current_yaw,
                    escape_follow_remaining
                ) = await guarded_forward(
                    drone,
                    scan_monitor,
                    current_x,
                    path[-1][1],
                    target_y,
                    last_escape_direction,
                    consecutive_obstacle_count,
                    current_yaw,
                    escape_follow_remaining
                )
                if not ok:
                    abort_exploration = True
                    break
                path.append((current_x, current_y))

            if not should_continue_forward(target_y):
                print("İleri keşif sınırı görüldü, scan durduruluyor.")
                break

            step_count += 1
            if step_count > MAX_STEPS:
                print("Maksimum adım sayısına ulaşıldı.")
                break

            print(f"Scan merkez noktası: y={target_y:.1f}")
            await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 2)

            if should_scan_here(target_y):
                print("Kısa sağ-sol koridor taraması yapılıyor...")
                await goto(drone, current_x + SWEEP_X, current_y, ALTITUDE, current_yaw, 2)
                await goto(drone, current_x - SWEEP_X, current_y, ALTITUDE, current_yaw, 2)
                await goto(drone, current_x, current_y, ALTITUDE, current_yaw, 2)

        print("FAZ 3: Kaydedilen path üzerinden geri dönülüyor...")
        for north, east in reversed(path[1:-1]):
            await goto(drone, north, east, ALTITUDE, BASE_YAW, 3)

        print("Başlangıca yakın iniş noktasına geçiliyor...")
        await goto(drone, 0.0, 0.5, ALTITUDE, BASE_YAW, 5)

    finally:
        await soft_land(drone)
        scan_monitor.destroy_node()
        rclpy.shutdown()
        print("Reactive explorer uçuşu tamamlandı.")


if __name__ == "__main__":
    asyncio.run(main())
