#!/usr/bin/env python3
"""통합 상태 표시 노드 (LED + LCD)
- LED: 모드별 색상 표시
- LCD: 배터리, 모드, 상태 표시
- 저전력 최적화: 변화 있을 때만 업데이트
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32
import json
from collections import deque

from slam_mqtt_project.topics import ROS

# LED 라이브러리
try:
    from pinkylib import LED
    LED_AVAILABLE = True
except ImportError:
    LED_AVAILABLE = False

# LCD 라이브러리
try:
    from pinky_lcd import LCD
    from PIL import Image, ImageDraw, ImageFont
    LCD_AVAILABLE = True
except ImportError:
    LCD_AVAILABLE = False

# LED 상수
RED, ORANGE, GREEN, BLUE, OFF = (255, 0, 0), (255, 127, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0)
NUM_LEDS, LUX_THRESHOLD = 8, 100

# LCD 상수
IMG_WIDTH, IMG_HEIGHT = 320, 240
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans"
MODE_COLORS = {
    "SLAM": (0, 100, 200), "NAV2": (100, 0, 200),
    "EXPLORING": (0, 150, 100), "NAVIGATING": (150, 100, 0), "IDLE": (50, 50, 50),
}


class StatusDisplayNode(Node):
    def __init__(self):
        super().__init__('status_display_node')
        
        # ===== LED 초기화 =====
        self.leds = None
        if LED_AVAILABLE:
            try:
                self.leds = LED()
                self.leds.__enter__()
            except Exception as e:
                self.get_logger().warn(f"LED init failed: {e}")
        
        # ===== LCD 초기화 =====
        self.lcd = None
        if LCD_AVAILABLE:
            try:
                self.lcd = LCD()
                self._init_fonts()
            except Exception as e:
                self.get_logger().warn(f"LCD init failed: {e}")
        
        # ===== 공통 상태 =====
        self.robot_mode = "IDLE"
        self.current_status = "Ready"
        self.is_driving = False
        self.current_lux = 0
        self.map_save_count = 0
        
        # LED 상태
        self.current_led_mode = "idle"
        self.last_led_mode = None
        
        # LCD 상태
        self.battery_percent = 50.0
        self.battery_history = deque(maxlen=10)  # 10개 샘플 평균
        self.last_battery_int = None
        self.last_lcd_mode = None
        self.force_lcd_update = True
        
        # ===== 구독자 =====
        self.create_subscription(String, ROS.ROBOT_MODE, self.mode_cb, 1)
        self.create_subscription(String, ROS.ROBOT_STATUS, self.status_cb, 1)
        self.create_subscription(Bool, ROS.AUTO_DRIVE_ACTIVE, self.driving_cb, 1)
        self.create_subscription(String, ROS.MQTT_MCU_SENSORS, self.sensor_cb, 1)
        self.create_subscription(Int32, ROS.MAP_SAVER_SAVED, self.map_saved_cb, 1)
        self.create_subscription(Float32, ROS.BATTERY_PRESENT, self.battery_cb, 1)
        
        # ===== 퍼블리셔 (모드 발행 - mode_publisher 대체) =====
        self.mode_pub = self.create_publisher(String, ROS.ROBOT_MODE, 1)
        self.declare_parameter('mode', 'SLAM')
        self.initial_mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # ===== 타이머 (통합) =====
        self.create_timer(2.0, self.update_led)    # LED: 2초
        self.create_timer(10.0, self.update_lcd)   # LCD: 10초
        self.create_timer(10.0, self.publish_mode) # 모드: 10초
        
        # 초기 모드 발행
        self.robot_mode = self.initial_mode
        self.publish_mode()
        
        self._set_led_all(OFF)
        self.get_logger().info(f"Status Display Node Started (LED+LCD, Mode={self.initial_mode})")

    def _init_fonts(self):
        try:
            self.font_large = ImageFont.truetype(f"{FONT_PATH}-Bold.ttf", 32)
            self.font_medium = ImageFont.truetype(f"{FONT_PATH}.ttf", 24)
            self.font_small = ImageFont.truetype(f"{FONT_PATH}.ttf", 18)
        except:
            self.font_large = self.font_medium = self.font_small = ImageFont.load_default()

    # ===== 콜백 함수들 =====
    def mode_cb(self, msg):
        new_mode = msg.data.upper()
        if self.robot_mode != new_mode:
            self.robot_mode = new_mode
            self.force_lcd_update = True
            self.get_logger().info(f"Mode: {self.robot_mode}")
            
            if self.robot_mode == "NAV2":
                self.current_led_mode = "bright" if self.current_lux >= LUX_THRESHOLD else "dark"

    def status_cb(self, msg):
        if self.current_status != msg.data:
            self.current_status = msg.data
            self.force_lcd_update = True

    def driving_cb(self, msg):
        self.is_driving = msg.data
        if self.robot_mode == "SLAM":
            self.current_led_mode = "driving" if msg.data else "idle"

    def sensor_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if "Lux" in data:
                self.current_lux = float(data["Lux"])
                if self.robot_mode == "NAV2" or not self.is_driving:
                    self.current_led_mode = "bright" if self.current_lux >= LUX_THRESHOLD else "dark"
        except:
            pass

    def map_saved_cb(self, msg):
        old_count = self.map_save_count
        self.map_save_count = msg.data
        self.current_led_mode = "map_saving"
        self.force_lcd_update = True
        # count가 변경되면 LED 강제 업데이트
        if old_count != self.map_save_count:
            self.last_led_mode = None  # 강제 업데이트
            self.get_logger().info(f"🗺️ Map saved: {self.map_save_count}/8")

    def battery_cb(self, msg):
        if 0 <= msg.data <= 100:
            self.battery_history.append(msg.data)
            self.battery_percent = sum(self.battery_history) / len(self.battery_history)

    def publish_mode(self):
        """현재 모드를 주기적으로 발행"""
        msg = String()
        msg.data = self.robot_mode
        self.mode_pub.publish(msg)

    # ===== LED 업데이트 =====
    def _set_led_all(self, color):
        if self.leds:
            try:
                self.leds.fill(color)  # fill이 자동으로 show() 호출
            except Exception as e:
                self.get_logger().warn(f"LED fill error: {e}")

    def _set_led_progress(self, count, total=8):
        """맵 저장 진행률: 저장된 만큼 ORANGE, 나머지 RED"""
        if not self.leds:
            return
        try:
            for i in range(NUM_LEDS):
                # 저장된 개수만큼 ORANGE, 나머지는 RED
                color = ORANGE if i < count else RED
                self.leds.set_pixel(i, color)
            self.leds.show()
            self.get_logger().info(f"💡 LED Progress: {count}/8 (ORANGE:{count}, RED:{8-count})")
        except Exception as e:
            self.get_logger().warn(f"LED progress error: {e}")

    def update_led(self):
        if self.current_led_mode == self.last_led_mode:
            return
        
        self.last_led_mode = self.current_led_mode
        
        if self.current_led_mode == "driving":
            self._set_led_all(RED)
        elif self.current_led_mode == "map_saving":
            self._set_led_progress(self.map_save_count)
        elif self.current_led_mode == "bright":
            self._set_led_all(GREEN)
        elif self.current_led_mode == "dark":
            self._set_led_all(BLUE)
        else:
            self._set_led_all(OFF)

    # ===== LCD 업데이트 =====
    @staticmethod
    def _get_battery_color(pct):
        if pct > 50: return (0, 150, 0)
        elif pct > 20: return (255, 165, 0)
        return (200, 0, 0)

    def update_lcd(self):
        if not self.lcd:
            return
        
        battery_int = int(self.battery_percent)
        
        # 변화 없으면 스킵
        if not self.force_lcd_update:
            if battery_int == self.last_battery_int and self.robot_mode == self.last_lcd_mode:
                return
        
        self.force_lcd_update = False
        self.last_battery_int = battery_int
        self.last_lcd_mode = self.robot_mode
        
        try:
            img = Image.new('RGB', (IMG_WIDTH, IMG_HEIGHT), (0, 0, 0))
            draw = ImageDraw.Draw(img)
            
            # 모드 표시
            mode_color = MODE_COLORS.get(self.robot_mode, (50, 50, 50))
            draw.rectangle([(0, 0), (IMG_WIDTH, 50)], fill=mode_color)
            draw.text((IMG_WIDTH // 2, 25), self.robot_mode, 
                     font=self.font_large, fill=(255, 255, 255), anchor="mm")
            
            # 배터리
            bat_color = self._get_battery_color(battery_int)
            bar_width = int((IMG_WIDTH - 40) * battery_int / 100)
            draw.rectangle([(20, 80), (IMG_WIDTH - 20, 130)], outline=(100, 100, 100), width=2)
            draw.rectangle([(22, 82), (22 + bar_width, 128)], fill=bat_color)
            draw.text((IMG_WIDTH // 2, 105), f"{battery_int}%", 
                     font=self.font_medium, fill=(255, 255, 255), anchor="mm")
            
            # 상태
            draw.text((IMG_WIDTH // 2, 170), self.current_status,
                     font=self.font_medium, fill=(200, 200, 200), anchor="mm")
            
            # 맵 저장 진행률
            if self.map_save_count > 0:
                draw.text((IMG_WIDTH // 2, 210), f"Maps: {self.map_save_count}/8",
                         font=self.font_small, fill=(150, 150, 150), anchor="mm")
            
            self.lcd.img_show(img)
        except Exception as e:
            self.get_logger().error(f"LCD update error: {e}")

    def destroy_node(self):
        if self.leds:
            try:
                self._set_led_all(OFF)
                self.leds.__exit__(None, None, None)
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
