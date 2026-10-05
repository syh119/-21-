#!/usr/bin/env python3
import os
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("Pillow is required for oled_text_renderer") from exc


OLED_WIDTH = 128
OLED_HEIGHT = 64
TOP_RESERVED_HEIGHT = 16
CONTENT_HEIGHT = OLED_HEIGHT - TOP_RESERVED_HEIGHT
ROW_COUNT = CONTENT_HEIGHT // 8
OLED_BITMAP_TOPIC = "/oled_bitmap"
PAGE_PERIOD_SEC = 2.0
REFRESH_PERIOD_SEC = 0.5
DEFAULT_FONT_SIZE = 12
MIN_FONT_SIZE = 10
LINE_GAP = 2
MAX_LINES_PER_PAGE = 2
VERTICAL_SHIFT = 8


def first_existing(paths: List[str]) -> str:
    for path in paths:
        if path and Path(path).is_file():
            return path
    return ""


class OledTextRenderer(Node):
    def __init__(self) -> None:
        super().__init__("oled_text_renderer")
        self.declare_parameter("font_path", "")
        self.declare_parameter("font_size", DEFAULT_FONT_SIZE)

        font_path = self.get_parameter("font_path").get_parameter_value().string_value
        font_size = self.get_parameter("font_size").get_parameter_value().integer_value or DEFAULT_FONT_SIZE

        self.font_path = self.resolve_font_path(font_path)
        self.base_font_size = int(font_size)
        self.base_font = self.load_font(self.base_font_size)

        self.qr_text = ""
        self.ai_text = ""
        self.pages: List[List[str]] = []
        self.page_index = 0
        self.last_payload = bytes()

        self.create_subscription(String, "/qr_display_text", self.qr_callback, 10)
        self.create_subscription(String, "/object_recognition_result", self.ai_callback, 10)
        self.bitmap_pub = self.create_publisher(UInt8MultiArray, OLED_BITMAP_TOPIC, 10)
        self.page_timer = self.create_timer(PAGE_PERIOD_SEC, self.page_timer_callback)
        self.refresh_timer = self.create_timer(REFRESH_PERIOD_SEC, self.refresh_timer_callback)

        self.rebuild_pages()
        self.publish_bitmap(force=True)
        self.get_logger().info(f"Publishing OLED bitmap to {OLED_BITMAP_TOPIC}")

    def resolve_font_path(self, font_path: str) -> str:
        candidates = [
            font_path,
            os.environ.get("OLED_FONT_PATH", ""),
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/simhei.ttf",
        ]
        return first_existing(candidates)

    def load_font(self, font_size: int):
        if self.font_path:
            if font_size == self.base_font_size:
                self.get_logger().info(f"Using OLED font: {self.font_path}")
            return ImageFont.truetype(self.font_path, font_size)
        self.get_logger().warn("No CJK font found, falling back to default font")
        return ImageFont.load_default()

    def qr_callback(self, msg: String) -> None:
        self.qr_text = self.format_qr_text(msg.data)
        self.rebuild_pages()
        self.publish_bitmap(force=True)

    def ai_callback(self, msg: String) -> None:
        self.ai_text = " ".join(msg.data.split())
        self.rebuild_pages()
        self.publish_bitmap(force=True)

    def page_timer_callback(self) -> None:
        if len(self.pages) <= 1:
            return
        self.page_index = (self.page_index + 1) % len(self.pages)
        self.publish_bitmap(force=True)

    def refresh_timer_callback(self) -> None:
        self.publish_bitmap(force=True)

    def rebuild_pages(self) -> None:
        qr_pages = self.build_pages_for_text(self.qr_text) if self.qr_text else []
        ai_pages = self.build_pages_for_text(self.ai_text) if self.ai_text else []
        pages: List[List[str]] = []

        if qr_pages and ai_pages:
            qr_count = len(qr_pages)
            for index, ai_page in enumerate(ai_pages):
                pages.append(qr_pages[index % qr_count])
                pages.append(ai_page)
        elif qr_pages:
            pages.extend(qr_pages)
        elif ai_pages:
            pages.extend(ai_pages)

        if not pages:
            pages = [[]]
        self.pages = pages
        self.page_index = 0

    def publish_bitmap(self, force: bool = False) -> None:
        payload = self.render_payload()
        if not force and payload == self.last_payload:
            return

        msg = UInt8MultiArray()
        msg.data = list(payload)
        self.bitmap_pub.publish(msg)
        self.last_payload = payload

    def render_payload(self) -> bytes:
        image = Image.new("1", (OLED_WIDTH, CONTENT_HEIGHT), 0)
        draw = ImageDraw.Draw(image)

        page_lines = self.pages[self.page_index] if self.pages else []
        font = self.base_font
        line_height = self.measure_line_height(font)
        total_height = len(page_lines) * line_height + max(0, len(page_lines) - 1) * LINE_GAP
        y = max(0, (CONTENT_HEIGHT - total_height) // 2 - VERTICAL_SHIFT)

        for line in page_lines:
            text_width = int(draw.textlength(line, font=font))
            x = max(0, (OLED_WIDTH - text_width) // 2)
            draw.text((x, y), line, font=font, fill=1)
            y += line_height + LINE_GAP

        image = image.transpose(Image.FLIP_LEFT_RIGHT)

        pixels = image.load()
        payload = bytearray(OLED_WIDTH * ROW_COUNT)
        for page in range(ROW_COUNT):
            for x in range(OLED_WIDTH):
                value = 0
                for bit in range(8):
                    if pixels[x, page * 8 + bit]:
                        value |= (1 << bit)
                payload[page * OLED_WIDTH + x] = value
        return bytes(payload)

    def build_pages_for_text(self, text: str) -> List[List[str]]:
        lines = self.wrap_text(text)
        if not lines:
            return [[]]

        pages: List[List[str]] = []
        for i in range(0, len(lines), MAX_LINES_PER_PAGE):
            pages.append(lines[i:i + MAX_LINES_PER_PAGE])
        return pages

    def wrap_text(self, text: str) -> List[str]:
        if not text:
            return []

        draw = ImageDraw.Draw(Image.new("1", (OLED_WIDTH, CONTENT_HEIGHT), 0))
        lines: List[str] = []
        current = ""
        for char in text:
            trial = current + char
            if draw.textlength(trial, font=self.base_font) <= OLED_WIDTH:
                current = trial
            else:
                if current:
                    lines.append(current)
                current = char
        if current:
            lines.append(current)
        return lines

    def measure_line_height(self, font) -> int:
        bbox = font.getbbox("测A8")
        return bbox[3] - bbox[1]

    def format_qr_text(self, raw_text: str) -> str:
        parts = raw_text.strip().split()
        if not parts:
            return ""

        number = parts[0]
        direction = ""
        if len(parts) > 1:
            token = parts[1].upper()
            if token == "CW":
                direction = "顺时针"
            elif token == "CCW":
                direction = "逆时针"
            else:
                direction = parts[1]

        return f"{number} {direction}".strip()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OledTextRenderer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
