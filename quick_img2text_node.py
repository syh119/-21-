#!/usr/bin/env python3
import base64
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

try:
    from volcenginesdkarkruntime import Ark
except ImportError:
    Ark = None


DEFAULT_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
DEFAULT_PROMPT = "Describe this image briefly. Focus on the scene, objects, text, and QR codes if present."


class QuickImg2TextNode(Node):
    def __init__(self):
        super().__init__("quick_img2text_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_topic", "/image"),
                ("api_key", os.environ.get("ARK_API_KEY", "")),
                ("model_id", os.environ.get("ARK_MODEL_ID", "ep-20260531153550-mcvz2")),
                ("base_url", os.environ.get("ARK_BASE_URL", DEFAULT_BASE_URL)),
                ("prompt", DEFAULT_PROMPT),
                ("capture_timeout_sec", 3.0),
                ("api_timeout_sec", 8.0),
            ],
        )

        self.image_topic = self.get_parameter("image_topic").value
        self.api_key = self.get_parameter("api_key").value
        self.model_id = self.get_parameter("model_id").value
        self.base_url = self.get_parameter("base_url").value
        self.prompt = self.get_parameter("prompt").value
        self.capture_timeout_sec = float(self.get_parameter("capture_timeout_sec").value)
        self.api_timeout_sec = float(self.get_parameter("api_timeout_sec").value)

        self.client = None
        self.subscription = None
        self.timeout_timer = None
        self.finished = False
        self.ready = False

        if Ark is None:
            self.get_logger().error("volcenginesdkarkruntime is not installed")
            return

        if not self.api_key or not self.model_id:
            self.get_logger().error("ARK_API_KEY or ARK_MODEL_ID is missing")
            return

        try:
            self.client = Ark(
                api_key=self.api_key,
                base_url=self.base_url,
                timeout=self.api_timeout_sec,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to create Ark client: {exc}")
            return

        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.timeout_timer = self.create_timer(self.capture_timeout_sec, self._on_timeout)
        self.ready = True
        self.get_logger().info(f"Waiting for one frame on {self.image_topic}")

    def _on_timeout(self):
        if self.finished:
            return
        self.get_logger().error("No image received before timeout")
        self._finish()

    def _image_callback(self, msg):
        if self.finished:
            return

        self.finished = True
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        try:
            img_base64 = base64.b64encode(msg.data).decode("utf-8")
            image_url = f"data:image/jpeg;base64,{img_base64}"
            response = self.client.responses.create(
                model=self.model_id,
                input=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "input_image", "image_url": image_url},
                            {"type": "input_text", "text": self.prompt},
                        ],
                    }
                ],
                timeout=self.api_timeout_sec,
            )
            result_text = self._extract_text(response)
            if result_text:
                self.get_logger().info(result_text)
            else:
                self.get_logger().warn("Empty response from image-to-text API")
        except Exception as exc:
            self.get_logger().error(f"Image-to-text request failed: {exc}")
        finally:
            self._finish()

    def _extract_text(self, response):
        output = getattr(response, "output", None) or []
        for item in output:
            content = getattr(item, "content", None) or []
            for block in content:
                text = getattr(block, "text", None)
                if text:
                    return str(text).strip()

        choices = getattr(response, "choices", None) or []
        if choices:
            message = getattr(choices[0], "message", None)
            if message is not None:
                text = getattr(message, "content", None)
                if text:
                    return str(text).strip()

        output_text = getattr(response, "output_text", None)
        if output_text:
            return str(output_text).strip()

        return ""

    def _finish(self):
        if self.finished is False:
            self.finished = True
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = QuickImg2TextNode()
    try:
        if node.ready:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
