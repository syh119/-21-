#!/usr/bin/env python3
import argparse
import base64
import os
import sys
import time

import cv2

try:
    from volcenginesdkarkruntime import Ark
except ImportError:
    Ark = None


DEFAULT_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
# 直接在这里填也可以；命令行参数会覆盖这里的默认值
ARK_API_KEY = os.environ.get("ARK_API_KEY", "")
ARK_MODEL_ID = "ep-20260531155206-mhpp6"
ARK_BASE_URL = DEFAULT_BASE_URL
DEFAULT_PROMPT = "请用中文简要描述这张图片，重点说明场景、物体、文字和二维码。"


def build_parser():
    parser = argparse.ArgumentParser(description="Capture one frame from camera and call image-to-text API.")
    parser.add_argument("--device", default="/dev/video8", help="Camera device path or camera index.")
    parser.add_argument(
        "--backend",
        default="auto",
        choices=["auto", "any", "v4l2", "gstreamer"],
        help="OpenCV camera backend.",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument("--fps", type=int, default=30, help="Capture fps.")
    parser.add_argument("--capture-timeout", type=float, default=1.0, help="Seconds to wait for one valid frame.")
    parser.add_argument("--warmup-time", type=float, default=0.2, help="Seconds to allow camera to warm up.")
    parser.add_argument("--api-timeout", type=float, default=8.0, help="Seconds to wait for API response.")
    parser.add_argument("--jpeg-quality", type=int, default=85, help="JPEG quality before upload.")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT, help="Prompt text sent with the image.")
    parser.add_argument("--api-key", default=os.environ.get("ARK_API_KEY", ARK_API_KEY), help="ARK API key.")
    parser.add_argument("--model-id", default=os.environ.get("ARK_MODEL_ID", ARK_MODEL_ID), help="ARK model id.")
    parser.add_argument("--base-url", default=os.environ.get("ARK_BASE_URL", ARK_BASE_URL), help="ARK base url.")
    parser.add_argument("--save-frame", default="", help="Optional path to save the captured frame.")
    return parser


def parse_device(device_arg):
    if isinstance(device_arg, int):
        return device_arg
    if isinstance(device_arg, str) and device_arg.isdigit():
        return int(device_arg)
    return device_arg


def pick_backend(args, device):
    if args.backend == "any":
        return cv2.CAP_ANY
    if args.backend == "v4l2":
        return cv2.CAP_V4L2
    if args.backend == "gstreamer":
        return cv2.CAP_GSTREAMER
    if isinstance(device, int):
        return cv2.CAP_ANY
    return cv2.CAP_V4L2


def capture_one_frame(args):
    device = parse_device(args.device)
    backend = pick_backend(args, device)
    cap = cv2.VideoCapture(device, backend)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera: {args.device} (backend={args.backend}). "
            f"Try --backend v4l2 and check /dev/video*."
        )

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        deadline = time.time() + max(args.capture_timeout, 0.1)
        warmup_deadline = time.time() + max(args.warmup_time, 0.0)
        frame = None

        while time.time() < deadline:
            ok, current = cap.read()
            if ok and current is not None and current.size > 0:
                frame = current
                if time.time() >= warmup_deadline:
                    break

        if frame is None:
            raise RuntimeError("No valid frame captured before timeout")

        return frame
    finally:
        cap.release()


def encode_frame(frame, jpeg_quality):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
    ok, buffer = cv2.imencode(".jpg", frame, encode_param)
    if not ok:
        raise RuntimeError("Failed to encode frame to JPEG")
    return buffer.tobytes()


def call_img2text_api(args, jpeg_bytes):
    if Ark is None:
        raise RuntimeError("volcenginesdkarkruntime is not installed")
    if not args.api_key:
        raise RuntimeError("ARK_API_KEY is missing")
    if not args.model_id:
        raise RuntimeError("ARK_MODEL_ID is missing")

    client = Ark(
        api_key=args.api_key,
        base_url=args.base_url,
        timeout=args.api_timeout,
    )

    image_base64 = base64.b64encode(jpeg_bytes).decode("utf-8")
    image_url = f"data:image/jpeg;base64,{image_base64}"

    response = client.responses.create(
        model=args.model_id,
        input=[
            {
                "role": "user",
                "content": [
                    {"type": "input_image", "image_url": image_url},
                    {"type": "input_text", "text": args.prompt},
                ],
            }
        ],
        timeout=args.api_timeout,
    )
    return extract_text(response)


def extract_text(response):
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


def maybe_save_frame(path, frame):
    if not path:
        return
    if not cv2.imwrite(path, frame):
        raise RuntimeError(f"Failed to save frame to: {path}")


def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        start = time.time()
        frame = capture_one_frame(args)
        capture_end = time.time()

        maybe_save_frame(args.save_frame, frame)

        jpeg_bytes = encode_frame(frame, args.jpeg_quality)
        result = call_img2text_api(args, jpeg_bytes)
        end = time.time()

        print(f"capture_time={capture_end - start:.3f}s")
        print(f"api_time={end - capture_end:.3f}s")
        print(f"total_time={end - start:.3f}s")
        print("result:")
        print(result or "<empty>")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
