#!/usr/bin/env python3
# ============================================================
# LAPTOP / WSL2 - ESP32-CAM SCREENSHOT -> EDGE IMPULSE -> MAIN
#
# Flow:
#   1. Ask MAIN for the camera IP.
#   2. GET one JPEG from CAM /capture.
#   3. Run the JPEG through model.eim using Edge Impulse Linux SDK.
#   4. Send label + confidence + Pyramid/Cube/Ball scores to MAIN.
#   5. MAIN broadcasts the result to the existing dashboard WebSocket.
#
# No webcam is used.
# No ESP-NOW is used.
# No live MJPEG stream is used.
# ============================================================

import argparse
import sys
import time
from urllib.parse import urlencode

import cv2
import numpy as np
import requests
from edge_impulse_linux.image import ImageImpulseRunner

MAIN_IP = "192.168.4.1"
MAIN_STATUS_URL = f"http://{MAIN_IP}/camera/status"

CAPTURE_TIMEOUT = 2.0
MAIN_TIMEOUT = 2.0
LOOP_DELAY = 0.20

CONFIDENCE_THRESHOLD = 0.30


def get_camera_info():
    r = requests.get(MAIN_STATUS_URL, timeout=MAIN_TIMEOUT)
    r.raise_for_status()
    data = r.json()
    ip = data.get("ip", "")
    camera = data.get("camera", "OFF")
    link = bool(data.get("link", False))
    return camera, link, ip


def capture_image(camera_ip):
    # Cache-busting prevents any intermediary/browser cache from reusing
    # an old JPEG. The camera itself also sends no-cache headers.
    base_url = f"http://{camera_ip}/capture"
    last_error = None

    for attempt in range(3):
        try:
            url = f"{base_url}?refresh={time.monotonic_ns()}"
            r = requests.get(
                url,
                timeout=CAPTURE_TIMEOUT,
                headers={"Cache-Control": "no-cache", "Pragma": "no-cache"},
            )
            r.raise_for_status()

            arr = np.frombuffer(r.content, dtype=np.uint8)
            frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                raise RuntimeError("OpenCV could not decode the ESP32-CAM JPEG")

            return frame_bgr
        except Exception as exc:
            last_error = exc
            if attempt < 2:
                time.sleep(0.03)

    raise RuntimeError(f"Screenshot failed after 3 attempts: {last_error}")


def normalize_scores(labels, scores):
    result = {str(label): float(score) for label, score in zip(labels, scores)}

    # Support the exact class names used by the user's Edge Impulse model.
    def score_for(name):
        if name in result:
            return result[name]
        lower = name.lower()
        for k, v in result.items():
            if k.lower() == lower:
                return v
        return 0.0

    return result, score_for("Pyramid"), score_for("Cube"), score_for("Ball")


def classify_frame(runner, frame_rgb, labels):
    # Edge Impulse handles resizing/cropping according to the model input.
    features, _cropped = runner.get_features_from_image(frame_rgb)
    res = runner.classify(features)

    result = res.get("result", {})

    # Standard image classification model.
    if "classification" in result:
        raw = result["classification"]
        scores = [float(raw.get(label, 0.0)) for label in labels]
        score_map, p, c, b = normalize_scores(labels, scores)

        if score_map:
            best_label = max(score_map, key=score_map.get)
            best_score = score_map[best_label]
        else:
            best_label = "NONE"
            best_score = 0.0

        if best_score < CONFIDENCE_THRESHOLD:
            best_label = "NONE"

        return best_label, best_score, p, c, b, res

    # Object detection model.
    if "bounding_boxes" in result:
        best_by_label = {}
        for bb in result["bounding_boxes"]:
            label = str(bb.get("label", ""))
            value = float(bb.get("value", 0.0))
            if value > best_by_label.get(label, 0.0):
                best_by_label[label] = value

        def get_bb_score(name):
            for label, value in best_by_label.items():
                if label.lower() == name.lower():
                    return value
            return 0.0

        p = get_bb_score("Pyramid")
        c = get_bb_score("Cube")
        b = get_bb_score("Ball")

        if best_by_label:
            best_label = max(best_by_label, key=best_by_label.get)
            best_score = best_by_label[best_label]
        else:
            best_label = "NONE"
            best_score = 0.0

        if best_score < CONFIDENCE_THRESHOLD:
            best_label = "NONE"

        return best_label, best_score, p, c, b, res

    return "NONE", 0.0, 0.0, 0.0, 0.0, res


def send_detection(label, confidence, p, c, b):
    params = {
        "label": label,
        "confidence": f"{confidence:.4f}",
        "p": f"{p:.4f}",
        "c": f"{c:.4f}",
        "b": f"{b:.4f}",
    }

    url = f"http://{MAIN_IP}/detection?{urlencode(params)}"
    r = requests.get(url, timeout=MAIN_TIMEOUT)
    r.raise_for_status()


def main():
    global CONFIDENCE_THRESHOLD, LOOP_DELAY

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="model.eim", help="Path to Linux x86_64 Edge Impulse .eim")
    parser.add_argument("--interval", type=float, default=LOOP_DELAY, help="Delay between inference cycles")
    parser.add_argument("--threshold", type=float, default=CONFIDENCE_THRESHOLD, help="Detection confidence threshold")
    args = parser.parse_args()

    CONFIDENCE_THRESHOLD = max(0.0, min(1.0, args.threshold))
    LOOP_DELAY = max(0.01, args.interval)

    print("==============================================")
    print("ESP32-CAM -> LAPTOP -> EDGE IMPULSE -> DASH")
    print("==============================================")
    print(f"MAIN:  {MAIN_STATUS_URL}")
    print(f"MODEL: {args.model}")
    print(f"THRESHOLD: {CONFIDENCE_THRESHOLD:.2f}")

    runner = None

    try:
        runner = ImageImpulseRunner(args.model)
        model_info = runner.init()

        model_parameters = model_info.get("model_parameters", {})
        labels = model_parameters.get("labels", [])
        image_width = int(model_parameters.get("image_input_width", 0) or 0)
        image_height = int(model_parameters.get("image_input_height", 0) or 0)
        image_channels = int(model_parameters.get("image_channel_count", 3) or 3)
        resize_mode = model_parameters.get("image_resize_mode", "unknown")

        if image_channels == 1:
            model_color_mode = "Grayscale"
        elif image_channels == 3:
            model_color_mode = "RGB"
        else:
            model_color_mode = f"{image_channels}-channel"

        print("MODEL LOADED")
        print("PROJECT:", model_info["project"].get("name", "unknown"))
        print("LABELS:", ", ".join(labels))
        print(f"MODEL INPUT: {image_width}x{image_height}x{image_channels} ({model_color_mode})")
        print(f"RESIZE MODE: {resize_mode} (Largest Axis is handled by Edge Impulse SDK)")

        last_camera_ip = ""
        last_state = None

        while True:
            try:
                camera_state, link, camera_ip = get_camera_info()

                if camera_ip != last_camera_ip or camera_state != last_state:
                    print(f"CAMERA: {camera_state} | LINK: {link} | IP: {camera_ip}")
                    last_camera_ip = camera_ip
                    last_state = camera_state

                if camera_state != "ON" or not link or not camera_ip:
                    time.sleep(0.5)
                    continue

                frame_bgr = capture_image(camera_ip)
                frame_rgb = prepare_image_for_model(frame_bgr, image_channels)

                label, confidence, p, c, b, res = classify_frame(
                    runner,
                    frame_rgb,
                    labels,
                )

                send_detection(label, confidence, p, c, b)

                timing = res.get("timing", {})
                total_ms = int(timing.get("dsp", 0) + timing.get("classification", 0))

                print(
                    f"DETECTION: {label:8s} "
                    f"conf={confidence*100:5.1f}% "
                    f"P={p*100:5.1f}% "
                    f"C={c*100:5.1f}% "
                    f"B={b*100:5.1f}% "
                    f"time={total_ms}ms"
                )

                time.sleep(LOOP_DELAY)

            except requests.RequestException as e:
                print("NETWORK:", e)
                time.sleep(0.5)
            except Exception as e:
                print("INFERENCE/CAPTURE:", e)
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if runner is not None:
            runner.stop()


if __name__ == "__main__":
    main()
