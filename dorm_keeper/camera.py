# import os
# os.environ['QT_QPA_PLATFORM'] = 'xcb'

# import cv2
# import numpy as np
# import requests
# import time
# import zxingcpp


# # Configuration
# CAMERA_INDEX = 4  # External USB camera (Generic USB Camera - TM Technology, Inc. 10bb:2b08)
# # Note: On this system, uvcvideo module needs to be loaded first: `sudo modprobe uvcvideo`

# API_CACHE = {}
# API_CACHE_TTL = 60.0


# def ean_checksum_ok(ean):
#     """Validate EAN-8/EAN-13 checksum to avoid bogus reads hitting the API."""
#     if not ean.isdigit() or len(ean) not in (8, 13):
#         return False
#     digits = [int(d) for d in ean]
#     body, check = digits[:-1], digits[-1]
#     weights = [3, 1] * (len(body) // 2)
#     total = sum(w * d for w, d in zip(weights, body))
#     return (10 - (total % 10)) % 10 == check


# def check_kaucja(ean):
#     now = time.time()
#     cached = API_CACHE.get(ean)
#     if cached and now - cached[0] < API_CACHE_TTL:
#         return cached[1]

#     url = f"https://api.kaucja.pl/buf/pos/product/{ean}"
#     try:
#         r = requests.get(url, timeout=5)
#         if r.status_code != 200:
#             return None
#         data = r.json()
#         dep = data.get("deposit") or {}
#         result = {
#             "ean": data.get("ean"),
#             "nazwa": data.get("publishedName") or data.get("name"),
#             "kaucyjny": data.get("deposit") is not None,
#             "kaucja": dep.get("amount"),
#             "waluta": dep.get("currency"),
#             "status": data.get("collectionStatusId"),
#         }
#         API_CACHE[ean] = (now, result)
#         return result
#     except (requests.RequestException, ValueError):
#         return None


# def zxing_read(img):
#     """Run zxing-cpp on a grayscale/numpy image. Returns (text, rect) or (None, None)."""
#     results = zxingcpp.read_barcodes(img)
#     for r in results:
#         text = r.text
#         if text and text.isdigit() and len(text) in (8, 13):
#             pos = r.position
#             # zxingcpp gives 4 corner points
#             xs = [p.x for p in (pos.top_left, pos.top_right,
#                                 pos.bottom_left, pos.bottom_right)]
#             ys = [p.y for p in (pos.top_left, pos.top_right,
#                                 pos.bottom_left, pos.bottom_right)]
#             left, top = min(xs), min(ys)
#             width, height = max(xs) - left, max(ys) - top
#             return text, (left, top, width, height)
#     return None, None


# def decode_frame_multi(frame):
#     """
#     zxing-cpp already does binarization, downscaling and multi-angle
#     scan internally, so the pipeline is simpler than the pyzbar one.
#     Multiple preprocessing variants help with different surface types:
#     - Plastic/glossy items benefit from upscaling and blurring
#     - Metal cans benefit from CLAHE and thresholding
#     - Curved surfaces benefit from band slicing
#     """
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # 1) Plain grayscale — zxing handles most cases straight away.
#     text, rect = zxing_read(gray)
#     if text:
#         return text, rect

#     # 2) Slightly upscaled + blurred — helps when the barcode is small
#     #    in frame or plastic wrap creates fine glare speckle.
#     big = cv2.resize(gray, None, fx=1.5, fy=1.5,
#                      interpolation=cv2.INTER_CUBIC)
#     big = cv2.GaussianBlur(big, (3, 3), 0)
#     text, rect = zxing_read(big)
#     if text:
#         if rect:
#             l, t, w, h = rect
#             rect = (int(l / 1.5), int(t / 1.5),
#                     int(w / 1.5), int(h / 1.5))
#         return text, rect

#     # 3) CLAHE enhancement — improves contrast on metal cans with glare.
#     clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
#     enhanced = clahe.apply(gray)
#     text, rect = zxing_read(enhanced)
#     if text:
#         return text, rect

#     # 4) Adaptive thresholding — helps with low-contrast barcodes on metal.
#     thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
#                                      cv2.THRESH_BINARY, 11, 2)
#     text, rect = zxing_read(thresh)
#     if text:
#         return text, rect

#     # 5) Horizontal band slices — still useful for strongly curved cans.
#     h, w = gray.shape
#     band = 80
#     step = 50
#     for y in range(0, max(1, h - band), step):
#         strip = gray[y:y + band, :]
#         text, rect = zxing_read(strip)
#         if text:
#             if rect:
#                 l, t, rw, rh = rect
#                 rect = (l, y + t, rw, rh)
#             return text, rect

#     # 6) CLAHE + band slices — for curved metal cans with glare.
#     for y in range(0, max(1, h - band), step):
#         strip = enhanced[y:y + band, :]
#         text, rect = zxing_read(strip)
#         if text:
#             if rect:
#                 l, t, rw, rh = rect
#                 rect = (l, y + t, rw, rh)
#             return text, rect

#     return None, None


# def camera_qr_reader():
#     cap = cv2.VideoCapture(CAMERA_INDEX)
#     if not cap.isOpened():
#         print(f"Error: Could not open camera {CAMERA_INDEX}.")
#         return

#     # Prefer a native sensor mode; forcing 1080p on many webcams yields
#     # an upscaled blurry stream. 720p is usually the real sensor mode.
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#     try:
#         cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#         cap.set(cv2.CAP_PROP_FOCUS, 20)  # ~30-40cm; tune for your rig
#     except Exception:
#         pass
#     time.sleep(1.0)

#     print("Camera opened. Hold the item ~15-30 cm away, barcode facing the lens.")
#     print("Angle it slightly so glare is NOT on the barcode.")
#     print("Press 'q' to quit.")

#     last_ean = None

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Error: Could not read frame.")
#             break

#         ean, rect = decode_frame_multi(frame)

#         if ean and ean_checksum_ok(ean):
#             if ean != last_ean:
#                 last_ean = ean
#                 print(f"\nFound EAN: {ean}")
#                 result = check_kaucja(ean)
#                 if result:
#                     print(f"  Product: {result.get('nazwa')}")
#                     print(f"  EAN: {result.get('ean')}")
#                     print(f"  Deposit: {result.get('kaucja')} {result.get('waluta')}")
#                     print(f"  Deposit applicable: {result.get('kaucyjny')}")
#                     print(f"  Status: {result.get('status')}")
#                 else:
#                     print(f"  No deposit information found for EAN: {ean}")

#             if rect:
#                 left, top, width, height = rect
#                 cv2.rectangle(frame, (left, top),
#                               (left + width, top + height),
#                               (0, 255, 0), 2)
#                 cv2.putText(frame, ean, (left, max(20, top - 10)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#         cv2.imshow('Barcode Reader - Press q to quit', frame)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()


# if __name__ == "__main__":
#     camera_qr_reader()

import os
import sys
import time
import cv2
import numpy as np
import requests
import zxingcpp

os.environ['QT_QPA_PLATFORM'] = 'xcb'

# Configuration
# Default target device for Linux, fallback indices for Windows/other OS
LINUX_DEVICE = "/dev/video4"
FALLBACK_INDICES = [4, 0, 1, 2]

API_CACHE = {}
API_CACHE_TTL = 60.0


def open_camera():
    """Dynamically attempts to open the camera across Linux and Windows platforms."""
    is_windows = sys.platform.startswith("win")
    
    # 1. On Linux, try string device path first (/dev/video4)
    if not is_windows and os.path.exists(LINUX_DEVICE):
        cap = cv2.VideoCapture(LINUX_DEVICE, cv2.CAP_V4L2)
        if cap.isOpened():
            print(f"Successfully opened camera at device path: {LINUX_DEVICE}")
            return cap

    # 2. Try integer indices with OS-appropriate backends
    for idx in FALLBACK_INDICES:
        if is_windows:
            # Try DirectShow first on Windows, then default
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap = cv2.VideoCapture(idx)
        else:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(idx)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"Successfully opened camera index: {idx}")
                return cap
            cap.release()

    return None


def ean_checksum_ok(ean):
    """Validate EAN-8/EAN-13 checksum to avoid bogus reads hitting the API."""
    if not ean.isdigit() or len(ean) not in (8, 13):
        return False
    digits = [int(d) for d in ean]
    body, check = digits[:-1], digits[-1]
    weights = [3, 1] * (len(body) // 2) if len(ean) == 8 else [1, 3] * 6
    total = sum(w * d for w, d in zip(weights, body))
    return (10 - (total % 10)) % 10 == check


def check_kaucja(ean):
    now = time.time()
    cached = API_CACHE.get(ean)
    if cached and now - cached[0] < API_CACHE_TTL:
        return cached[1]

    url = f"https://api.kaucja.pl/buf/pos/product/{ean}"
    try:
        r = requests.get(url, timeout=5)
        if r.status_code != 200:
            return None
        data = r.json()
        dep = data.get("deposit") or {}
        result = {
            "ean": data.get("ean"),
            "nazwa": data.get("publishedName") or data.get("name"),
            "kaucyjny": data.get("deposit") is not None,
            "kaucja": dep.get("amount"),
            "waluta": dep.get("currency"),
            "status": data.get("collectionStatusId"),
        }
        API_CACHE[ean] = (now, result)
        return result
    except (requests.RequestException, ValueError):
        return None


def zxing_read(img):
    """Run zxing-cpp on a grayscale/numpy image. Returns (text, rect) or (None, None)."""
    results = zxingcpp.read_barcodes(img)
    for r in results:
        text = r.text
        if text and text.isdigit() and len(text) in (8, 13):
            pos = r.position
            xs = [p.x for p in (pos.top_left, pos.top_right, pos.bottom_left, pos.bottom_right)]
            ys = [p.y for p in (pos.top_left, pos.top_right, pos.bottom_left, pos.bottom_right)]
            left, top = min(xs), min(ys)
            width, height = max(xs) - left, max(ys) - top
            return text, (left, top, width, height)
    return None, None


def decode_frame_multi(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1) Plain grayscale
    text, rect = zxing_read(gray)
    if text:
        return text, rect

    # 2) Upscaled + blurred
    big = cv2.resize(gray, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
    big = cv2.GaussianBlur(big, (3, 3), 0)
    text, rect = zxing_read(big)
    if text:
        if rect:
            l, t, w, h = rect
            rect = (int(l / 1.5), int(t / 1.5), int(w / 1.5), int(h / 1.5))
        return text, rect

    # 3) CLAHE enhancement
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    text, rect = zxing_read(enhanced)
    if text:
        return text, rect

    # 4) Adaptive thresholding
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    text, rect = zxing_read(thresh)
    if text:
        return text, rect

    # 5) Horizontal band slices
    h, w = gray.shape
    band = 80
    step = 50
    for y in range(0, max(1, h - band), step):
        strip = gray[y:y + band, :]
        text, rect = zxing_read(strip)
        if text:
            if rect:
                l, t, rw, rh = rect
                rect = (l, y + t, rw, rh)
            return text, rect

    # 6) CLAHE + band slices
    for y in range(0, max(1, h - band), step):
        strip = enhanced[y:y + band, :]
        text, rect = zxing_read(strip)
        if text:
            if rect:
                l, t, rw, rh = rect
                rect = (l, y + t, rw, rh)
            return text, rect

    return None, None


def camera_qr_reader():
    cap = open_camera()
    if cap is None or not cap.isOpened():
        print("Error: Could not open any available camera device.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    try:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, 20)
    except Exception:
        pass
    time.sleep(1.0)

    print("Camera opened. Hold the item ~15-30 cm away, barcode facing the lens.")
    print("Press 'q' to quit.")

    last_ean = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        ean, rect = decode_frame_multi(frame)

        if ean and ean_checksum_ok(ean):
            if ean != last_ean:
                last_ean = ean
                print(f"\nFound EAN: {ean}")
                result = check_kaucja(ean)
                if result:
                    print(f"  Product: {result.get('nazwa')}")
                    print(f"  EAN: {result.get('ean')}")
                    print(f"  Deposit: {result.get('kaucja')} {result.get('waluta')}")
                    print(f"  Deposit applicable: {result.get('kaucyjny')}")
                    print(f"  Status: {result.get('status')}")
                else:
                    print(f"  No deposit information found for EAN: {ean}")

            if rect:
                left, top, width, height = rect
                cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 255, 0), 2)
                cv2.putText(frame, ean, (left, max(20, top - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Barcode Reader - Press q to quit', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_qr_reader()
