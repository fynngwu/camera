import os
import cv2
import threading

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp|max_delay;0|fflags;nobuffer|flags;low_delay"
)

url = "rtsp://100.103.8.66:8554/cam"

cap = cv2.VideoCapture(
    url,
    cv2.CAP_FFMPEG,
    [
        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
        cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000,
        cv2.CAP_PROP_N_THREADS, 1,
    ],
)

latest = {"frame": None}
lock = threading.Lock()
running = True

def reader():
    while running:
        ok = cap.grab()
        if not ok:
            continue
        ok, frame = cap.retrieve()
        if not ok:
            continue
        with lock:
            latest["frame"] = frame

th = threading.Thread(target=reader, daemon=True)
th.start()

while True:
    with lock:
        frame = None if latest["frame"] is None else latest["frame"].copy()
    if frame is None:
        continue

    cv2.imshow("live", frame)
    if cv2.waitKey(1) == 27:
        running = False
        break

cap.release()
cv2.destroyAllWindows()