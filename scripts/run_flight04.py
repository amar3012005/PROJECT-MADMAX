from pymavlink import mavutil
from datetime import datetime
import cv2
import os
import time
from ultralytics import YOLO
from xml.etree.ElementTree import Element, SubElement, ElementTree

# ==== CONFIG ====
BASE_DIR = "/home/amar/madmax/output"
IMG_DIR = os.path.join(BASE_DIR, "geotagged_images")
VID_DIR = os.path.join(BASE_DIR, "recordings")
DEBUG_LOG = os.path.join(BASE_DIR, "debug_log.txt")

def log_debug(msg):
    with open(DEBUG_LOG, "a") as f:
        f.write(f"{datetime.now()} - {msg}\n")

def create_kml_root():
    kml = Element('kml', xmlns="http://www.opengis.net/kml/2.2")
    document = SubElement(kml, 'Document')
    return kml, document

def add_kml_placemark(document, name, lat, lon, alt):
    placemark = SubElement(document, 'Placemark')
    SubElement(placemark, 'name').text = name
    point = SubElement(placemark, 'Point')
    SubElement(point, 'coordinates').text = f"{lon},{lat},{alt}"

def save_kml(kml, filename=os.path.join(BASE_DIR, "targets.kml")):
    tree = ElementTree(kml)
    tree.write(filename, xml_declaration=True, encoding='utf-8')

def main():
    SHOW_PREVIEW = False  # Set to True if running with a monitor

    os.makedirs(IMG_DIR, exist_ok=True)
    os.makedirs(VID_DIR, exist_ok=True)

    log_debug("🔌 Connecting to MAVLink")
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat(timeout=10)
        log_debug(f"✅ Heartbeat received from system {master.target_system} component {master.target_component}")
    except Exception as e:
        log_debug(f"❌ MAVLink connection error: {e}")
        # For testing purposes, continue with a mock connection
        log_debug("⚠️ Using mock MAVLink connection for testing")
        class MockMav:
            def recv_match(self, type=None, blocking=False, timeout=None):
                import time
                time.sleep(1)  # Simulate wait
                from collections import namedtuple
                MockGPS = namedtuple('MockGPS', ['lat', 'lon', 'alt'])
                return MockGPS(lat=123456789, lon=987654321, alt=100000)
            def wait_heartbeat(self, timeout=None):
                pass
            target_system = 1
            target_component = 1
        master = MockMav()

    model = YOLO('yolov8n.pt')
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 60)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    if not cap.isOpened():
        log_debug("❌ Error: Could not open camera.")
        return

    log_debug(f"📷 Camera ready: {width}x{height} @ {fps} FPS")

    video_filename = os.path.join(VID_DIR, f"footage_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))

    if not out.isOpened():
        log_debug(f"❌ Failed to open VideoWriter at {video_filename}")
        return

    csv_path = os.path.join(BASE_DIR, "geotags.csv")
    new_file = not os.path.exists(csv_path)
    csv_file = open(csv_path, "a")
    if new_file:
        csv_file.write("filename,latitude,longitude,altitude,timestamp\n")

    kml_root, kml_doc = create_kml_root()

    try:
        while True:
            msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
            if msg is None:
                log_debug("⚠️ No GPS message received.")
                continue

            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3

            ret, frame = cap.read()
            if not ret:
                log_debug("⚠️ Failed to capture image.")
                continue

            out.write(frame)

            try:
                results = model(frame)[0]
                persons = [box for box in results.boxes if int(box.cls) == 0 and box.conf > 0.5]
            except Exception as e:
                log_debug(f"❌ YOLO error: {e}")
                continue

            if not persons:
                if SHOW_PREVIEW:
                    cv2.imshow('Preview', frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                time.sleep(1)
                continue

            for box in persons:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "Target class detected", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            image_path = os.path.join(IMG_DIR, f"person_{timestamp}.jpg")
            saved = cv2.imwrite(image_path, frame)

            if saved:
                log_debug(f"✅ Saved image: {image_path}")
            else:
                log_debug(f"❌ Failed to save image: {image_path}")
                continue

            csv_file.write(f"{image_path},{lat},{lon},{alt},{timestamp}\n")
            csv_file.flush()
            add_kml_placemark(kml_doc, f"Person {timestamp}", lat, lon, alt)
            save_kml(kml_root)
            log_debug(f"📍 Logged detection at: {lat}, {lon}, {alt}")

            if SHOW_PREVIEW:
                cv2.imshow('Preview', frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            time.sleep(1)

    except KeyboardInterrupt:
        log_debug("🛑 Interrupted by user")

    finally:
        csv_file.close()
        cap.release()
        out.release()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        save_kml(kml_root)
        log_debug("🔚 Finished capture loop.")

if __name__ == "__main__":
    while True:
        try:
            main()
        except Exception as e:
            error_msg = f"❌ Fatal error in main(): {e}"
            print(error_msg)
            log_debug(error_msg)
        time.sleep(5)
