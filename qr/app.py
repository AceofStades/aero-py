import cv2
from flask import Flask, Response
from qreader import QReader

app = Flask(__name__)
qreader = QReader()


def gen_frames():
    """
    Generator function that continuously captures frames from the camera,
    encodes them as JPEG, and yields them for the web stream.
    Also runs the QR code detection every 60 frames.
    """
    # Initialize the camera.
    # '0' is usually the default built-in camera or first USB camera.
    # On a Raspberry Pi, if using the Pi Camera Module, you might need to
    # configure it properly or use '0' depending on your OS/driver setup.
    camera = cv2.VideoCapture(0)

    # Optional: Set lower resolution to improve streaming performance over Wi-Fi
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    count = 0
    while True:
        success, frame = camera.read()
        if not success:
            # If we fail to grab a frame, break the loop
            break
        else:
            # Run QR detection every 60 frames (same as qr.py)
            if count % 60 == 0:
                text = qreader.detect_and_decode(frame)
                if text:
                    # Print to terminal output
                    print(text)

            count += 1

            # Encode the OpenCV frame into a JPEG format
            ret, buffer = cv2.imencode(".jpg", frame)

            # Convert the buffer to bytes
            frame_bytes = buffer.tobytes()

            # Yield the frame in the multipart format expected by web browsers for MJPEG streams
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
            )


@app.route("/")
def index():
    """
    Home page route that displays the video feed.
    """
    # A simple HTML page containing an image tag that points to the video_feed route
    return """
    <html>
        <head>
            <title>Raspberry Pi Video Stream</title>
            <style>
                body { text-align: center; font-family: Arial, sans-serif; background-color: #f4f4f4; padding-top: 50px; }
                h1 { color: #333; }
                img { border: 2px solid #333; border-radius: 5px; box-shadow: 0px 4px 8px rgba(0,0,0,0.2); }
            </style>
        </head>
        <body>
            <h1>AEROTHON QR Detection Stream</h1>
            <p>Live feed from Raspberry Pi camera</p>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():
    """
    Video streaming route. Put this in the src attribute of an img tag.
    """
    # Returns the generated frames as a continuous multipart response
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    # host='0.0.0.0' allows the server to be accessible from other devices on the local network
    # port=5000 is the default Flask port
    app.run(host="0.0.0.0", port=5000, debug=False)
