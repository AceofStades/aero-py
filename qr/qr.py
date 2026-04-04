import cv2 as cv
from qreader import QReader

cap = cv.VideoCapture(0)
qreader = QReader()
ret = True
count = 0
while ret:
    ret, frame = cap.read()
    cv.imshow("webcam", frame)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break

    if count % 60 == 0:
        text = qreader.detect_and_decode(frame)
        print(text)
    count += 1

cap.release()
cv.destroyAllWindows()
