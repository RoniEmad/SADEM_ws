import cv2
import time

vid = cv2.VideoCapture(0+cv2.CAP_V4L2)

count = 0
while True:
	ret, frame = vid.read()
	#cv2.imshow("frame", frame)
	#if cv2.waitKey(1) & 0xFF == ord('q'):
	#	break
	if count == 0:
		startTime = time.monotonic()
	count += 1
	print(count/(time.monotonic()-startTime))

vid.release()
cv2.destroyAllWindows()
