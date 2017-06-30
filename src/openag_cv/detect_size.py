from scipy.spatial import distance as dist   # Need to add to install list - check with ros depends - sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2

def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)



def creating_image_mask(luminance):
	# cv2.imshow("luminance", luminance)
	#cv2.imshow("luv", luv)
	#split the luv image and retain only band 1
	cv2.imwrite("luminance.png", luminance)
	ret, thresh = cv2.threshold(luminance, 127, 255, cv2.THRESH_BINARY)
	# cv2.imshow("hist", thresh)
	ret1, thresh2 = cv2.threshold(thresh, 0, 255, cv2.THRESH_OTSU)
	# cv2.imshow("otsu", thresh2)
	mask_inv = cv2.bitwise_not(thresh2)
	return mask_inv



def detect_edges(mask_inv):
	edged = cv2.dilate(mask_inv, None, iterations=1)
	edged = cv2.erode(edged, None, iterations=1)
	#cv2.imshow("final", edged)
	# find contours in the edge map
	cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	count_of_edges = cnts[0] if imutils.is_cv2() else cnts[1]
	# sort the contours from left-to-right and initialize the
	# 'pixels per metric' calibration variable
	(count_of_edges, _) = contours.sort_contours(count_of_edges)
	return count_of_edges

def process_image(frame):
	frame = np.array(frame, dtype=np.uint8)
	luv = cv2.cvtColor(frame, cv2.COLOR_BGR2LUV)
	#cv2.imshow("luv", luv)
	#split the luv image and retain only band 1
	l, u, v = cv2.split(luv)
	mask_inv = creating_image_mask(l)

	#cv2.imshow("inv", mask_inv)
	count_of_edges = detect_edges(mask_inv)
	pixelsPerMetric = None

	# loop over the contours individually
	for edge in count_of_edges:
		# if the contour is not sufficiently large, ignore it
		if cv2.contourArea(edge) < 10000:
			continue

		# compute the rotated bounding box of the contour
		orig = frame.copy()
		box = cv2.minAreaRect(edge)
		box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
		box = np.array(box, dtype="int")

		# order the points in the contour such that they appear
		# in top-left, top-right, bottom-right, and bottom-left
		# order, then draw the outline of the rotated bounding
		# box
		box = perspective.order_points(box)
		cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

		# loop over the original points and draw them
		for (x, y) in box:
			cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

		# unpack the ordered bounding box, then compute the midpoint
		# between the top-left and top-right coordinates, followed by
		# the midpoint between bottom-left and bottom-right coordinates
		(topleft, topright, bottomright, bottomleft) = box
		print topleft, topright, bottomright, bottomleft
		(tltrX, tltrY) = midpoint(topleft, topright)
		(blbrX, blbrY) = midpoint(bottomleft, bottomright)

		# compute the midpoint between the top-left and top-right points,
		# followed by the midpoint between the top-righ and bottom-right
		(tlblX, tlblY) = midpoint(topleft, bottomleft)
		(trbrX, trbrY) = midpoint(topright, bottomright)
		print (tlblX, tlblY)
		print (trbrX, trbrY)

		cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
		cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
		cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
		cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

		# draw lines between the midpoints
		cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
			(255, 0, 255), 2)
		cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
			(255, 0, 255), 2)

		# compute the Euclidean distance between the midpoints
		dA_euclideandist = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
		dB_euclideandist = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
		print dA_euclideandist
		print dB_euclideandist


		# if the pixels per metric has not been initialized, then
		# compute it as the ratio of pixels to supplied metric
		# (in this case, inches)
		if pixelsPerMetric is None:
			pixelsPerMetric = dB_euclideandist / 10

		# compute the size of the object
		dimension_A = dA_euclideandist / pixelsPerMetric
		dimension_B = dB_euclideandist / pixelsPerMetric

		# draw the object sizes on the image
		cv2.putText(orig, "{:.1f}in".format(dimension_A),
			(int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
			0.65, (255, 255, 255), 2)
		cv2.putText(orig, "{:.1f}in".format(dimension_B),
			(int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
			0.65, (255, 255, 255), 2)
	return [orig,
	       [(tltrX, tltrY),
	        (blbrX, blbrY),
	        (tlblX, tlblY),
		    (trbrX, trbrY)],
		   dA_euclideandist,
		   dB_euclideandist,
    	   dimension_A,
    	   dimension_B]

if __name__ == "__main__":

	im = cv2.imread("/tmp/VemxPh.jpg")
	frame = np.array(im, dtype=np.uint8)
	process_image(frame)

