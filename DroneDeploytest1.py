import math




import cv2

import numpy as np

imgpath = 'images/IMG_6726.jpg'


img = cv2.imread(imgpath)

print("Processing image: ", imgpath)

img = cv2.resize(img, (600, 800))

imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

retval, thresholdedimage = cv2.threshold(imgray, 190, 255, cv2.THRESH_BINARY)

contourimage, contours, hierarchy = cv2.findContours(thresholdedimage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

cv2.imshow("image with contours", img)

cv2.imwrite("intermediateoutput/Contoured-image.PNG", img);

cv2.waitKey()

areas = [cv2.contourArea(contour) for contour in contours]

max_area_index = np.argmax(areas)

patterncontour = contours[max_area_index]

d = {}
for contour in contours:

    M = cv2.moments(contour)

    if M["m00"] == 0:

        continue

    cX = int(M["m10"] / M["m00"])

    cY = int(M["m01"] / M["m00"])

    if d.get((cX, cY), 0):

        d[(cX, cY)] += 1 

    else:

        d[(cX, cY)] = 1 

three_hits = []

two_hits = []

for key in d:

    if d[key] == 3:

        three_hits.append(key)

    if d[key] == 2:

        two_hits.append(key)

if len(three_hits) == 3:

    pass

elif len(three_hits) == 2:

    pass

else:

    pass


rect = cv2.minAreaRect(patterncontour)

box = cv2.boxPoints(rect)


box = np.int0(box)




cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

rotatedangle = rect[2]




print("Rotation Angle of the barcode pattern: {0:.2f} ".format(rotatedangle))

def mid_point(point_X, point_Y):

    return [(point_X[0] + point_Y[0]) / 2, (point_X[1] + point_Y[1]) / 2]


def distance(point_X, point_Y):

    return math.sqrt((point_X[0] - point_Y[0]) ** 2 + (point_X[1] - point_Y[1]) ** 2)

min_dist = 1000


for point in box:

    temp_dist = distance(patterncontour[0][0], point)

    min_dist = min(min_dist, temp_dist)

print("Degree from the birds eye view:  {0:.2f} ".format(min_dist))

P = box[0]

Q = box[1]

R = box[2]

S = box[3]


P_Q = mid_point(P, Q)

R_S = mid_point(R, S)

P_S = mid_point(P, S)

Q_R = mid_point(Q, R)


PS_QR_dist = distance(P_S, Q_R)

PQ_RS_dist = distance(P_Q, R_S)




if PS_QR_dist > PQ_RS_dist:

    width = PS_QR_dist

    height = PS_QR_dist

else:

    width = PS_QR_dist

    height = PS_QR_dist


width_ratio_1foot = 450

height_ratio_1foot = 580


width_ratio_2foot = 219

height_ratio_2foot = 285

one_foot_height = 1 / (height / height_ratio_1foot)

one_foot_width = 1 / (width / width_ratio_1foot)




two_foot_height = 2 / (height / height_ratio_2foot)

two_foot_width = 2 / (width / width_ratio_2foot)

distance_away = (one_foot_width + one_foot_height + two_foot_width + two_foot_height) / 4




print("Image was taken {0:.2f} feet away ".format(distance_away, min_dist, rotatedangle))




cv2.drawContours(img, contours, max_area_index, (0, 125, 0), 3)

cv2.imshow("output image", img)

cv2.imwrite("intermediateoutput/output-image.PNG", img);

cv2.waitKey(0)

cv2.destroyAllWindows()
