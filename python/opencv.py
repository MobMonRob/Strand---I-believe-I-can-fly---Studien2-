import cv2 as cv

# Read images from static file
# img = cv.imread('eule.jpg')
# cv.namedWindow('image', cv.WINDOW_NORMAL)
# cv.imshow('image', img)
# cv.waitKey(0)
# cv.destroyAllWindows()

# Read images from webcam
cap = cv.VideoCapture(0)
sub_mog = cv.bgsegm.createBackgroundSubtractorMOG()
sub_mog2 = cv.createBackgroundSubtractorMOG2()
sub_gmg = cv.bgsegm.createBackgroundSubtractorGMG()
kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))

while (True):
    ret, frame = cap.read()
    frame = cv.flip(frame, 1)

    fg_mog = sub_mog.apply(frame)
    fg_mog2 = sub_mog2.apply(frame)
    fg_gmg = sub_gmg.apply(frame)
    fg_gmg_filtered = cv.morphologyEx(fg_gmg, cv.MORPH_OPEN, kernel)

    cv.imshow('Original', frame)
    cv.imshow('MOG', fg_mog)
    cv.imshow('MOG2', fg_mog2)
    cv.imshow('GMG', fg_gmg)
    cv.imshow('GMG Filtered', fg_gmg_filtered)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv.destroyAllWindows()
