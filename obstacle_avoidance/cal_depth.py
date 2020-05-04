import numpy as np
import cv2
import camera_configs
import distance
# cv2.namedWindow("left")
# cv2.namedWindow("right")
# cv2.namedWindow("depth")
# cv2.moveWindow("left", 0, 0)
# cv2.moveWindow("right", 600, 0)
# cv2.createTrackbar("num", "depth", 0, 10, lambda x: None)
# cv2.createTrackbar("blockSize", "depth", 5, 255, lambda x: None)
# camera1 = cv2.VideoCapture(0)
# camera2 = cv2.VideoCapture(1)


# def callbackFunc(e, x, y, f, p):
# if e == cv2.EVENT_LBUTTONDOWN:
#   print threeD[y][x]
 
# cv2.setMouseCallback("depth", callbackFunc, None)

def cal_dep(frame1, frame2):
    #ret1, frame1 = camera1.read()
    # ret2, frame2 = camera2.read()

    # if not ret1 or not ret2:
        # break

    #print("566566566666555")
    img1_rectified = cv2.remap(frame1, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(frame2, camera_configs.right_map1, camera_configs.right_map2, cv2.INTER_LINEAR)
    # img1_rectified = frame1
    # img2_rectified = frame2

    imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)
    #print("11111")

    # num = cv2.getTrackbarPos("num", "depth")
    # blockSize = cv2.getTrackbarPos("blockSize", "depth")
    # if blockSize % 2 == 0:
    #  blockSize += 1
    # if blockSize < 5:
    # blockSize = 5

    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)
    disparity = stereo.compute(imgL, imgR)
    #disparity = stereo.compute(img1_rectified, img2_rectified)

    disp = cv2.normalize(disparity, disparity, alpha=5, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    #cv2.rectangle(disp, (240, 180), (360, 270), (255, 0, 0), 2)
    # cv2.rectangle(disp, (160, 40), (478, 334), (255, 0, 0), 2)
    cv2.rectangle(disp, (160, 40), (266, 138), (255, 0, 0), 1)
    cv2.rectangle(disp, (266, 40), (372, 138), (255, 0, 0), 1)
    cv2.rectangle(disp, (372, 40), (478, 138), (255, 0, 0), 1)
    cv2.rectangle(disp, (160, 138), (266, 236), (255, 0, 0), 1)
    cv2.rectangle(disp, (266, 138), (372, 236), (255, 0, 0), 1)
    cv2.rectangle(disp, (372, 138), (478, 236), (255, 0, 0), 1)
    cv2.rectangle(disp, (160, 236), (266, 334), (255, 0, 0), 1)
    cv2.rectangle(disp, (266, 236), (372, 334), (255, 0, 0), 1)
    cv2.rectangle(disp, (372, 236), (478, 334), (255, 0, 0), 1)

    threeD = cv2.reprojectImageTo3D(disparity.astype(np.float32)/16., camera_configs.Q)

    cv2.imshow("left", frame1)
    cv2.imshow("right", imgR)
    cv2.imshow("gray", disp)
    cv2.waitKey(1)
    # #print("222222")
    #  #print(threeD)
    # sum1 = 0
    # sum2 = 0
    # a = 0
    # b = 0
    # c = 0
    # d = 0
    # distance = []
    #
    # for i in range(0, 98):
    #     for j in range(0, 106):
    #         a = np.array(threeD[i+138][j+266][2])
    #         if np.isinf(a):
    #             a = 400
    #             sum0 = a
    #             distance.append(a)
    #         else:
    #             sum0 = abs(a)
    #             distance.append(sum0)
    #         sum1 = sum1+sum0
    # average1 = sum1 / (98 * 106)
    # temp = np.array(distance)
    # average2 = round(temp.mean())
    # average3 = round(average2)
    # if average2 < 140:
    #     average2 = 140
    # elif average2 > 440:
    #     average2 = 440
    # else:
    #     average2=(int(average2/5))*5
    # average = []
    # average1 = distance.distance_cal(threeD, 40, 160)
    # average.append(average1)
    # average2 = distance.distance_cal(threeD, 40, 266)
    # average.append(average2)
    # average3 = distance.distance_cal(threeD, 40, 372)
    # average.append(average3)
    # average4 = distance.distance_cal(threeD, 138, 160)
    # average.append(average4)
    # average5 = distance.distance_cal(threeD, 138, 266)
    # average.append(average5)
    # average6 = distance.distance_cal(threeD, 138, 372)
    # average.append(average6)
    # average7 = distance.distance_cal(threeD, 236, 160)
    # average.append(average7)
    # average8 = distance.distance_cal(threeD, 236, 266)
    # average.append(average8)
    # average9 = distance.distance_cal(threeD, 236, 372)
    # average.append(average9)
    # print(average)
    # for i in range(0, 9):
    #     print(i+1)
    #     print(average[i])

    # print(threeD.shape)

    return threeD
    cv2.destroyAllWindows()
    # print(threeD[270][320])
    # key = cv2.waitKey(1)
    # if key == ord("q"):
    #      return
    # elif key == ord("s"):
    #     cv2.imwrite("BM_left.jpg", imgL)
    #     cv2.imwrite("BM_right.jpg", imgR)
    #     cv2.imwrite("BM_depth.jpg", threeD)

 
# camera1.release()
# camera2.release()

