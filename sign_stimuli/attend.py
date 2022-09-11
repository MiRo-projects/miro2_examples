import numpy as np
import cv2

class Attend:

    def __init__(self):
        self.old_frame = [None, None]
        self.old_gray = [None, None]
        self.p0 = [None, None]
        self.color = np.random.randint(0, 255, (100, 3))


    def activate(self, im, imageno):
        # Parameters for ShiTomasi corner detection
        # self.kinvel = self.kinvel - 0.8*self.kinvel

        feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        # Parameters for Lucas Kanade optical flow
        lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        
        if self.old_frame[imageno] is None:
            # Take first frame and find corners in it
            self.old_frame[imageno] = im
            self.old_gray[imageno] = cv2.cvtColor(self.old_frame[imageno], cv2.COLOR_BGR2GRAY)
            self.p0[imageno] = cv2.goodFeaturesToTrack(self.old_gray[imageno], mask=None, **feature_params)

        # Create a mask image for drawing purposes
        mask = np.zeros_like(self.old_frame[imageno])
        
        # Read new frame
        frame = im
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.image_w = frame.shape[1]
        self.image_h = frame.shape[0]

        # Calculate Optical Flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray[imageno], frame_gray, self.p0[imageno], None, **lk_params
        )

        target = None
        vf =  0

        if p1 is None:
            self.p0[imageno] = cv2.goodFeaturesToTrack(self.old_gray[imageno], mask=None, **feature_params)
            p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray[imageno], frame_gray, self.p0[imageno], None, **lk_params
            )

        # Select good points
        good_new = p1[st == 1]
        good_old = self.p0[imageno][st == 1]

        result = np.array([0.0, 0.0])
        counter = np.array([0.0, 0.0])

        # Draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()

            v = np.sqrt((a - c)**2 + (b - d)**2)

            if v < 2:
                continue

            print(int(c), ", ", self.image_w)
            idx = int(c/(0.5*self.image_w))
            counter[idx if idx <= 1 else 1] += 1
            
            result += np.array([a-c, b-d])
            

        if np.linalg.norm(result) > 2:
            target = self.image_w/4.0 if counter[0]>counter[1] else 3.0*self.image_w/4.0
            mask = cv2.line(mask, (int(target), int(100)), (int(target) + int(result[0]), 100 + int(result[1])), self.color[i].tolist(), 2)
            frame = cv2.circle(frame, (int(target), int(100)), 20, self.color[i].tolist(), -1)
            
            target = 0 if counter[0]>counter[1] else self.image_w
            vf = result[0]
                
        
        self.p0[imageno] = good_new.reshape(-1, 1, 2)

            # Display the demo
        img = cv2.add(frame, mask)
        cv2.imshow(str(imageno), img)
        # k = cv2.waitKey(25) & 0xFF
        # if k == 27:
        #     return

        # Update the previous frame and previous points
        self.old_gray[imageno] = frame_gray.copy()
        # cv2.imshow('frame', im)
        if(imageno == 1):
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return target, vf
        return target, vf

