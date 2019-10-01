import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[0.91979737155165608, -0.005650779796284422, 14.549250291682462], [0.0050984125702820127, 0.92535323173382056, 46.503440026116891]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.world_frame = np.zeros(3)
        self.projection =np.array([[  1.86365121e-03  , 1.85859997e-05  ,-6.00905701e-01],
                                    [  1.77272276e-05 , -1.87306461e-03  , 5.14438954e-01],
                                    [ -3.16498746e-19 ,  7.91435291e-19 ,  1.00000000e+00]]
                                                                                        )

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            #if(self.kinectCalibrated):
            self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            #else:
            #    self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertBlockFrame(self) :
        BlockFrame = self.detectBlocksInDepthImage()
        
        try:
            self.DepthHSV[...,0] = BlockFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)
            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            
            return img
        except:
            return None


    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        A =np.array([])
        matrix_affine = np.array([])
        ma_vect =[]
        #print(self.kinect.rgb_click_points[0])
        for i, rgb in enumerate(coord1) :
            a = np.array([[rgb[0],rgb[1],1,0,0,0],[0,0,0,rgb[0],rgb[1],1]])
            if(i==0) :
                A =a
            else :
                A = np.concatenate((A,a),axis = 0)
        coord2 = coord2.flatten()
        ma_vect = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(A),A)),np.transpose(A)),coord2)
        #print(ma_vect)
        matrix_affine = [[ma_vect[0],ma_vect[1],ma_vect[2]],[ma_vect[3],ma_vect[4],ma_vect[5]]]
        return matrix_affine

    def getAffineTransform3(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        A =np.array([])
        matrix_affine = np.array([])
        ma_vect =[]
        #print(self.kinect.rgb_click_points[0])
        for i, rgb in enumerate(coord1) :
            a = np.array([[rgb[0],rgb[1],rgb[2],0,0,0,0,0,0],[0,0,0,rgb[0],rgb[1],rgb[2],0,0,0],[0,0,0,0,0,0,rgb[0],rgb[1],rgb[2]]])
            if(i==0) :
                A =a
            else :
                A = np.concatenate((A,a),axis = 0)
        coord2 = coord2.flatten()
        ma_vect = np.dot(np.linalg.pinv(A),coord2)
        #print(ma_vect)
        matrix_affine = [[ma_vect[0],ma_vect[1],ma_vect[2]],[ma_vect[3],ma_vect[4],ma_vect[5]],[ma_vect[6],ma_vect[7],ma_vect[8]]]
        return matrix_affine
        

    def registerDepthFrame(self, frame):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        
        #self.depth2rgb_affine =cv2.getPerspectiveTransform(self.depth_click_points,self.rgb_click_points)
        
        self.depth2rgb_affine = np.array(self.depth2rgb_affine)
        return cv2.warpAffine(frame,self.depth2rgb_affine,(640,480))
       
        #rgb_frame = np.dot(self.depth2rgb_affine,frame)
        #return rgb_frame
        

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
        affine = np.loadtxt("calibration.cfg",dtype=float,delimiter=',')
       # print(affine)
        return affine
        pass
    '''
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        # rects: (x,y), (width, length), theta
        ROI = self.detectBlocksInDepthImage()
        return ROI

        # filter w.r.t colors


        #return rects[0][0], rects[0][2]*np.pi/180
        # return (x, y, theta)

      '''  

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        I_depth = self.currentDepthFrame
        #print(I_depth.shape)
        ROI = np.zeros((I_depth.shape[0], I_depth.shape[1]),np.uint8)
        '''
        #Convert world to rgb for bounding box
        #World coordinates for bounding box
        Zc = 939
        bounding_box = 310
        bounding_wc = np.array([[-bounding_box,-0,Zc],[0,bounding_box,Zc],[bounding_box,0,Zc],[0,-bounding_box,Zc]])
        bounding_rgb = np.zeros((4,3))
        for i,wc in enumerate(bounding_wc) :
            rgb = (1/Zc) * np.dot(np.linalg.inv(self.projection),wc)
            bounding_rgb[i,:] = rgb
       '''
        ROI[100:460,146:490] = I_depth[100:460,146:490]
        ROI[230:320,270:370] = 255
        
        ret, I_th = cv2.threshold(ROI, 200, 255, cv2.THRESH_BINARY_INV)
        I_th = cv2.blur(I_th, (7,7))
        #print(I_th.shape)
        image, contours,hierarchy = cv2.findContours(I_th,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        rects = list()
        mask = np.zeros(I_depth.shape,np.uint8)
        max_depth =960
        min_depth = 700
        for i,contour in enumerate(contours):
            area = cv2.contourArea(contour)

            if area>2000 or area<100:
                # contours.remove(contour)
                pass
            else:
                rect = cv2.minAreaRect(contour)
                temp = rect[0]
                x=temp[0]
                y=temp[1]
                z = self.currentDepthFrame[int(y)][int(x)]
                depth = 1000* 0.1236 * np.tan(z/2842.5 + 1.1863)
                print(depth)
                if depth > max_depth or depth<min_depth :
                    pass
                else :



                # print(contour)
                

                    cv2.drawContours(mask, contours, i, 255, -1)
                    rects.append(rect)
                #masks.append(cv2.drawContours(mask, contours, i, 255, -1))
        #cv2.imshow('Depth', I_depth)
        
        
        #cv2.imshow('Depth_th', I_th)
       # cv2.imshow('I_crop', ROI)
        #cv2.imshow('mask 1', masks[0])
        #cv2.imshow('mask 2', masks[1])
        if rects :
           print(rects)


        return mask


'''
def test():
    print('open!!')
    kinect = Kinect()
    kinect.loadDepthFrame()
    kinect.loadVideoFrame()

    # kinect.detectBlocksInDepthImage()
    print(kinect.blockDetector())



if __name__ == '__main__':
    test()
'''