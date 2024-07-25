import cv2
import numpy as np
import glob


#写入照片
camera=cv2.VideoCapture(0)
i = 0
j = 0
# 找棋盘格角点
# 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) # 阈值
#棋盘格模板规格
w = 6   
h = 4  
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
objp = objp * 40  

# 储存棋盘格角点的世界坐标和图像坐标对
objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点

while 1:
    (grabbed, img) = camera.read()
#        filename = str('./Visual/SummerStudy/cv/img/'+si+'.jpg')
#        cv2.imwrite(filename, img)
#        print('写入：',filename)

    # 获取画面中心点
    # 获取图像的长宽
    h1, w1 = img.shape[0], img.shape[1]

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    u, v = img.shape[:2]
    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)

    if cv2.waitKey(1) == ord('j'):  # 按j保存一张图片
        i += 1
        si = str(i)

        filename = str(si+ '.jpg')
        cv2.imwrite(filename, img)

    # 如果找到足够点对，将其存储起来
    if ret == True:
        
        # 在原角点的基础上寻找亚像素角点
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        #追加进入世界三维点和平面二维点中
        objpoints.append(objp)
        imgpoints.append(corners)

        img_corner = img

        # 将角点在图像上显示
        cv2.drawChessboardCorners(img_corner, (w,h), corners, ret)
        cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('findCorners', 640, 480)

        cv2.imshow('findCorners',img_corner)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        cv2.imshow('findCorners',img)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 找棋盘格角点
# 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) # 阈值
#棋盘格模板规格
w = 6   # 10 - 1
h = 4   # 7  - 1
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
objp = objp * 40  # 40 mm

# 储存棋盘格角点的世界坐标和图像坐标对
objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点
#加载pic文件夹下所有的jpg图像
images = glob.glob('./img/*.jpg')  #   拍摄的十几张棋盘图片所在目录

j=0
for fname in images:

    img = cv2.imread(fname)
    # 获取画面中心点
    #获取图像的长宽
    h1, w1 = img.shape[0], img.shape[1]

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    u, v = img.shape[:2]
    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
    # 如果找到足够点对，将其存储起来
    if ret == True:
        print("img:", j)
        j = j+1
        # 在原角点的基础上寻找亚像素角点
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        #追加进入世界三维点和平面二维点中
        objpoints.append(objp)
        imgpoints.append(corners)
        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (w,h), corners, ret)
        cv2.namedWindow('findCorners', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('findCorners', 640, 480)
        cv2.imshow('findCorners',img)
        cv2.waitKey(200)
cv2.destroyAllWindows()

print('正在计算')

#标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


print("重投影误差ret:",ret)
print("内参矩阵mtx:\n",mtx)     
print("畸变参数dist:\n",dist)   # 畸变系数 distortion cofficients = (k_1,k_2,p_1,p_2,k_3)