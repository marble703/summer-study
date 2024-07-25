import cv2
camera=cv2.VideoCapture(0)
i = 0
while 1:
    (grabbed, img) = camera.read()
    cv2.imshow('img',img)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        u = str(i)
        filename = str('./'+u+'.jpg')
        cv2.imwrite(filename, img)
        print('写入：',filename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

