import cv2 as cv
import numpy as np


def color_filtering(detecting_color, hsv_image):     # 이미지 내 특정 색상 검출 함수
    if detecting_color == 1: # Blue
        lower_color = np.array([110, 50, 50])
        upper_color = np.array([130, 255, 255])
        mask = cv.inRange(hsv_image, lower_color, upper_color) # 색상 범위에 해당하는 마스크 생성
    elif detecting_color == 2: # Green
        lower_color = np.array([50, 50, 50])
        upper_color = np.array([70, 255, 255])
        mask = cv.inRange(hsv_image, lower_color, upper_color)
    elif detecting_color == 3: # Red
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])
        lower_color2 = np.array([170, 50, 50])
        upper_color2 = np.array([180, 255, 255])
        mask = cv.inRange(hsv_image, lower_color, upper_color) + cv.inRange(hsv_image, lower_color2, upper_color2)
    return mask

def shape_and_label(detecting_shape, raw_image, contours):    # 원하는 도형의 윤곽선 면적과 중심점 찾기, 도형 labelling
    contour_info = []
    for contour in contours:
        approx = cv.approxPolyDP(contour, cv.arcLength(contour, True) * 0.01, True)
        line_num = len(approx)
        if detecting_shape == 0:  # 원 ### 수정이필요해 ~ㅠ
            _, radius = cv.minEnclosingCircle(approx)  # 원으로 근사
            ratio = radius * radius * 3.14  # 해당 넓이와 정원 간의 넓이 비
            if 0.5 < ratio < 2:  # 원에 가까울 때만 필터링
                area, center = find_area_centroid(contour)
                setLabel(raw_image, contour, 'CIRCLE')
            else:
                area, center = None, None
        elif detecting_shape == 3 and line_num == 3:  # 삼각형
            area, center = find_area_centroid(contour)
            setLabel(raw_image, contour, 'TRIANGLE')
        elif detecting_shape == 4 and line_num == 4:  # 사각형
            area, center = find_area_centroid(contour)
            setLabel(raw_image, contour, 'RECTANGLE')
        elif detecting_shape == 12 and line_num == 12:  # 십자가
            area, center = find_area_centroid(contour)
            setLabel(raw_image, contour, 'CROSS')
        else:
            continue

        if area is not None and center is not None:
            contour_info.append((area, center))
            cv.circle(raw_image, center, 5, (255, 0, 0), -1)

    return contour_info, raw_image

def find_area_centroid(contour):
    # 윤곽선의 면적 계산
    area = cv.contourArea(contour)
    center = (0,0)
    # 윤곽선의 중심점 계산
    if area > 10000 : # 인식 면적 제한 두기
        M = cv.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center = (cx, cy)
            return area, center
        else:
            return None, None      
    else :
#        print('중심점계산오류')
        return None, None

def image_preprocessing(cam): 
    # 영상 이미지 전처리 함수
    raw_image = cam
    img0 = mean_brightness(raw_image) # 평균 밝기로 보정하는 함수
    img = cv.GaussianBlur(img0, (5, 5), 0) # 가우시안 필터 적용 # (n,n) : 가우시안 필터의 표준편차. 조정하면서 해야 함
    hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV) # BGR 형식의 이미지를 HSV 형식으로 전환
    return hsv_image

def show_the_shape_contour(hsv_image,detecting_color):
    # 탐지 범위에 따른 마스크 형성 및 외곽선 검출하는 함수   
    mask = color_filtering(detecting_color, hsv_image)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
    contours = np.array(contours)
#    contours = contours.astype(np.float)
    return contours

def show_the_shape_info(raw_image, detecting_shape,contours) :
    contour_info, raw_image = shape_and_label(detecting_shape, raw_image, contours)
#    cv.imshow("CONTROLLER", raw_image)
    servo_angle, thruster_value = move_with_largest(contour_info, raw_image.shape[1]) # return : servo, thruster
    return raw_image, servo_angle, thruster_value


def move_with_largest(contour_info, raw_image_width):
    # 제일 큰 도형 선택
#    print("Contour Info Before Filtering:", contour_info)  # Print contour_info before filtering
    contour_info = [info for info in contour_info if info[0] is not None]  # (area, center) 가 None인 경우 필터링
#    print("Contour Info After Filtering:", contour_info)  # Print contour_info after filtering
    Limage_limit = raw_image_width / 2 - 10
    Rimage_limit = raw_image_width / 2 + 10
    servo = 93
    thruster = 50 # thruster_mid
    if len(contour_info) > 0: # contour에 area, center가 입력되었을 때 ( 도형이 1개 이상 인식되었을 때 )
        contour_info.sort(key=lambda x: x[0], reverse=True)  # 도형 면적 기준으로 area, center 내림차순 정렬
        largest_contour = contour_info[0] # 제일 큰 도형 선택
        # 제일 큰 도형에 대한 연산 수행
        largest_area, largest_center = largest_contour
        centroid_x = largest_center[0]
        a = 5 # 도형 크기 비 (직진 여부 확인용)
        largest_width = largest_area  # 도형의 가로 길이를 largest_area로 간주

        if centroid_x < Limage_limit :
        # and largest_width < raw_image_width / a: # center의 x좌표가 화면 절반보다 왼쪽에 있을 때 : 왼쪽으로 회전
        #    print(centroid_x, Limage_limit, largest_width, raw_image_width, "Move Left")
        #    print(contour_info)
            print("Left")
            servo = 81

        elif centroid_x > Rimage_limit :
        # and largest_width < raw_image_width / a: # center의 x좌표가 화면 절반보다 오른쪽에 있을 때 : 오른쪽으로 회전
        #    print(centroid_x, Limage_limit, largest_width, raw_image_width, "Move Right")
        #    print(contour_info)
            print("Right")
            servo = 105

        elif Limage_limit < centroid_x < Rimage_limit and largest_width < raw_image_width / a :
            print("Move Front")
            servo = 93
            thruster = 100 # thruster_max
        elif Limage_limit < centroid_x < Rimage_limit and largest_width > raw_image_width / a :
            print("STOP")
            servo = 93
            thruster = 30 # thruster_min
        ## 예외case
        # elif centroid_x < Limage_limit and largest_width > raw_image_width / a : 
        #     print("case1")
        # elif centroid_x > Rimage_limit and largest_width > raw_image_width / a :
        #     print("case2") 
    else:
        print("No contour found")
    print("servo : ", servo, "thruster : ", thruster)
    return servo, thruster
#    print(contour_info)  # Print contour_info for debugging

def mean_brightness(img):
    fixed = 100  # 이 값 주변으로 평균 밝기 조절함
    m = cv.mean(img)  # 평균 밝기
    scalar = (-int(m[0]) + fixed, -int(m[1]) + fixed, -int(m[2]) + fixed, 0)
    dst = cv.add(img, scalar)
    return dst

def setLabel(img, pts, label):
    (x,y,w,h) = cv.boundingRect(pts)
    pt1 = (x,y)
    pt2 = (x+w, y+h)
    cv.rectangle(img, pt1, pt2, (0,255,0), 2)
    cv.putText(img, label, (pt1[0], pt1[1]-3), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def test_with_webcam(detecting_color, detecting_shape):
    """ 웹캠으로 기능 테스트 """
    """
    detecting_color : Blue = 1, Green = 2, Red = 3
    detecting_shape : Circle = 0, Triangle = 3, Rectangle = 4, cross = 12
    """

    webcam = cv.VideoCapture(2) # 캠 연결된 USB 포트 번호 수정하기

    if not webcam.isOpened(): # 캠이 연결되지 않았을 경우 # true시 캠이 잘 연결되어있음
        print("Could not open webcam")
        exit()

    # 카메라에 보이는 이미지가 연결되어 동영상으로 보이는 형태
    while webcam.isOpened(): 
        ret, cam = webcam.read() # webcam으로 연결된 정보 읽어오기
#        cv.imshow('webcam', cam) # webcam 창에 cam 보이기
        # raw_img = cv.imread(webcam, cv.IMREAD_COLOR)
        # cv.imshow("RAW_IMG", raw_img)

    # 1. 영상 이미지 전처리
        raw_image = cam
        img0 = mean_brightness(raw_image) # 평균 밝기로 보정하는 함수
        img = cv.GaussianBlur(img0, (5, 5), 0) # 가우시안 필터 적용 # (n,n) : 가우시안 필터의 표준편차. 조정하면서 해야 함
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV) # BGR 형식의 이미지를 HSV 형식으로 전환

    # 2. 탐지 색상 범위에 따라 마스크 형성
        mask = color_filtering(detecting_color, hsv_image)

    # 3. 형성된 마스크에서 외곽선 검출 ( datatype 변경 )
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # 컨투어 검출
        contours = np.array(contours)
    #    contours = contours.astype(np.float)
        
        contour_info, raw_image = shape_and_label(detecting_shape, raw_image, contours)
        cv.imshow("CONTROLLER", raw_image)
        h,w,c = raw_image.shape # 원본 이미지에서 가로 길이 받아오기
        move_with_largest(contour_info, w)

        if cv.waitKey(1) & 0xFF == 27: # esc버튼 누르면 창 꺼짐
            break 

test_with_webcam(3,3)