import cv2
import numpy as np

# 이미지 파일 경로
image_path = 'KakaoTalk_Photo_2024-05-17-21-29-11 001.jpeg'

# 이미지 읽기
frame = cv2.imread(image_path)
original_frame = frame
# 이미지를 그레이스케일로 변환
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# 캐니 엣지 검출 적용
edges = cv2.Canny(gray, 100, 150)

# 푸리에 변환 적용
rows, cols = edges.shape
f = np.fft.fft2(edges)
fshift = np.fft.fftshift(f)
magnitude_spectrum = 20 * np.log(np.abs(fshift))

# 중앙의 고주파 성분 제거 (저역통과 필터 적용)
crow, ccol = rows // 2, cols // 2
mask = np.zeros((rows, cols), np.uint8)
r = 50  # 필터 반경
center = [crow, ccol]
x, y = np.ogrid[:rows, :cols]
mask_area = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= r * r
mask[mask_area] = 1
fshift = fshift * mask

# 역 푸리에 변환
f_ishift = np.fft.ifftshift(fshift)
img_back = np.fft.ifft2(f_ishift)
img_back = np.abs(img_back)

# 배경 제거된 이미지
frame = img_back

# 이미지를 그레이스케일로 변환 (필요시)
if len(frame.shape) == 3 and frame.shape[2] == 3:
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
frame_origin = frame
# 반전
# frame = cv2.bitwise_not(frame)



# 블러링
# blurred = cv2.GaussianBlur(frame, (11, 11), 0)

# 적응형 임계값 적용
# blurred = blurred.astype(np.uint8)
# adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

# 추가 블러링
# adaptive_thresh = cv2.GaussianBlur(adaptive_thresh, (11, 11), 0)

# 이진화
# _, binary = cv2.threshold(adaptive_thresh, 50, 255, cv2.THRESH_BINARY_INV)

# 형태학적 변환
# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
# morph = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)

morph = frame 
morph = morph.astype(np.uint8)


# 윤곽선 검출
contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 작은 물체로 판단되는 윤곽선 필터링 및 표시
for contour in contours:
    area = cv2.contourArea(contour)
    if 3000 < area < 15000:  # 면적 기준으로 작은 물체 필터링 (적절히 조절 가능)
        print(area)
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(morph, (x, y), (x + w, y + h), (255, 255, 255), 2)
morph = cv2.bitwise_not(morph)

# 결과 출력
cv2.imshow('original_frame', original_frame)
cv2.imshow('frame_origin', frame_origin)
cv2.imshow('morph', morph)
# cv2.imshow('frame', frame)

# 'q' 키를 누르면 종료
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
