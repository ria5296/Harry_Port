import cv2

def gen_map():
    while True:
        frame = cv2.imread('static/map_sample.jpg')  # SLAM 이미지 예시
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
