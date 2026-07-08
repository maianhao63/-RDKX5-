from maix import camera, display, image, uart, time

uart_dev = uart.UART("/dev/serial0", 115200)
cam = camera.Camera(320, 240)
dis = display.Display()

# 红色识别阈值
RED = [0, 100, 40, 255, 0, 255]
W, H = 320, 240
CX, CY = W // 2, H // 2

# 标定参数
BLOCK_SIZE = 5.0       # 正方体边长5cm
FOCAL_PX = 240.0        # 等效像素焦距

while True:
    img = cam.read()
    blobs = img.find_blobs([RED], merge=True)
    img.draw_line(CX, 0, CX, H, image.COLOR_WHITE)
    img.draw_line(0, CY, W, CY, image.COLOR_WHITE)

    off_x = 0.0    # 左右偏移 X(cm)
    dis_y = 0.0    # 前后距离 Y(cm)
    send_ok = False

    if blobs:
        b = blobs[0]
        # 取平均像素尺寸
        avg_px = (b.w() + b.h()) / 2.0
        # 相似三角形求真实前后距离Y
        dis_y = (FOCAL_PX * BLOCK_SIZE) / avg_px
        # 像素偏移转真实左右偏移X
        center_x = b.x() + b.w() / 2
        px_diff = center_x - CX
        off_x = (px_diff * dis_y) / FOCAL_PX

        # 画面绘制
        img.draw_rect(b.x(), b.y(), b.w(), b.h(), image.COLOR_GREEN, 2)
        img.draw_circle(int(center_x), b.y()+b.h()//2, 5, image.COLOR_RED, -1)
        img.draw_string(10, 10, f"X:{off_x:.1f} Y:{dis_y:.1f}cm", scale=2)
        send_ok = True

    # 发送格式：X偏移,Y距离
    if send_ok:
        uart_dev.write(f"{off_x:.2f},{dis_y:.2f}\n".encode())
        print(f"发送：左右X={off_x:.2f}cm 前后Y={dis_y:.2f}cm")

    dis.show(img)
    time.sleep(0.05)