
import pyautogui
import time
import pytesseract
# from PIL import Image
from PIL import Image, ImageEnhance, ImageFilter
# def click_on_chat():
#     time.sleep(2)
#     x, y = 8, 579
#     pyautogui.click(x, y)
#
#
# def type_and_send_message():
#     message = "=work"
#     pyautogui.typewrite(message)
#     pyautogui.press('enter')
#     time.sleep(0.5)
#     message2 = "=slot 300"
#     pyautogui.typewrite(message2)
#     pyautogui.press('enter')
#     time.sleep(0.5)
#     # message3 = "=heal"
#     # pyautogui.typewrite(message3)
#     # pyautogui.press('enter')
#
#
# def main():
#     i = 0
#     # print(i)
#     while True:
#         if i == 11:
#             input('Press ENTER to exit')
#         else:
#             i = i + 1
#             print(i)
#             click_on_chat()
#             type_and_send_message()
#             time.sleep(120)
#
#
# if __name__ == "__main__":
#     main()

def click_on_chat():
    time.sleep(1)
    x, y = 8, 579
    pyautogui.click(x, y)


def type_and_send_message():
    message = "=work"
    pyautogui.typewrite(message)
    pyautogui.press('enter')
    time.sleep(0.5)
    message2 = "=slot 300"
    pyautogui.typewrite(message2)
    pyautogui.press('enter')
    time.sleep(0.5)


# def get_number_from_image(image_path):
#     # 使用 pytesseract 辨識數字
#     image = binarize_image(image_path)
#     text = pytesseract.image_to_string(image, config='--psm 6 -c tessedit_char_whitelist=0123456789')
#     try:
#         return int(text.strip())
#     except ValueError:
#         return None

def binarize_image(img_path, threshold=150):
    # """二值化圖像"""
    # image = Image.open(img_path)
    # image = image.convert('L')  # 轉換成灰度圖像
    # pixels = image.load()
    # for x in range(image.width):
    #     for y in range(image.height):
    #         if pixels[x, y] > threshold:
    #             pixels[x, y] = 255
    #         else:
    #             pixels[x, y] = 0
    # return image
    """使用Adaptive Thresholding進行二值化"""
    image = Image.open(img_path)
    image = image.convert('L')  # 轉換成灰度圖像
    image = image.filter(ImageFilter.MedianFilter(5))
    enhancer = ImageEnhance.Contrast(image)
    image = enhancer.enhance(2)
    image = image.point(lambda p: p > 128 and 255)
    return image

def get_number_from_image(image_path):
    # 使用 pytesseract 辨識數字前先二值化圖像
    image = binarize_image(image_path)
    text = pytesseract.image_to_string(image, config='--psm 6 -c tessedit_char_whitelist=0123456789')
    time.sleep(1)
    try:
        return int(text.strip())
    except ValueError:
        return None

def main():
    i = 0
    pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
    while True:
        print(i)
        i = i + 1
        click_on_chat()
        message = "=work"
        pyautogui.typewrite(message)
        pyautogui.press('enter')
        time.sleep(2)
        # input('enter')
        # 如果需要圖像辨識的認證
        # 取得畫面截圖 (您可能需要調整座標和大小以匹配實際認證的位置)
        screenshot = pyautogui.screenshot(region=(397, 847, 202, 84))
        screenshot.save('temp.png')
        time.sleep(1.5)
        number = get_number_from_image('temp.png')
        if number:
            pyautogui.typewrite(str(number))
            pyautogui.press('enter')
            time.sleep(0.5)
            err = "0"
            pyautogui.typewrite(err)
            pyautogui.press('enter')
            message2 = "=slot 300"
            pyautogui.typewrite(message2)
            pyautogui.press('enter')
        else:
            print('沒偵測到數字')
            time.sleep(0.5)
            err1 = "1"
            pyautogui.typewrite(err1)
            pyautogui.press('enter')
            message2 = "=slot 300"
            pyautogui.typewrite(message2)
            pyautogui.press('enter')
            time.sleep(2)
            screenshot = pyautogui.screenshot(region=(397, 847, 202, 84))
            screenshot.save('temp.png')
            time.sleep(1.5)
            number = get_number_from_image('temp.png')
            if number:
                pyautogui.typewrite(str(number))
                pyautogui.press('enter')
            else:
                print('沒偵測到數字')
                err2 = "2"
                pyautogui.typewrite(err2)
                pyautogui.press('enter')
        # if i == 0 or i % 10 == 0:
        #     click_on_chat()
        #     message = "=work"
        #     pyautogui.typewrite(message)
        #     pyautogui.press('enter')
        #     time.sleep(2)
        #     # input('enter')
        #     # 如果需要圖像辨識的認證
        #     # 取得畫面截圖 (您可能需要調整座標和大小以匹配實際認證的位置)
        #     screenshot = pyautogui.screenshot(region=(397, 847, 202, 84))
        #     screenshot.save('temp.png')
        #     number = get_number_from_image('temp.png')
        #     if number:
        #         pyautogui.typewrite(str(number))
        #         pyautogui.press('enter')
        #         time.sleep(0.5)
        #         message2 = "=slot 300"
        #         pyautogui.typewrite(message2)
        #         pyautogui.press('enter')
        #         i = i + 1
        #     else:
        #         print('沒偵測到數字')
        # else:
        #     i = i + 1
        #     click_on_chat()
        #     type_and_send_message()
        time.sleep(60)


if __name__ == "__main__":
    main()
