from pioneer_sdk import Pioneer, Camera
import os
import cv2
import cv2.aruco as aruco
import numpy as np
import time


# Функция обработки изображения, если в кадре есть маркеры, возвращает кадр с обведённым маркерами и списком их ID.
def image_proc(camera_frame):
    gray = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
    # основной метод подмодуля cv2.aruco, возвращает вектор углов (x, y) и список ID найденных маркеров
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)
    if np.all(ids is not None):  # если хоть 1 маркер найден
        aruco.drawDetectedMarkers(camera_frame, corners)  # обводит маркер (соединяет углы линиями)
    return camera_frame, ids


# Функция управления полётом по траектории "змейки", согласно ей дрон летит направо на расстоянии ширины поля,
# вперёд на длину шага, налево на расстояние ширины поля и т.д. Количество точек регулируется переменной max_counter
# (точка достигается каждый раз когда дрон меняет направление). Изначально настроено на квадрат 2.5х2.5 в метрах.
def drone_control(drone, camera_ip):
    length = float(2.5)  # ширина поля (ставить значение меньше, чтобы избежать столкновений)
    increment_len = float(0.5)  # длина шага
    flight_height = float(1.5)  # высота полёта
    counter = 1
    max_counter = 11
    command_x = 0
    command_y = 0
    ch_dir = False  # флажок для смены направления направо/налево
    new_point = True
    ids_buf = []  # буфер для ID найденных маркеров

    while True:
        try:
            if new_point:  # подаёт команду дрону двигаться на следующую точку, если он достиг прошлой
                drone.go_to_local_point(x=command_x, y=command_y, z=flight_height, yaw=0)
                new_point = False
            if drone.point_reached():  # если точка достигнута
                counter += 1
                print('[INFO] Новая точка достигнута')
                # если счётчик содержит нечётное число, летит вперёд
                if (counter % 2) != 0:
                    command_y += increment_len
                # если счётчик содержит чётное число и флажок неактивен, летит налево и активирует флажок
                elif (counter % 2) == 0 and ch_dir is False:
                    command_x += length
                    ch_dir = True
                # если счётчик содержит четное число и флажок активен, летит направо и дезактивирует флажок
                else:
                    command_x -= length
                    ch_dir = False
                new_point = True
            if counter == max_counter:  # последняя точка достигнута - вывод маркеров и завершение скрипта
                print('[INFO] Задание выполнено, дрон нашёл маркеры со следующими ID:', end=' ')
                for i in range(len(ids_buf)):
                    print(ids_buf[i], end=" ")
                break

            camera_frame = camera_ip.get_cv_frame()  # встроенный метод класса Camera, получает кадр с камеры дрона
            if np.sum(camera_frame) == 0:  # если не получил кадр, продолжает цикл (а не завершает скрипт)
                continue

            camera_frame, ids = image_proc(camera_frame)
            if np.all(ids is not None):
                for i in range(ids.size):  # для каждого маркера в кадре
                    if ids[i] not in ids_buf:  # если найденный маркер раньше не встречался
                        ids_buf.append(ids[i])  # записывает в буфер его ID
                        print("[INFO] Найден новый ArUco маркер, его ID: {}".format(ids))
                        print('Маркер сохраняется, его ID - ', ids[i])
                        # сохраняет кадр с обведённым маркером и вставляет его ID в конце названия файла
                        cv2.imwrite(os.path.join(ARUCO, 'markerid_%d.jpg' % ids[i]), camera_frame)
                        pioneer_mini.led_control(r=0, g=255, b=0)  # мигает
                        time.sleep(0.1)
                        pioneer_mini.led_control(r=0, g=0, b=0)
        except cv2.error:  # в случае ошибки не завершает скрипт предварительно, а продолжает работу в цикле
            continue

        cv2.imshow('Marker Detection', camera_frame)  # вывод изображения на компьютер
        key = cv2.waitKey(1)
        if key == 27:  # завершает скрипт, если нажата клавиша ESC
            print('[INFO] ESC нажат, программа завершается')
            break


if __name__ == '__main__':  # скрипт запускается только напрямую
    ARUCO = os.path.join(os.getcwd(), "ARUCO")  # создаёт папку ARUCO для сохранения маркеров
    if not os.path.isdir(ARUCO):
        os.mkdir(ARUCO)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)  # задаёт формат принимаемых маркеров Aruco
    aruco_parameters = aruco.DetectorParameters_create()
    pioneer_mini = Pioneer()  # pioneer_mini как экземпляр класса Pioneer
    pioneer_mini.arm()
    pioneer_mini.takeoff()
    camera = Camera()

    drone_control(pioneer_mini, camera)

    pioneer_mini.land()
    time.sleep(5)
    cv2.destroyAllWindows()  # закрывает окно с выводом изображения
    exit(0)
