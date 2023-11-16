import cv2
from cv2 import aruco
import yaml
import numpy as np
from pathlib import Path
from tqdm import tqdm
import os
import glob

# Defina esta flag como True para calibrar a câmera e False para validar resultados em tempo real
calibrate_camera = True

# Defina o caminho para as imagens

# Caminho atual do script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Caminho para a pasta de imagens relativa ao script atual
images_path = os.path.realpath(os.path.join("..", "aruco_data"))
images = glob.glob(f'{images_path}/*.png')

# Usando glob para listar arquivos PNG
for image_file in images:
    print(image_file)  # Imprime o caminho de cada arquivo de imagem

# Para validar os resultados, mostre o marcador aruco para a câmera.
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Forneça o comprimento do lado do marcador
markerLength = 20  # Aqui, a unidade de medida é centímetro.

# Crie um marcador ArUco para calibração
board = aruco.GridBoard_create(1, 1, float(markerLength) / 100, 0.01, aruco_dict)

# Descomente o bloco a seguir para desenhar e mostrar o marcador
# img_board = board.draw((200*3, 200*3))
# cv2.imwrite('aruco_board.png', img_board)

arucoParams = aruco.DetectorParameters_create()

if calibrate_camera:
    gray = None
    img_list = []
    
    print('Usando...', end='')
    for idx, fn in enumerate(images):
        print(idx, '', end='')
        img = cv2.imread(str(fn))
        img_list.append(img)
    print('imagens de calibração')

    corners_list, id_list, counter = [], [], []
    for im in tqdm(img_list):
        gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)
        if ids is not None:
            for corner in corners:
                # Reformata cada canto para ter dimensão (1, 4, 2)
                corners_list.append(corner.reshape((1, 4, 2)))
            id_list.append(ids)
            counter.append(len(ids))

    counter = np.array(counter).flatten()
    print("Calibrando câmera .... Aguarde...")
    corners_array = np.array(corners_list, dtype=np.float32)
    ids_array = np.array(id_list, dtype=np.float32)
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(corners_array, ids_array, counter, board, gray.shape[::-1], None, None)

    print("Matriz da câmera é \n", mtx, "\n E está armazenada no arquivo calibration.yaml junto com os coeficientes de distorção : \n", dist)
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    with open("calibration.yaml", "w") as f:
        yaml.dump(data, f)


else:
    camera = cv2.VideoCapture(0)
    ret, img = camera.read()

    with open('calibration.yaml') as f:
        loadeddict = yaml.load(f)
    mtx = loadeddict.get('camera_matrix')
    dist = loadeddict.get('dist_coeff')
    mtx = np.array(mtx)
    dist = np.array(dist)

    ret, img = camera.read()
    img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    h,  w = img_gray.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    pose_r, pose_t = [], []
    while True:
        ret, img = camera.read()
        img_aruco = img
        im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        h,  w = im_gray.shape[:2]
        dst = cv2.undistort(im_gray, mtx, dist, None, newcameramtx)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=arucoParams)
        #cv2.imshow("original", img_gray)
        if corners == None:
            print ("pass")
        else:

            ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, newcameramtx, dist) # For a board
            print ("Rotation ", rvec, "Translation", tvec)
            if ret != 0:
                img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                img_aruco = aruco.drawAxis(img_aruco, newcameramtx, dist, rvec, tvec, 10)    # axis length 100 can be changed according to your requirement

            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
        cv2.imshow("World co-ordinate frame axes", img_aruco)

cv2.destroyAllWindows()