# Dado que se usa SIFT, se requiere instalar lo siguiente:
#      pip install opencv-contrib-python
#

import cv2
from matplotlib import pyplot as plt
import numpy as np

class SIFT_detector(object):
      def __init__(self, max_keypoints=15,):
        self.max_keypoints = max_keypoints

      def detect(self, image1, image2):
        #print(image1.shape)
        #print(image1.size)
        if (image1.shape == (256,256,3)):
            self.I1 = cv2.cvtColor(image1, cv2.COLOR_RGB2GRAY)
            self.I2 = cv2.cvtColor(image2, cv2.COLOR_RGB2GRAY)
            # Atributos importantes con el descritor SIFT
            sift = cv2.xfeatures2d.SIFT_create()
            keypts1, descriptores1 = sift.detectAndCompute(self.I1, None)
            keypts2, descriptores2 = sift.detectAndCompute(self.I2, None)

            # Parámetros de FLANN para ser usados con SIFT
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=10) # A más chequeos, más exactitud (más lento)

            # Correspondencias con FLANN
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(descriptores1, descriptores2, k=2)

            # Máscara vacía para dibujar
            mask_matches = [[0, 0] for i in range(len(matches))]
            # Llenar valores usando el ratio
            NNDR = 0.1
            for i, (m, n) in enumerate(matches):
                if m.distance < NNDR * n.distance:
                    mask_matches[i]=[1, 0]

            # Dibujo de las correspondencias
            img_matches = cv2.drawMatchesKnn(self.I1, keypts1, self.I2, keypts2, matches, None,
                                             matchColor=(0, 255, 0), 
                                             singlePointColor=(255, 0, 0),
                                             matchesMask=mask_matches, flags=0)

            return img_matches

        else:
            return np.array(0)