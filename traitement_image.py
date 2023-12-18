import cv2
import numpy as np
import skimage.morphology as morph
import math

def getLcam(img, CHECKERBOARD, objp, mtx, dist):
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
       
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
    
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
    
    Lcam=mtx.dot(np.hstack((cv2.Rodrigues(rvecs)[0],tvecs)))
    return Lcam
    
    
def getOrigin(img, CHECKERBOARD, objp, mtx, dist):
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
    
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
    
    # On affiche l'origine sur l'image avec un point rouge
    imgpt, jac = cv2.projectPoints((0,0,0), rvecs, tvecs, mtx, dist)
    return (int(imgpt[0,0,0]), int(imgpt[0,0,1]))
    
    
def realPosition(px, py, Z, Lcam, squareDim):
    X=np.linalg.inv(np.hstack((Lcam[:,0:2],np.array([[-1*px],[-1*py],[-1]])))).dot((-Z*Lcam[:,2]-Lcam[:,3]))  
    return (X[0]*squareDim[0], X[1]*squareDim[1])


# Fonction combinant blur, opening et closing sur la composante de saturation (HSV)
def advancedThreshold(img, blur, th, closingTh, openingTh):

    imgBlur = cv2.GaussianBlur(img, (blur,blur), 7)
    imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)
    _, imgTh = cv2.threshold(imgHSV[:,:,1], th, 255, cv2.THRESH_BINARY)
    imgOpen = cv2.morphologyEx(imgTh, cv2.MORPH_OPEN, morph.disk(openingTh))
    imgClos = cv2.morphologyEx(imgOpen, cv2.MORPH_CLOSE, morph.disk(closingTh)) 
    return imgClos

            
def getContoursNoDisplay(img,imgContour,th,Lcam,squareDim):
    
    # Recupere les contours
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
    # Compte le nombre d'objets
    objectCount = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # On ne considere que les contours suffisement grands
        if area > th:
            objectCount = objectCount + 1
            
    objectPositions = np.zeros([int(objectCount),3])
    
    # Recuperation des positions
    i = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # On ne considere que les contours suffisement grands
        if area > th:
            # Trouver le centre du polygone et recuperer sa couleur
            M = cv2.moments(cnt)
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
            pos = realPosition(cX, cY, 0, Lcam, squareDim)
            objectPositions[i, 0] = pos[0]
            objectPositions[i, 1] = pos[1]
            i = i+1     

    return objectPositions

def CalibrationPosition(imgCalib, CHECKERBOARD, squareDim):
    
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    # Charger paramètres intrinsèques
    with np.load("./calibration/calibration.npz") as X:
        mtx, dist = [X[i] for i in ('mtx','dist')]
    
    Lcam = getLcam(imgCalib, CHECKERBOARD, objp, mtx, dist)
    origin = getOrigin(imgCalib, CHECKERBOARD, objp, mtx, dist)
    
    return [Lcam, origin]

def GetObjectsPositions(img, opening, closing, blur, threshold, minSize, Lcam, origin, CHECKERBOARD, squareDim):
    
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Filtrage pour utilisation
    imgFiltered = advancedThreshold(imgHSV, blur*2+1, threshold, closing*2+1, opening*2+1)    # IMPORTER CA
    # Detection du contour
    imgCanny = cv2.Canny(imgFiltered, 4, 4)
    imgContour = img.copy()
    objectsPositions = getContoursNoDisplay(imgCanny, imgContour,minSize,Lcam,squareDim)    # IMPORTER CA
    return objectsPositions

# Definition de la matrice de changement de repère a partir des points mesurés
def MatriceChangementRepere(posOrigine, posCoinX, posCoinY):

    nouvX = posCoinX-posOrigine
    nouvY = posCoinY-posOrigine
    
    nouvX[2] = 0
    nouvY[2] = 0
    
    nouvX = nouvX/(math.sqrt(nouvX[0]*nouvX[0]+nouvX[1]*nouvX[1]))
    nouvY = nouvY/(math.sqrt(nouvY[0]*nouvY[0]+nouvY[1]*nouvY[1]))
        
    M = [[nouvX[0], nouvY[0], 0, posOrigine[0]],[nouvX[1], nouvY[1], 0, posOrigine[1]],[0, 0, 1, posOrigine[2]],[0, 0, 0, 1]]
    
    return M