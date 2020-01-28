# Camera calibration
# https://github.com/dclemmon/projection_mapping/blob/master/aruco_calibration.py


import cv2
from cv2 import aruco
import json

def save_json(data):
    """
    Save our data object as json to the camera_config file
    :param data: data to  write to file
    """
    filename = 'camera_config.json'
    print('Saving to file: ' + filename)
    json_data = json.dumps(data)
    with open(filename, 'w') as f:
        f.write(json_data)

def calibrate_camera():

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    board = aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
    img = board.draw((200,200))
    #cv2.imwrite('charuco.png',img)

    cap = cv2.VideoCapture(0)

    allCharucoCorners = []
    allCharucoIds = []

    frame_idx = 0
    frame_spacing = 5 # Only check every fifth frame
    required_count = 20
    success = False

    while True:

        # Capture and image
        ret,frame = cap.read()

        # Convert to gray scale
        grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        # Detect markers
        marker_corners, marker_ids, _ = aruco.detectMarkers(grayframe,dictionary)
        aruco.drawDetectedMarkers(grayframe, marker_corners, marker_ids)

        # If detected any markers
        if len(marker_corners)>0 and frame_idx % frame_spacing == 0:
            # Detect corners
            ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                grayframe,
                board)
            

            #If detected any corners
            if charuco_corners is not None and charuco_ids is not None and len(charuco_corners)>3:
                allCharucoCorners.append(charuco_corners)
                allCharucoIds.append(charuco_ids)
                # Draw detected corners
                aruco.drawDetectedCornersCharuco(grayframe, charuco_corners,charuco_ids)
            
        #cv2.imshow('grayframe',grayframe)

        if (cv2.waitKey(0) & 0xFF == ord('q')):
            break

        frame_idx += 1
        print("Found: "+str(len(allCharucoIds))+"/"+str(required_count))

        if len(allCharucoIds)>required_count:
            success = True
            break

    imsize = grayframe.shape
    print(imsize)

    if success:
        print("Finished")
        try:    
            err, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                allCharucoCorners,
                allCharucoIds,
                board,
                imsize, 
                None, 
                None)
            print('Calibrated with error',err)
            print(camera_matrix)
            save_json({
                    'camera_matrix': camera_matrix.tolist(),
                    'dist_coeffs': dist_coeffs.tolist(),
                    'err': err
                })
            
        except:
            success = False
            print(e)

        # Generate the corrections  
        new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            imsize,
            0)
        mapx, mapy = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix,
            imsize,
            5)
        while True:
            ret,frame = cap.read()
            if mapx is not None and mapy is not None:
                frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 255 == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate_camera()