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
    filename = 'test.json'
    print('Saving to file: ' + filename)
    json_data = json.dumps(data)
    with open(filename, 'w') as f:
        f.write(json_data)

def calibrate_camera():
    # Generate Charuco board
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    squareSize = 0.03907 #0.03
    markerSize = squareSize/1.5 #0.02
    board = aruco.CharucoBoard_create(7,5,squareSize,markerSize,dictionary)
    img = board.draw((int(1000*1.414),1000))
    cv2.imwrite('charuco.png',img)

    cap = cv2.VideoCapture(2)
    
    # Camera resolution
    res = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    
    ret, frame = cap.read()
    print(frame.shape)

    allCharucoCorners = []
    allCharucoIds = []

    frame_idx = 0
    frame_spacing = 20 # Only check every fifth frame
    required_count = 60
    success = False

    while True:

        # Capture and image
        ret,frame = cap.read()

        # Convert to gray scale
        grayframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        # Detect markers
        marker_corners, marker_ids, _ = aruco.detectMarkers(grayframe, dictionary)

        # If detected any markers
        if len(marker_corners)>0 and frame_idx % frame_spacing == 0:
            aruco.drawDetectedMarkers(grayframe, marker_corners, marker_ids)

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
            
        cv2.imshow('grayframe',grayframe)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        frame_idx += 1
        print("Found: "+str(len(allCharucoIds))+"/"+str(required_count))

        if len(allCharucoIds)>required_count:
            success = True
            break

    if success:
        print("Finished")
        try:    
            err, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                allCharucoCorners,
                allCharucoIds,
                board,
                res, 
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
            print("failed!")
            return

        # Generate the corrections  
        new_camera_matrix, valid_pix_roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            res,
            0)
        #new_camera_matrix = camera_matrix
        mapx, mapy = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix,
            res,
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

def check_camera():
    cap = cv2.VideoCapture(2)
    cam_id = 2

    while True:
        try:
            ret, frame = cap.read()
            cv2.imshow('frame',frame)

            if cv2.waitKey(1) & 255 == ord('q'):
                break
            if cv2.waitKey(1) & 255 == ord('n'):
                cam_id+=1
        except:
            cam_id+=1

    cap.release()
    cv2.destroyAllWindows()
        


if __name__ == "__main__":
    #check_camera()
    calibrate_camera()
