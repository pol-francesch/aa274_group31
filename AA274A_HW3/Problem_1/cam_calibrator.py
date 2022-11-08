#!/usr/bin/env python3

import pdb
import os
import sys

import cv2
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from camera_calibration.calibrator import (
    MonoCalibrator,
    ChessboardInfo,
    Patterns,
)


class CameraCalibrator:
    def __init__(self):
        self.calib_flags = 0
        self.pattern = Patterns.Chessboard

    def loadImages(
        self,
        cal_img_path,
        name,
        n_corners,
        square_length,
        n_disp_img=1e5,
        display_flag=True,
    ):
        self.name = name
        self.cal_img_path = cal_img_path

        self.boards = []
        self.boards.append(
            ChessboardInfo(
                n_cols=n_corners[0],
                n_rows=n_corners[1],
                dim=float(square_length),
            )
        )
        self.c = MonoCalibrator(self.boards, self.calib_flags, self.pattern)

        if display_flag:
            fig = plt.figure("Corner Extraction", figsize=(12, 5))
            gs = gridspec.GridSpec(1, 2)
            gs.update(wspace=0.025, hspace=0.05)

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            img = cv2.imread(
                self.cal_img_path + "/" + file, 0
            )  # Load the image
            img_msg = self.c.br.cv2_to_imgmsg(
                img, "mono8"
            )  # Convert to ROS Image msg
            drawable = self.c.handle_msg(
                img_msg
            )  # Extract chessboard corners using ROS camera_calibration package

            if display_flag and i < n_disp_img:
                ax = plt.subplot(gs[0, 0])
                plt.imshow(img, cmap="gray")
                plt.axis("off")

                ax = plt.subplot(gs[0, 1])
                plt.imshow(drawable.scrib)
                plt.axis("off")

                plt.subplots_adjust(
                    left=0.02, right=0.98, top=0.98, bottom=0.02
                )
                fig.canvas.set_window_title(
                    "Corner Extraction (Chessboard {0})".format(i + 1)
                )

                plt.show(block=False)
                plt.waitforbuttonpress()

        # Useful parameters
        self.d_square = square_length  # Length of a chessboard square
        self.h_pixels, self.w_pixels = img.shape  # Image pixel dimensions
        self.n_chessboards = len(
            self.c.good_corners
        )  # Number of examined images
        (
            self.n_corners_y,
            self.n_corners_x,
        ) = n_corners  # Dimensions of extracted corner grid
        self.n_corners_per_chessboard = n_corners[0] * n_corners[1]

    def genCornerCoordinates(self, u_meas, v_meas):
        """
        Inputs:
            u_meas: a list of arrays where each array are the u values for each board.
            v_meas: a list of arrays where each array are the v values for each board.
        Output:
            corner_coordinates: a tuple (Xg, Yg) where Xg/Yg is a list of arrays where each array are the x/y values for each board.

        HINT: u_meas, v_meas starts at the blue end, and finishes with the pink end
        HINT: our solution does not use the u_meas and v_meas values
        HINT: it does not matter where your frame is, as long as you are consistent!
        HINT: You MAY find the function np.meshgrid() useful.
        """
        ########## Code starts here ##########
        # Pick top left as origin
        d = self.d_square
        x = np.arange(start=d, stop=d*(self.n_corners_x+1), step=d, dtype=np.float64)
        y = np.arange(start=d, stop=d*7.5, step=d, dtype=np.float64)

        # Normalize
        x = x / x.max()
        y = y / y.max()

        Xg, Yg = np.meshgrid(x,y)

        corner_coordinates = (np.repeat(Xg[np.newaxis,...], self.n_chessboards, axis=0).tolist(), np.repeat(Yg[np.newaxis,...], self.n_chessboards, axis=0).tolist())

        ########## Code ends here ##########
        return corner_coordinates

    def estimateHomography(self, u_meas, v_meas, X, Y):  # Zhang Appendix A
        """
        Inputs:
            u_meas: an array of the u values for a board.
            v_meas: an array of the v values for a board.
            X: an array of the X values for a board. (from genCornerCoordinates)
            Y: an array of the Y values for a board. (from genCornerCoordinates)
        Output:
            H: the homography matrix. its size is 3x3

        HINT: What is the size of the matrix L?
        HINT: What are the outputs of the np.linalg.svd function? Based on this, where does the eigenvector corresponding to the smallest eigen value live?
        HINT: Some numpy functions that might come in handy are stack, vstack, hstack, column_stack, expand_dims, zeros_like, and ones_like.
        """
        ########## Code starts here ##########
        # Prep work
        zeros_mat = np.zeros((1,3))
        n = u_meas.shape[0]

        L = np.zeros((2*n, 9))
        X = np.array(X).flatten()
        Y = np.array(Y).flatten()

        # Calulate M_tilde and L
        for point in range(0, n):
            M_tilde = np.array([[X[point]], [Y[point]], [1]], dtype=np.float64).T

            L[2*point:2*(point+1),:] = np.vstack((np.hstack((M_tilde, zeros_mat, -u_meas[point]*M_tilde)),
                                                  np.hstack((zeros_mat, M_tilde, -v_meas[point]*M_tilde))))

        # Use SVD to get the eigenvectors and eigenvalues
        u, s, _ = np.linalg.svd(L.T@L)

        # Pick the eigenvector associated with the smallest eigenvalue
        x = u[np.argmin(s), :]

        H = x.reshape((3,3))
        ########## Code ends here ##########
        return H

    def getCameraIntrinsics(self, H):  # Zhang 3.1, Appendix B
        """
        Input:
            H: a list of homography matrices for each board
        Output:
            A: the camera intrinsic matrix

        HINT: MAKE SURE YOU READ SECTION 3.1 THOROUGHLY!!! V. IMPORTANT
        HINT: What is the definition of h_ij?
        HINT: It might be cleaner to write an inner function (a function inside the getCameraIntrinsics function)
        HINT: What is the size of V?
        """
        ########## Code starts here ##########
        def calc_vij(H, i, j):
            return np.array([
                    [H[0,i] * H[j,0]],
                    [H[0,i] * H[j,1] + H[1,i] * H[j,0]],
                    [H[1,i] * H[j,1]],
                    [H[2,i] * H[j,0] + H[0,i] * H[j,2]],
                    [H[2,i] * H[j,1] + H[1,i] * H[j,2]],
                    [H[2,i] * H[j,2]]
                ]).T
        
        # Calculate V
        V = np.zeros((2*len(H), 6))

        for i in range(0, self.n_chessboards):
            v12 = calc_vij(H[i], 1, 0)
            v11 = calc_vij(H[i], 0, 0)
            v22 = calc_vij(H[i], 1, 1)

            V[2*i:2*(i+1), :] = np.vstack((v12, (v11-v22)))

        # Use SVD to get the eigenvectors and eigenvalues
        u, s, _ = np.linalg.svd(V.T@V)

        # Pick the eigenvector associated with the smallest eigenvalue
        b = u[np.argmin(s), :]
        B11 = b[0]; B12 = b[1]; B22 = b[2]; B13 = b[3]; B23 = b[4]; B33 = b[5]

        # Get intrisic parameters
        v0    = (B12*B13 - B11*B23) / (B11*B22 - B12*B12)
        lam   = B33 - (B13*B13 + v0*(B12*B13 - B11*B23)) / B11
        alpha = np.sqrt(lam / B11)
        beta  = np.sqrt(lam*B11 / (B11*B22 - B12*B12))
        gamma = -B12*alpha*alpha*beta/lam
        u0    = gamma * v0 / beta - B13 * alpha*alpha/lam

        # Build A
        A = np.array([[alpha, gamma, u0],
                      [0, beta, v0],
                      [0, 0, 1]])
        print(A)
        ########## Code ends here ##########
        return A

    def getExtrinsics(self, H, A):  # Zhang 3.1, Appendix C
        """
        Inputs:
            H: a single homography matrix
            A: the camera intrinsic matrix
        Outputs:
            R: the rotation matrix
            t: the translation vector
        """
        ########## Code starts here ##########





        ########## Code ends here ##########
        return R, t

    def transformWorld2NormImageUndist(
        self, X, Y, Z, R, t
    ):  # Zhang 2.1, Eq. (1)
        """
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            x, y: the coordinates in the ideal normalized image plane

        """
        ########## Code starts here ##########




        ########## Code ends here ##########
        return x, y

    def transformWorld2PixImageUndist(
        self, X, Y, Z, R, t, A
    ):  # Zhang 2.1, Eq. (1)
        """
        Inputs:
            X, Y, Z: the world coordinates of the points for a given board. This is an array of 63 elements
                     X, Y come from genCornerCoordinates. Since the board is planar, we assume Z is an array of zeros.
            A: the camera intrinsic parameters
            R, t: the camera extrinsic parameters (rotation matrix and translation vector) for a given board.
        Outputs:
            u, v: the coordinates in the ideal pixel image plane
        """
        ########## Code starts here ##########





        ########## Code ends here ##########
        return u, v

    def plotBoardPixImages(
        self, u_meas, v_meas, X, Y, R, t, A, n_disp_img=1e5, k=np.zeros(2)
    ):
        # Expects X, Y, R, t to be lists of arrays, just like u_meas, v_meas

        fig = plt.figure(
            "Chessboard Projection to Pixel Image Frame", figsize=(8, 6)
        )
        plt.clf()

        for p in range(min(self.n_chessboards, n_disp_img)):
            plt.clf()
            ax = plt.subplot(111)
            ax.plot(u_meas[p], v_meas[p], "r+", label="Original")
            u, v = self.transformWorld2PixImageUndist(
                X[p], Y[p], np.zeros(X[p].size), R[p], t[p], A
            )
            ax.plot(u, v, "b+", label="Linear Intrinsic Calibration")

            box = ax.get_position()
            ax.set_position(
                [
                    box.x0,
                    box.y0 + box.height * 0.15,
                    box.width,
                    box.height * 0.85,
                ]
            )
            ax.axis([0, self.w_pixels, 0, self.h_pixels])
            plt.gca().set_aspect("equal", adjustable="box")
            plt.title("Chessboard {0}".format(p + 1))
            ax.legend(
                loc="lower center",
                bbox_to_anchor=(0.5, -0.3),
                fontsize="medium",
                fancybox=True,
                shadow=True,
            )

            plt.show(block=False)
            plt.waitforbuttonpress()

    def plotBoardLocations(self, X, Y, R, t, n_disp_img=1e5):
        # Expects X, U, R, t to be lists of arrays, just like u_meas, v_meas

        ind_corners = [
            0,
            self.n_corners_x - 1,
            self.n_corners_x * self.n_corners_y - 1,
            self.n_corners_x * (self.n_corners_y - 1),
        ]
        s_cam = 0.02
        d_cam = 0.05
        xyz_cam = [
            [0, -s_cam, s_cam, s_cam, -s_cam],
            [0, -s_cam, -s_cam, s_cam, s_cam],
            [0, -d_cam, -d_cam, -d_cam, -d_cam],
        ]
        ind_cam = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]]
        verts_cam = []
        for i in range(len(ind_cam)):
            verts_cam.append(
                [
                    list(zip(
                        [xyz_cam[0][j] for j in ind_cam[i]],
                        [xyz_cam[1][j] for j in ind_cam[i]],
                        [xyz_cam[2][j] for j in ind_cam[i]],
                    ))
                ]
            )

        fig = plt.figure("Estimated Chessboard Locations", figsize=(12, 5))
        axim = fig.add_subplot(121)
        ax3d = fig.add_subplot(122, projection="3d")

        boards = []
        verts = []
        for p in range(self.n_chessboards):

            M = []
            W = np.column_stack((R[p], t[p]))
            for i in range(4):
                M_tld = W.dot(
                    np.array([X[p][ind_corners[i]], Y[p][ind_corners[i]], 0, 1])
                )
                if np.sign(M_tld[2]) == 1:
                    Rz = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                    M_tld = Rz.dot(M_tld)
                    M_tld[2] *= -1
                M.append(M_tld[0:3])

            M = (np.array(M).T).tolist()
            verts.append([list(zip(M[0], M[1], M[2]))])
            boards.append(Poly3DCollection(verts[p]))

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            if i < n_disp_img:
                img = cv2.imread(self.cal_img_path + "/" + file, 0)
                axim.imshow(img, cmap="gray")
                axim.axis("off")

                ax3d.clear()

                for j in range(len(ind_cam)):
                    cam = Poly3DCollection(verts_cam[j])
                    cam.set_alpha(0.2)
                    cam.set_color("green")
                    ax3d.add_collection3d(cam)

                for p in range(self.n_chessboards):
                    if p == i:
                        boards[p].set_alpha(1.0)
                        boards[p].set_color("blue")
                    else:
                        boards[p].set_alpha(0.1)
                        boards[p].set_color("red")

                    ax3d.add_collection3d(boards[p])
                    ax3d.text(
                        verts[p][0][0][0],
                        verts[p][0][0][1],
                        verts[p][0][0][2],
                        "{0}".format(p + 1),
                    )
                    plt.show(block=False)

                view_max = 0.2
                ax3d.set_xlim(-view_max, view_max)
                ax3d.set_ylim(-view_max, view_max)
                ax3d.set_zlim(-2 * view_max, 0)
                ax3d.set_xlabel("X axis")
                ax3d.set_ylabel("Y axis")
                ax3d.set_zlabel("Z axis")

                if i == 0:
                    ax3d.view_init(azim=90, elev=120)

                plt.tight_layout()
                fig.canvas.set_window_title(
                    "Estimated Board Locations (Chessboard {0})".format(i + 1)
                )

                plt.show(block=False)

                try:
                    raw_input("<Hit Enter To Continue>")
                except NameError:
                    input("<Hit Enter To Continue>")

    def undistortImages(self, A, k=np.zeros(2), n_disp_img=1e5, scale=0):
        Anew_no_k, roi = cv2.getOptimalNewCameraMatrix(
            A, np.zeros(4), (self.w_pixels, self.h_pixels), scale
        )
        mapx_no_k, mapy_no_k = cv2.initUndistortRectifyMap(
            A,
            np.zeros(4),
            None,
            Anew_no_k,
            (self.w_pixels, self.h_pixels),
            cv2.CV_16SC2,
        )
        Anew_w_k, roi = cv2.getOptimalNewCameraMatrix(
            A, np.hstack([k, 0, 0]), (self.w_pixels, self.h_pixels), scale
        )
        mapx_w_k, mapy_w_k = cv2.initUndistortRectifyMap(
            A,
            np.hstack([k, 0, 0]),
            None,
            Anew_w_k,
            (self.w_pixels, self.h_pixels),
            cv2.CV_16SC2,
        )

        if k[0] != 0:
            n_plots = 3
        else:
            n_plots = 2

        fig = plt.figure("Image Correction", figsize=(6 * n_plots, 5))
        gs = gridspec.GridSpec(1, n_plots)
        gs.update(wspace=0.025, hspace=0.05)

        for i, file in enumerate(sorted(os.listdir(self.cal_img_path))):
            if i < n_disp_img:
                img_dist = cv2.imread(self.cal_img_path + "/" + file, 0)
                img_undist_no_k = cv2.undistort(
                    img_dist, A, np.zeros(4), None, Anew_no_k
                )
                img_undist_w_k = cv2.undistort(
                    img_dist, A, np.hstack([k, 0, 0]), None, Anew_w_k
                )

                ax = plt.subplot(gs[0, 0])
                ax.imshow(img_dist, cmap="gray")
                ax.axis("off")

                ax = plt.subplot(gs[0, 1])
                ax.imshow(img_undist_no_k, cmap="gray")
                ax.axis("off")

                if k[0] != 0:
                    ax = plt.subplot(gs[0, 2])
                    ax.imshow(img_undist_w_k, cmap="gray")
                    ax.axis("off")

                plt.subplots_adjust(
                    left=0.02, right=0.98, top=0.98, bottom=0.02
                )
                fig.canvas.set_window_title(
                    "Image Correction (Chessboard {0})".format(i + 1)
                )

                plt.show(block=False)
                plt.waitforbuttonpress()

    def writeCalibrationYaml(self, A, k):
        self.c.intrinsics = np.array(A)
        self.c.distortion = np.hstack(([k[0], k[1]], np.zeros(3))).reshape(
            (1, 5)
        )
        # self.c.distortion = np.zeros(5)
        self.c.name = self.name
        self.c.R = np.eye(3)
        self.c.P = np.column_stack((np.eye(3), np.zeros(3)))
        self.c.size = [self.w_pixels, self.h_pixels]

        filename = self.name + "_calibration.yaml"
        with open(filename, "w") as f:
            f.write(self.c.yaml())

        print("Calibration exported successfully to " + filename)

    def getMeasuredPixImageCoord(self):
        u_meas = []
        v_meas = []
        for chessboards in self.c.good_corners:
            u_meas.append(chessboards[0][:, 0][:, 0])
            v_meas.append(
                self.h_pixels - chessboards[0][:, 0][:, 1]
            )  # Flip Y-axis to traditional direction

        return u_meas, v_meas  # Lists of arrays (one per chessboard)
