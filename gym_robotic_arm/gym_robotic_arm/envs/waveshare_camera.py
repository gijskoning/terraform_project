
import cv2

class WaveShareCamera:
    # https://www.waveshare.com/wiki/OV2710_2MP_USB_Camera_(A)
    # OV2710 2MP USB Camera (A) 1920×1080

    def __init__(self, round):
        #initiate cameras
        self.clamp_cam = cv2.VideoCapture(0)
        self.laptop_cam = cv2.VideoCapture(1)

        self.laptop_cam_res = [1280, 720] # width, height
        # camera: https://www.sossolutions.nl/waveshare-ov2710-2mp-usb-camera-a-low-light-sensitivity?gclid=CjwKCAiAzrWOBhBjEiwAq85QZ46Il8CN38SxIpZrvZ56z-IghpgvatB7D990TEnIgrXWM6l-1vPRHBoCRusQAvD_BwE
        self.clamp_cam_res = [1920, 1080]

        # initiate recording
        self.ret1 = None
        self.frame1 = None
        self.ret2 = None
        self.frame2 = None

        # variables for saving stuff
        self.round = round
        self.current_frame = 0
        self.name = None

    def return_cam_obs(self):
        # Capture frame-by-frame
        self.ret1, self.frame1 = self.clamp_cam.read()
        self.ret2, self.frame2 = self.laptop_cam.read()

        # Handles the mirroring of the current frame
        self.frame1 = cv2.flip(self.frame1,1)
        self.frame2 = cv2.flip(self.frame2,1)

        # Our operations on the frame come here
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return [self.frame1, self.frame2]

    def show_feed_continuous(self):
        # Display the resulting frame
        cv2.imshow('frame1', self.frame1)
        cv2.imshow('frame2', self.frame2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows
        # When everything done, release the capture
        self.clamp_cam.release()
        self.laptop_cam.release()
        cv2.destroyAllWindows()

    def show_feed(self):
        self.current_frame += 1
        cv2.imshow('frame1', self.frame1)
        cv2.imshow('frame2', self.frame2)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows
        # When everything done, release the capture
        self.clamp_cam.release()
        self.laptop_cam.release()
        cv2.destroyAllWindows()

    def save_image(self):
        # Saves image of the current frame in jpg file
        self.name1 = "round1." + str(self.round) + 'frame_' + str(self.current_frame) + '.jpg'
        cv2.imwrite(self.name1, self.frame1)
        self.name2 = "round2." + str(self.round) + 'frame_' + str(self.current_frame) + '.jpg'
        cv2.imwrite(self.name2, self.frame2)