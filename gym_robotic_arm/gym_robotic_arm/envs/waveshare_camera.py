import cv2


class WaveShareCamera:
    # https://www.waveshare.com/wiki/OV2710_2MP_USB_Camera_(A)
    # OV2710 2MP USB Camera (A) 1920×1080

    def __init__(self, round_str):
        # initiate cameras
        self.clamp_cam = cv2.VideoCapture(0)
        self.laptop_cam = cv2.VideoCapture(1)
        self.cameras_active = self.clamp_cam.isOpened() and self.laptop_cam.isOpened()
        self.cameras = [self.clamp_cam, self.laptop_cam]
        self.laptop_cam_res = [1280, 720]  # width, height
        # camera: https://www.sossolutions.nl/waveshare-ov2710-2mp-usb-camera-a-low-light-sensitivity?gclid=CjwKCAiAzrWOBhBjEiwAq85QZ46Il8CN38SxIpZrvZ56z-IghpgvatB7D990TEnIgrXWM6l-1vPRHBoCRusQAvD_BwE
        self.clamp_cam_res = [1920, 1080]

        # initiate recording
        self.frames = [None, None]

        # variables for saving stuff
        self.round = round_str
        self.current_frame = 0
        self.name = None

    def return_cam_obs(self):
        if not self.cameras_active:
            return None
        # Capture frame-by-frame
        for i, c in enumerate(self.cameras):
            # Handles the mirroring of the current frame
            self.frames[i] = cv2.flip(c.read(), 1)

        # Our operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return self.frames

    def _show_feed(self, continoues=True):
        if continoues:
            waitkey = 1
        else:
            waitkey = 0
        for i in range(len(self.frames)):
            if self.frames[i] is not None:
                cv2.imshow(f'frame{i+1}', self.frames[i])
        if cv2.waitKey(waitkey) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
        # When everything done, release the capture
        for c in self.cameras:
            c.release()
        cv2.destroyAllWindows()

    def show_feed_continuous(self):
        self._show_feed(continoues=True)

    def show_feed(self):
        self.current_frame += 1
        self._show_feed(continoues=False)

    def save_image(self):
        # Saves image of the current frame in jpg file
        for i in range(len(self.frames)):
            name = f"round{i+1}." + str(self.round) + 'frame_' + str(self.current_frame) + '.jpg'
            cv2.imwrite(name, self.frames[i])
