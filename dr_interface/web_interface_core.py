import requests
from bs4 import BeautifulSoup


# Class that interfaces with thea web page to control the DeepRacer, load models, and receive camera data
class DRInterface():
    def __init__(self, password, ip='192.168.1.100', name="deep_racer"):
        self.session = requests.Session()
        self.password = password
        self.name = name
        self.ip = ip

        # basic URLs that are needed for logging on and retrieving data
        self.URL = "https://" + self.ip + "/"  # Main URL for logging onto DeepRacer web page for first time
        self.post_login_url = self.URL + "/login"  # Where to redirect with login password
        self.video_url = self.URL + "/route?topic=/video_mjpeg&width=480&height=360"

    def log_on(self):
        # Get the CSRF Token and logon on to a DeepRacer control interface session
        response = self.session.get(self.URL, verify=False)  # Cannot verify with Deep Racer
        soup = BeautifulSoup(response.text, 'lxml')
        csrf_token = soup.select_one('meta[name="csrf-token"]')['content']
        headers = {'X-CSRFToken': csrf_token}
        payload = {'password': self.password}
        post = self.session.post(self.post_login_url, data=payload, headers=headers, verify=False)
        print("LOGIN POST RETURN: " + str(post))

    def get_raw_video_stream(self):
        # Get the video stream
        return self.session.get(self.video_url, stream=True, verify=False)
