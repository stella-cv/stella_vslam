from boto3 import client
from os import environ
from dotenv import load_dotenv
import subprocess
from urllib.parse import unquote
import re
import tempfile
from concurrent import futures


'''
    1) Execute a python script that spawns a process to run stella_vslam_convertion
    2) Uploads the output 360 Image frames and camera Localization Output to the target path in S3 using boto3
'''

load_dotenv()


class Utils:
    @staticmethod
    def extract_s3_info(object_url: str) -> [str, str]:
        """ 
            Extracts Bucket and the object key from the S3 url resource
        """
        regex = "https://s3\.amazonaws\.com/(.*?)/(.*)"
        match = re.match(regex, object_url)
        bucket, object_key = match.group(1), unquote(match.group(2))
        return [bucket, object_key]


class Processor:

    def __init__(self) -> None:
        self._S3_READ_CLIENT = client(
            "s3",
            region_name=environ.get('AWS_BUCKET_REGION')
        )
        self.skip_frame = environ.get('SKIP_FRAME', 3)
        self.video_path = environ.get('VIDEO_PATH')
        self.camera_path = environ.get('CAMERA_PATH')
        self.output_path = ''

    def localize(self):
        '''
            Run the stella-vslam localizer to extract the map information
        '''
        subprocess.run([
            './run_video_slam', '-v', '/tmp/data/orb_vocab.fbow', '-m', self.video_path, '-c', self.camera_path, '--frame-skip', self.skip_frame, '--no-sleep',
            '--map-db-in', self.output_path
        ])

    def download(self, source_url: str):
        """
            Downloads the file object from Source (s3) and creates a temporary file
        """
        bucket, object_key = Utils.extract_s3_info(
            source_url.get("s3_path"))

        temp_file = tempfile.NamedTemporaryFile(delete=False)

        self._S3_READ_CLIENT.download_fileobj(
            Bucket=bucket, Key=object_key, Fileobj=temp_file)

        return temp_file.name

    def fetch_data(self):
        '''
            Downloads the source data
        '''

        self.camera_local_path = self.download(self.camera_path)
        self.video_local_path = self.download(self.video_local_path)

    def upload(self):
        '''
            Uploads the processed data back to the target path
        '''
        # TODO: This changes after the new changes in stella vslam

    def acknowledge(self):
        '''
            Acknowledges by sending a processed message back to rabbitMQ
        '''
        pass

    def process(self):
        '''
            Run all the tasks to process the source data
        '''
        self.fetch_data()
        self.localize()
        self.upload()
        self.acknowledge()


if __name__ == '__main__':

    processor = Processor()
    processor.localize()
