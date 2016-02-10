import datetime
import os
import random
import string
import time

ts = time.time()
datetime_ts = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d%H_%M_%S')

GLOBAL_STORAGE_BASE_PATH = "/tmp/rafcon_"+\
                           os.environ.get('USER', 'anonymous')+"_"+\
                           str(datetime_ts)+"_"+\
                           ''.join(random.choice(string.ascii_uppercase) for x in range(10))