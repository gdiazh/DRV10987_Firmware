'''
* @file file_handler.py
* @author Gustavo Diaz H.
* @date 04 Aug 2020
* @brief Data csv file handler
'''
import pandas as pd
import datetime
from time import strftime
from pathlib import Path

class fileHandler(object):
    def __init__(self, folder, test_name, debug = False):
        self.debug = debug
        self.folder = folder
        self.file_ext = ".csv"
        self.test_date = datetime.datetime.now().strftime('%Y-%m-%d %H-%M-%S')
        self.file_name = folder+self.test_date+"["+test_name+"]"
        self.file_path = Path(self.file_name+self.file_ext)
        self.data_names = ["time[s]", "current[mA]", "speed[RPM]", "speed[cmd]"]
        self.historical_time = []
        self.historical_current_ma = []
        self.historical_speed_rpm = []
        self.historical_speed_cmd = []

    def init(self):
        Path(self.folder).mkdir(parents=True, exist_ok=True)

    def save(self, test_specs):
        data = {"time[s]": self.historical_time,
                "current[mA]": self.historical_current_ma,
                "speed[RPM]": self.historical_speed_rpm,
                "speed[cmd]": self.historical_speed_cmd}
        df = pd.DataFrame(data, columns=self.data_names)
        df.to_csv(self.file_path)
        if test_specs != None:
            specs = {"Specifications": test_specs}
            df = pd.DataFrame(specs, columns=["Specifications"])
            df.to_csv(self.file_name+".specs")